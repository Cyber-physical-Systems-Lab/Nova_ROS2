#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import rclpy.duration
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
import tf2_ros


def _coerce_tag_ids(value) -> set[int]:
    if value is None:
        return set()
    return {int(tag_id) for tag_id in value}


def _format_pose_xyz(pose: PoseStamped) -> str:
    return (
        f'x={pose.pose.position.x:.3f} '
        f'y={pose.pose.position.y:.3f} '
        f'z={pose.pose.position.z:.3f}'
    )


class AprilTagPosePrinter(Node):
    def __init__(self):
        super().__init__('apriltag_pose_printer')

        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('anchor_tag_id', 11)
        self.declare_parameter('anchor_parent_frame', 'base_link')
        self.declare_parameter('anchor_child_frame', 'latched_tag_11')
        self.declare_parameter('tag_ids', [0, 1, 2, 3, 4])
        self.declare_parameter('frames_to_log', ['latched_tag_11'])
        self.declare_parameter('log_period_s', 0.5)
        self.declare_parameter('anchor_publish_period_s', 0.2)

        self._detection_topic = str(self.get_parameter('detection_topic').value)
        self._anchor_tag_id = int(self.get_parameter('anchor_tag_id').value)
        self._anchor_parent_frame = str(
            self.get_parameter('anchor_parent_frame').value
        )
        self._anchor_child_frame = str(
            self.get_parameter('anchor_child_frame').value
        )
        self._tag_ids = _coerce_tag_ids(self.get_parameter('tag_ids').value)
        self._frames_to_log = [
            str(frame_id) for frame_id in self.get_parameter('frames_to_log').value
        ]
        self._log_period_s = max(0.0, float(self.get_parameter('log_period_s').value))
        self._anchor_publish_period_s = max(
            0.0, float(self.get_parameter('anchor_publish_period_s').value)
        )
        self._last_log_times_ns: dict[int, int] = {}
        self._anchor_pose: PoseStamped | None = None

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            AprilTagDetectionArray,
            self._detection_topic,
            self._detection_cb,
            10,
        )
        self.create_timer(self._anchor_publish_period_s, self._anchor_timer_cb)

        tag_list = sorted(self._tag_ids)
        self.get_logger().info(
            f'AprilTag pose printer ready on {self._detection_topic}; '
            f'logging tags {tag_list} in frames {self._frames_to_log}. '
            f'Anchor: tag_{self._anchor_tag_id} -> {self._anchor_child_frame} '
            f'({self._anchor_parent_frame}).'
        )

    def _should_log_tag(self, tag_id: int) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._last_log_times_ns.get(int(tag_id))
        if last_ns is not None and self._log_period_s > 0.0:
            elapsed_s = (now_ns - last_ns) / 1e9
            if elapsed_s < self._log_period_s:
                return False

        self._last_log_times_ns[int(tag_id)] = now_ns
        return True

    def _lookup_tag_pose(self, target_frame: str, tag_frame: str) -> PoseStamped | None:
        try:
            tf_msg = self._tf_buf.lookup_transform(
                target_frame,
                tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

        pose = PoseStamped()
        pose.header = tf_msg.header
        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = tf_msg.transform.translation.z
        pose.pose.orientation = tf_msg.transform.rotation
        return pose

    def _publish_anchor_transform(self) -> None:
        if self._anchor_pose is None:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._anchor_parent_frame
        tf_msg.child_frame_id = self._anchor_child_frame
        tf_msg.transform.translation.x = self._anchor_pose.pose.position.x
        tf_msg.transform.translation.y = self._anchor_pose.pose.position.y
        tf_msg.transform.translation.z = self._anchor_pose.pose.position.z
        tf_msg.transform.rotation = self._anchor_pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)

    def _anchor_timer_cb(self) -> None:
        self._publish_anchor_transform()

    def _update_anchor_pose(self) -> bool:
        anchor_frame = f'tag_{self._anchor_tag_id}'
        anchor_pose = self._lookup_tag_pose(self._anchor_parent_frame, anchor_frame)
        if anchor_pose is None:
            return False

        self._anchor_pose = anchor_pose
        self._publish_anchor_transform()
        self.get_logger().info(
            f'Latched {anchor_frame} in {self._anchor_parent_frame}: '
            f'{_format_pose_xyz(anchor_pose)}'
        )
        return True

    def _log_tag_pose(self, tag_id: int) -> None:
        tag_frame = f'tag_{int(tag_id)}'
        segments = []
        for target_frame in self._frames_to_log:
            pose = self._lookup_tag_pose(target_frame, tag_frame)
            if pose is None:
                continue
            segments.append(f'{target_frame}: {_format_pose_xyz(pose)}')

        if not segments:
            if (
                self._anchor_child_frame in self._frames_to_log
                and self._anchor_pose is None
            ):
                self.get_logger().warning(
                    f'Detected {tag_frame}, but anchor tag_{self._anchor_tag_id} '
                    'has not been latched yet.'
                )
                return
            self.get_logger().warning(
                f'Detected {tag_frame}, but no TF pose was available in '
                f'{self._frames_to_log}.'
            )
            return

        self.get_logger().info(f'{tag_frame} -> ' + ' | '.join(segments))

    def _detection_cb(self, msg: AprilTagDetectionArray) -> None:
        if not msg.detections:
            return

        detected_ids = {int(detection.id) for detection in msg.detections}
        if self._anchor_tag_id in detected_ids:
            self._update_anchor_pose()
        elif self._anchor_pose is not None:
            self._publish_anchor_transform()

        for detection in msg.detections:
            tag_id = int(detection.id)
            if tag_id == self._anchor_tag_id:
                continue
            if self._tag_ids and tag_id not in self._tag_ids:
                continue
            if not self._should_log_tag(tag_id):
                continue
            self._log_tag_pose(tag_id)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPosePrinter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
