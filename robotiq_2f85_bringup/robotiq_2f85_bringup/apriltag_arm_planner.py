#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag localization relay.

Behaviour
---------
1. Subscribes to /detections (apriltag_ros AprilTagDetectionArray).
2. For each detected tag, waits for apriltag_ros to publish the corresponding
   tag TF frame (for example tag_0).
3. Resolves that frame into the map frame through the robot's camera-linked TF
   chain and republishes it as a map-anchored frame.

The node does not command robot motion. It only localizes detected tags in map.
"""

import rclpy
import rclpy.duration
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    TransformStamped,
)
from std_msgs.msg import Bool
import tf2_ros

BASE_LINK           = 'base_link'
MAP_FRAME           = 'map'
CAMERA_FRAME        = 'camera_optical_frame'

# ── main node ─────────────────────────────────────────────────────────────────

class AprilTagArmPlanner(Node):

    def __init__(self):
        super().__init__('apriltag_arm_planner')

        self._enabled = True

        # TF
        self._tf_buf      = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self._detection_cb,
            10,
        )
        self.create_subscription(Bool, '/apriltag_planner/enable', self._enable_cb, 10)

        self.get_logger().info(
            f'AprilTag localization relay ready. Camera frame: '
            f'{CAMERA_FRAME} → {BASE_LINK} → {MAP_FRAME}.'
        )

    # ── enable/disable ────────────────────────────────────────────────────────

    def _enable_cb(self, msg: Bool) -> None:
        self._enabled = msg.data
        self.get_logger().info(f'Planner {"enabled" if self._enabled else "disabled"}.')

    # ── detection callback ────────────────────────────────────────────────────

    def _detection_cb(self, msg: AprilTagDetectionArray) -> None:
        if not self._enabled or not msg.detections:
            return

        for det in msg.detections:
            tag_frame = f'tag_{det.id}'
            tag_map = self._lookup_tag_pose_in_frame(tag_frame, MAP_FRAME)
            if tag_map is None:
                self.get_logger().warn(
                    f'Could not localize {tag_frame} in {MAP_FRAME}. '
                    f'Check that apriltag_ros is publishing tag TF frames and '
                    f'that the camera↔robot TF chain is connected.'
                )
                continue

            self._publish_map_tag_tf(det.id, tag_map)
            self.get_logger().info(
                f'Localized tag_{det.id} in {MAP_FRAME}: '
                f'x={tag_map.pose.position.x:.3f}  '
                f'y={tag_map.pose.position.y:.3f}  '
                f'z={tag_map.pose.position.z:.3f}'
            )

    # ── TF helper ─────────────────────────────────────────────────────────────

    def _lookup_tag_pose_in_frame(self, tag_frame: str, target_frame: str):
        try:
            tf_msg = self._tf_buf.lookup_transform(
                target_frame,
                tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(f'TF error for {tag_frame}: {exc}')
            return None

        tag_pose = PoseStamped()
        tag_pose.header = tf_msg.header
        tag_pose.pose.position = Point(
            x=tf_msg.transform.translation.x,
            y=tf_msg.transform.translation.y,
            z=tf_msg.transform.translation.z,
        )
        tag_pose.pose.orientation = tf_msg.transform.rotation
        return tag_pose

    def _publish_map_tag_tf(self, tag_id: int, tag_map: PoseStamped) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = MAP_FRAME
        tf_msg.child_frame_id = f'map_tag_{tag_id}'
        tf_msg.transform.translation.x = tag_map.pose.position.x
        tf_msg.transform.translation.y = tag_map.pose.position.y
        tf_msg.transform.translation.z = tag_map.pose.position.z
        tf_msg.transform.rotation = tag_map.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagArmPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
