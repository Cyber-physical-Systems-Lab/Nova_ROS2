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

The node does not command robot motion. It only localizes detected tags in
map. Tag poses are smoothed per ID with a lightweight exponential filter so
the published map frames are less jittery than the raw detector output.
"""

import math

import rclpy
import rclpy.duration
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    Quaternion,
    TransformStamped,
)
from std_msgs.msg import Bool
import tf2_ros

BASE_LINK           = 'base_link'
MAP_FRAME           = 'map'
CAMERA_FRAME        = 'camera_link'


def _clamp_alpha(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _normalize_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def _blend_quaternions(previous: Quaternion, current: Quaternion, alpha: float) -> Quaternion:
    alpha = _clamp_alpha(alpha)
    px, py, pz, pw = previous.x, previous.y, previous.z, previous.w
    cx, cy, cz, cw = current.x, current.y, current.z, current.w

    # Keep the interpolation on one quaternion hemisphere.
    dot = (px * cx) + (py * cy) + (pz * cz) + (pw * cw)
    if dot < 0.0:
        cx, cy, cz, cw = -cx, -cy, -cz, -cw

    bx = ((1.0 - alpha) * px) + (alpha * cx)
    by = ((1.0 - alpha) * py) + (alpha * cy)
    bz = ((1.0 - alpha) * pz) + (alpha * cz)
    bw = ((1.0 - alpha) * pw) + (alpha * cw)
    nx, ny, nz, nw = _normalize_quaternion(bx, by, bz, bw)

    return Quaternion(x=nx, y=ny, z=nz, w=nw)


def _blend_tag_pose(previous: PoseStamped, current: PoseStamped, alpha: float) -> PoseStamped:
    alpha = _clamp_alpha(alpha)

    blended = PoseStamped()
    blended.header = current.header
    blended.pose.position = Point(
        x=((1.0 - alpha) * previous.pose.position.x) + (alpha * current.pose.position.x),
        y=((1.0 - alpha) * previous.pose.position.y) + (alpha * current.pose.position.y),
        z=((1.0 - alpha) * previous.pose.position.z) + (alpha * current.pose.position.z),
    )
    blended.pose.orientation = _blend_quaternions(
        previous.pose.orientation,
        current.pose.orientation,
        alpha,
    )
    return blended

# ── main node ─────────────────────────────────────────────────────────────────

class AprilTagArmPlanner(Node):

    def __init__(self):
        super().__init__('apriltag_arm_planner')

        self._enabled = True
        self.declare_parameter('pose_filter_alpha', 0.2)
        self._pose_filter_alpha = _clamp_alpha(
            self.get_parameter('pose_filter_alpha').value
        )
        self._filtered_tag_poses: dict[int, PoseStamped] = {}

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
            f'{CAMERA_FRAME} → {BASE_LINK} → {MAP_FRAME}. '
            f'Pose filter alpha={self._pose_filter_alpha:.2f}.'
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

            filtered_tag_map = self._filter_tag_pose(det.id, tag_map)
            self._publish_map_tag_tf(det.id, filtered_tag_map)
            self.get_logger().info(
                f'Localized tag_{det.id} in {MAP_FRAME}: '
                f'x={filtered_tag_map.pose.position.x:.3f}  '
                f'y={filtered_tag_map.pose.position.y:.3f}  '
                f'z={filtered_tag_map.pose.position.z:.3f}'
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

    def _filter_tag_pose(self, tag_id: int, current_pose: PoseStamped) -> PoseStamped:
        previous_pose = self._filtered_tag_poses.get(int(tag_id))
        if previous_pose is None or self._pose_filter_alpha >= 1.0:
            filtered_pose = current_pose
        else:
            filtered_pose = _blend_tag_pose(
                previous_pose,
                current_pose,
                self._pose_filter_alpha,
            )

        self._filtered_tag_poses[int(tag_id)] = filtered_pose
        return filtered_pose

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
