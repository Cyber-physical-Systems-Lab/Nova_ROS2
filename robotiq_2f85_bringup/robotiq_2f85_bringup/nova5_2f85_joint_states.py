#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge between dobot_bringup_v3's /joint_states_robot topic and MoveIt's
expected /joint_states topic for the Nova5 + Robotiq 2F-85 combined setup.

Why this node exists
--------------------
nova5_2f85_moveit does NOT use ros2_control mock hardware. Instead:
  - Arm joint feedback  : /joint_states_robot  (dobot_bringup_v3)
  - Gripper joint state : tracked from /gripper/joint_states if available,
                          otherwise held at last known position (default: open)

This node merges both into a single /joint_states message so robot_state_publisher
and move_group see consistent state for all joints (joint1–6 + gripper mimic set).

Gripper mimic multipliers (from nova5_2f85.urdf.xacro)
-------------------------------------------------------
  left_knuckle         = pos  * +1.0  (primary actuated joint)
  right_knuckle        = pos  * -1.0
  left_inner_knuckle   = pos  * +1.0
  right_inner_knuckle  = pos  * -1.0
  left_finger_tip      = pos  * -1.0
  right_finger_tip     = pos  * +1.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

GRIPPER_JOINTS = [
    'robotiq_85_left_knuckle_joint',
    'robotiq_85_right_knuckle_joint',
    'robotiq_85_left_inner_knuckle_joint',
    'robotiq_85_right_inner_knuckle_joint',
    'robotiq_85_left_finger_tip_joint',
    'robotiq_85_right_finger_tip_joint',
]

# Multipliers relative to left_knuckle (primary actuated joint)
GRIPPER_MIMIC = [1.0, -1.0, 1.0, -1.0, -1.0, 1.0]


class Nova52f85JointStates(Node):
    """Merge arm + gripper joint states and republish as /joint_states."""

    def __init__(self):
        super().__init__('nova5_2f85_joint_states')

        # Last known arm positions (radians).  Populated by /joint_states_robot.
        self._arm_positions = [0.0] * 6

        # Last known gripper left_knuckle position (radians). 0.0 = fully open.
        self._gripper_pos = 0.0

        # ── subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            JointState,
            '/joint_states_robot',
            self._robot_cb,
            10,
        )

        # Optional: track real gripper position if the action server
        # publishes joint states on /gripper/joint_states
        self.create_subscription(
            JointState,
            '/gripper/joint_states',
            self._gripper_cb,
            10,
        )

        # ── publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(JointState, 'joint_states', 10)

        self.get_logger().info(
            'nova5_2f85_joint_states: bridging /joint_states_robot → /joint_states '
            '(arm + gripper mimic joints)'
        )

    # ── callbacks ─────────────────────────────────────────────────────────

    def _publish_combined_state(self) -> None:
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'joint_states'

        # Build the complete message in Python lists first; ROS converts them
        # to the message field storage types on assignment.
        out.name = list(ARM_JOINTS) + GRIPPER_JOINTS
        out.position = list(self._arm_positions) + [
            self._gripper_pos * m for m in GRIPPER_MIMIC
        ]

        self._pub.publish(out)

    def _robot_cb(self, msg: JointState) -> None:
        """Receive real arm joint positions and republish combined state."""
        if len(msg.position) < 6:
            self.get_logger().warn(
                f'Expected ≥ 6 arm joints, got {len(msg.position)} – skipping.',
                throttle_duration_sec=5.0,
            )
            return

        self._arm_positions = list(msg.position[:6])
        self._publish_combined_state()

    def _gripper_cb(self, msg: JointState) -> None:
        """Track real gripper position if published."""
        primary = 'robotiq_85_left_knuckle_joint'
        name_list = list(msg.name)
        if primary in name_list:
            idx = name_list.index(primary)
            if idx < len(msg.position):
                self._gripper_pos = msg.position[idx]
                self._publish_combined_state()


def main(args=None):
    rclpy.init(args=args)
    node = Nova52f85JointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
