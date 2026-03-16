#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node


class GripperCmdSender(Node):
    def __init__(self):
        super().__init__('gripper_cmd_sender')

        self.declare_parameter('position', 100.0)
        self.declare_parameter('max_effort', 50.0)

        self._position = float(self.get_parameter('position').value)
        self._max_effort = float(self.get_parameter('max_effort').value)

        self._client = ActionClient(
            self,
            GripperCommand,
            '/gripper_position_controller/gripper_cmd',
        )

    def send_once(self):
        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting again...')

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = self._position
        goal_msg.command.max_effort = self._max_effort
        self.get_logger().info(
            f'Sending gripper goal position={self._position:.1f}% max_effort={self._max_effort:.1f}'
        )

        goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"Result reached_goal={result.reached_goal} stalled={result.stalled} effort={result.effort:.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperCmdSender()
    node.send_once()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
