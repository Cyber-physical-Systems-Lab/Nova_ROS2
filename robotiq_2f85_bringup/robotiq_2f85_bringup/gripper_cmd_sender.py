#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from robotiq_2f85_bringup.web_command import extract_gripper_percent
from robotiq_2f85_bringup.web_command import parse_web_command

_JOINT_OPEN = 0.0
_JOINT_CLOSED = 0.7929


def _percent_to_joint(percent: float) -> float:
    clamped = max(0.0, min(100.0, float(percent)))
    normalized = 1.0 - (clamped / 100.0)
    return _JOINT_OPEN + normalized * (_JOINT_CLOSED - _JOINT_OPEN)


def _parse_web_command(text: str) -> float | None:
    command = parse_web_command(text)
    return extract_gripper_percent(command)


class GripperCmdSender(Node):
    def __init__(self):
        super().__init__('gripper_cmd_sender')

        self.declare_parameter('position', -1.0)
        self.declare_parameter('max_effort', 50.0)
        self.declare_parameter('command_topic', '/web_command')

        self._startup_position = float(self.get_parameter('position').value)
        self._max_effort = float(self.get_parameter('max_effort').value)
        self._command_topic = str(self.get_parameter('command_topic').value)

        self._client = ActionClient(
            self,
            GripperCommand,
            '/gripper_position_controller/gripper_cmd',
        )

        self._busy = False
        self._pending_percent = None
        self._active_percent = None

        self.create_subscription(
            String,
            self._command_topic,
            self._web_command_cb,
            10,
        )

        self.get_logger().info(
            f'Listening for web gripper commands on {self._command_topic} '
            '(gripper_pos:<percent>, 0=closed, 100=open)'
        )

        if self._startup_position >= 0.0:
            self._queue_command(self._startup_position, source='startup parameter')

    def _queue_command(self, percent: float, source: str) -> None:
        clamped = max(0.0, min(100.0, float(percent)))
        if not math.isclose(clamped, float(percent), rel_tol=0.0, abs_tol=1e-9):
            self.get_logger().info(
                f'Clamped {source} command from {float(percent):.2f} to {clamped:.2f}'
            )

        self._pending_percent = clamped
        self.get_logger().info(
            f'Queued gripper command from {source}: {clamped:.2f}%'
        )

        if not self._busy:
            self._dispatch_pending_goal()

    def _dispatch_pending_goal(self) -> None:
        if self._busy or self._pending_percent is None:
            return

        percent = self._pending_percent
        self._pending_percent = None

        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting again...')

        joint_position = _percent_to_joint(percent)

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = joint_position
        goal_msg.command.max_effort = self._max_effort

        self._busy = True
        self._active_percent = percent

        self.get_logger().info(
            f'Sending gripper goal from web command {percent:.2f}% '
            f'-> {joint_position:.4f} rad'
        )

        goal_future = self._client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send gripper goal: {exc}')
            self._busy = False
            self._active_percent = None
            self._dispatch_pending_goal()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            self._busy = False
            self._active_percent = None
            self._dispatch_pending_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f'Failed to receive gripper result: {exc}')
        else:
            active_percent = (
                self._active_percent if self._active_percent is not None else float('nan')
            )
            self.get_logger().info(
                f'Gripper command {active_percent:.2f}% completed with '
                f'position={result.position:.4f} rad '
                f'reached_goal={result.reached_goal} '
                f'stalled={result.stalled} '
                f'effort={result.effort:.1f}'
            )

        self._busy = False
        self._active_percent = None
        self._dispatch_pending_goal()

    def _web_command_cb(self, msg: String) -> None:
        try:
            command = parse_web_command(msg.data)
        except ValueError as exc:
            self.get_logger().warning(
                f'Ignoring invalid /web_command payload {msg.data!r}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        percent = extract_gripper_percent(command)
        if percent is None:
            return

        self._queue_command(percent, source=self._command_topic)


def main(args=None):
    rclpy.init(args=args)
    node = GripperCmdSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
