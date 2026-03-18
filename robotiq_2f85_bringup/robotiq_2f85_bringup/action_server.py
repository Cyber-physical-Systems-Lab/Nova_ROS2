#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from action_msgs.msg import GoalStatus
import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from robotiq_2f85_bringup.gripper_modbus import _INTERNAL_ACTION_NAME
from robotiq_2f85_bringup.gripper_modbus import _JOINT_OPEN
from robotiq_2f85_bringup.gripper_modbus import _PUBLIC_ACTION_NAME


def _build_result(
    position: float,
    effort: float,
    stalled: bool,
    reached_goal: bool,
) -> GripperCommand.Result:
    result = GripperCommand.Result()
    result.position = float(position)
    result.effort = float(effort)
    result.stalled = bool(stalled)
    result.reached_goal = bool(reached_goal)
    return result


class Robotiq2F85ActionServer(Node):
    def __init__(self):
        super().__init__('robotiq_2f85_action_server')

        self._manager_client = ActionClient(
            self,
            GripperCommand,
            _INTERNAL_ACTION_NAME,
        )
        self._action_server = ActionServer(
            self,
            GripperCommand,
            _PUBLIC_ACTION_NAME,
            self.execute_callback,
        )

        self.get_logger().info(
            'Proxying public gripper goals to the shared gripper Modbus manager'
        )

    def _manager_available(self, timeout_sec: float = 1.0) -> bool:
        return self._manager_client.wait_for_server(timeout_sec=timeout_sec)

    async def _send_manager_goal(self, goal_msg: GripperCommand.Goal):
        return await self._manager_client.send_goal_async(goal_msg)

    async def _get_manager_result(self, goal_handle):
        return await goal_handle.get_result_async()

    async def execute_callback(self, goal_handle):
        target = float(goal_handle.request.command.position)
        max_effort = float(goal_handle.request.command.max_effort)

        self.get_logger().info(
            f'Forwarding goal position={target:.4f} rad max_effort={max_effort:.1f}'
        )

        if not self._manager_available():
            self.get_logger().error(
                'Gripper Modbus manager action server is not available'
            )
            goal_handle.abort()
            return _build_result(_JOINT_OPEN, 0.0, True, False)

        manager_goal = GripperCommand.Goal()
        manager_goal.command.position = target
        manager_goal.command.max_effort = max_effort

        try:
            manager_goal_handle = await self._send_manager_goal(manager_goal)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to forward gripper goal to the Modbus manager: {exc}'
            )
            goal_handle.abort()
            return _build_result(_JOINT_OPEN, 0.0, True, False)

        if manager_goal_handle is None or not manager_goal_handle.accepted:
            self.get_logger().error('Gripper Modbus manager rejected the goal')
            goal_handle.abort()
            return _build_result(_JOINT_OPEN, 0.0, True, False)

        try:
            wrapped_result = await self._get_manager_result(manager_goal_handle)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to receive a result from the Modbus manager: {exc}'
            )
            goal_handle.abort()
            return _build_result(_JOINT_OPEN, 0.0, True, False)

        result = wrapped_result.result if wrapped_result is not None else None
        if result is None:
            result = _build_result(_JOINT_OPEN, 0.0, True, False)

        status = (
            wrapped_result.status
            if wrapped_result is not None
            else GoalStatus.STATUS_UNKNOWN
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = Robotiq2F85ActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
