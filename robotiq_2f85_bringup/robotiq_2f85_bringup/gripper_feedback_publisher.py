#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Legacy standalone gripper feedback poller.

The supported bringup path now uses `gripper_modbus_manager` as the single
Robotiq Modbus owner. This module remains in-tree for reference and ad hoc use,
but it is not exported as a console script and is not launched by the current
bringup.
"""

import rclpy
from dobot_msgs_v3.srv import GetHoldRegs, ModbusClose, ModbusCreate
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robotiq_2f85_bringup.gripper_modbus import _FEEDBACK_REG_ADDR
from robotiq_2f85_bringup.gripper_modbus import _FEEDBACK_REG_COUNT
from robotiq_2f85_bringup.gripper_modbus import _FEEDBACK_VAL_TYPE
from robotiq_2f85_bringup.gripper_modbus import decode_feedback_registers
from robotiq_2f85_bringup.gripper_modbus import parse_register_payload

_GRIPPER_JOINTS = [
    'robotiq_85_left_knuckle_joint',
    'robotiq_85_right_knuckle_joint',
    'robotiq_85_left_inner_knuckle_joint',
    'robotiq_85_right_inner_knuckle_joint',
    'robotiq_85_left_finger_tip_joint',
    'robotiq_85_right_finger_tip_joint',
]
_GRIPPER_MIMIC = [1.0, -1.0, 1.0, -1.0, -1.0, 1.0]


def _build_joint_state_message(joint_position: float, stamp) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.header.frame_id = 'joint_states'
    msg.name = list(_GRIPPER_JOINTS)
    msg.position = [joint_position * multiplier for multiplier in _GRIPPER_MIMIC]
    msg.velocity = []
    msg.effort = []
    return msg


class GripperFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('gripper_feedback_publisher')

        self.declare_parameter('modbus_host', '127.0.0.1')
        self.declare_parameter('modbus_port', 60000)
        self.declare_parameter('slave_id', 9)
        self.declare_parameter('feedback_period_s', 0.1)

        self._modbus_host = str(self.get_parameter('modbus_host').value)
        modbus_port_value = self.get_parameter('modbus_port').value
        self._modbus_port = int(modbus_port_value) if modbus_port_value is not None else 60000
        slave_id_value = self.get_parameter('slave_id').value
        self._slave_id = int(slave_id_value) if slave_id_value is not None else 9
        period_value = self.get_parameter('feedback_period_s').value
        self._feedback_period_s = float(period_value) if period_value is not None else 0.1

        self._modbus_index = -1
        self._create_future = None
        self._read_future = None

        self._modbus_create_cli = self.create_client(
            ModbusCreate,
            '/dobot_bringup_v3/srv/ModbusCreate',
        )
        self._get_hold_regs_cli = self.create_client(
            GetHoldRegs,
            '/dobot_bringup_v3/srv/GetHoldRegs',
        )
        self._modbus_close_cli = self.create_client(
            ModbusClose,
            '/dobot_bringup_v3/srv/ModbusClose',
        )
        self._gripper_joint_pub = self.create_publisher(
            JointState,
            '/gripper/joint_states',
            10,
        )

        while not self._modbus_create_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ModbusCreate service not available, waiting again...')
        while not self._get_hold_regs_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetHoldRegs service not available, waiting again...')
        while not self._modbus_close_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ModbusClose service not available, waiting again...')

        self._timer = self.create_timer(self._feedback_period_s, self._timer_cb)

        self.get_logger().info(
            'Publishing periodic gripper feedback to /gripper/joint_states '
            f'every {self._feedback_period_s:.3f} s'
        )

    def _invalidate_modbus_channel(self) -> None:
        self._modbus_index = -1

    def _request_modbus_create(self) -> None:
        request = ModbusCreate.Request()
        request.ip = self._modbus_host
        request.port = self._modbus_port
        request.slave_id = self._slave_id
        request.is_rtu = 1

        try:
            self._create_future = self._modbus_create_cli.call_async(request)
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to request feedback ModbusCreate: {exc}',
                throttle_duration_sec=5.0,
            )
            self._create_future = None
            return
        self._create_future.add_done_callback(self._handle_modbus_create_done)

    def _handle_modbus_create_done(self, future) -> None:
        self._create_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to create feedback Modbus channel: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        if response is None or response.res != 0 or str(response.index) == '':
            self.get_logger().warning(
                f'Failed to create feedback Modbus channel: {response}',
                throttle_duration_sec=5.0,
            )
            return

        self._modbus_index = int(response.index)
        self.get_logger().info(
            f'Using Modbus index {self._modbus_index} for feedback polling'
        )

    def _request_feedback_read(self) -> None:
        request = GetHoldRegs.Request()
        request.index = self._modbus_index
        request.addr = _FEEDBACK_REG_ADDR
        request.count = _FEEDBACK_REG_COUNT
        request.val_type = _FEEDBACK_VAL_TYPE

        try:
            self._read_future = self._get_hold_regs_cli.call_async(request)
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to request periodic gripper feedback: {exc}',
                throttle_duration_sec=5.0,
            )
            self._read_future = None
            self._invalidate_modbus_channel()
            return
        self._read_future.add_done_callback(self._handle_feedback_read_done)

    def _handle_feedback_read_done(self, future) -> None:
        self._read_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read periodic gripper feedback: {exc}',
                throttle_duration_sec=5.0,
            )
            self._invalidate_modbus_channel()
            return

        if response is None or response.res != 0:
            self.get_logger().warning(
                f'Failed to read periodic gripper feedback: {response}',
                throttle_duration_sec=5.0,
            )
            self._invalidate_modbus_channel()
            return

        try:
            registers = parse_register_payload(response.value)
            feedback = decode_feedback_registers(registers)
        except ValueError as exc:
            self.get_logger().warning(
                f'Failed to decode periodic gripper feedback: {exc}',
                throttle_duration_sec=5.0,
            )
            self._invalidate_modbus_channel()
            return

        self._publish_joint_position(feedback.joint_position)

    def _publish_joint_position(self, joint_position: float) -> None:
        msg = _build_joint_state_message(
            joint_position,
            self.get_clock().now().to_msg(),
        )
        self._gripper_joint_pub.publish(msg)

    def _request_modbus_close(self) -> None:
        if self._modbus_index < 0:
            return

        request = ModbusClose.Request()
        request.index = int(self._modbus_index)
        closing_index = self._modbus_index
        self._modbus_index = -1

        if not rclpy.ok():
            return

        try:
            future = self._modbus_close_cli.call_async(request)
        except Exception as exc:
            self.get_logger().debug(
                f'Failed to request ModbusClose for feedback index {closing_index}: {exc}'
            )
            return

        def log_close_result(done_future) -> None:
            try:
                done_future.result()
            except Exception as exc:
                self.get_logger().debug(
                    f'ModbusClose failed for feedback index {closing_index}: {exc}'
                )

        future.add_done_callback(log_close_result)

    def _timer_cb(self) -> None:
        if self._create_future is not None or self._read_future is not None:
            return

        if self._modbus_index < 0:
            self._request_modbus_create()
            return

        self._request_feedback_read()

    def destroy_node(self):
        self._timer.cancel()
        self._request_modbus_close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperFeedbackPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
