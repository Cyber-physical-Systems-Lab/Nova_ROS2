#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from control_msgs.action import GripperCommand
from dobot_msgs_v3.srv import ModbusClose, ModbusCreate, SetHoldRegs
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import JointState

_REG_ACTION = 1000
_REG_POS = 1001
_REG_SPD_FRC = 1002
_ACT_GOTO = 2304
_JOINT_OPEN = 0.0
_JOINT_CLOSED = 0.7929
_GRIPPER_JOINTS = [
    'robotiq_85_left_knuckle_joint',
    'robotiq_85_right_knuckle_joint',
    'robotiq_85_left_inner_knuckle_joint',
    'robotiq_85_right_inner_knuckle_joint',
    'robotiq_85_left_finger_tip_joint',
    'robotiq_85_right_finger_tip_joint',
]
_GRIPPER_MIMIC = [1.0, -1.0, 1.0, -1.0, -1.0, 1.0]

def _joint_to_reg(joint_position: float) -> int:
    clamped = max(_JOINT_OPEN, min(_JOINT_CLOSED, joint_position))
    normalized = (clamped - _JOINT_OPEN) / (_JOINT_CLOSED - _JOINT_OPEN)
    return int(normalized * 255)


class Robotiq2F85ActionServer(Node):
    def __init__(self):
        super().__init__('robotiq_2f85_action_server')

        self.declare_parameter('modbus_host', '127.0.0.1')
        self.declare_parameter('modbus_port', 60000)
        self.declare_parameter('slave_id', 9)
        self.declare_parameter('val_type', '')
        self.declare_parameter('modbus_create_retries', 5)
        self.declare_parameter('modbus_retry_delay_s', 0.4)

        self._modbus_host = self.get_parameter('modbus_host').value
        modbus_port_val = self.get_parameter('modbus_port').value
        self._modbus_port = int(modbus_port_val) if modbus_port_val is not None else 60000
        slave_id_val = self.get_parameter('slave_id').value
        self._slave_id = int(slave_id_val) if slave_id_val is not None else 9
        self._val_type = self.get_parameter('val_type').value
        modbus_create_retries_val = self.get_parameter('modbus_create_retries').value
        self._modbus_create_retries = int(modbus_create_retries_val) if modbus_create_retries_val is not None else 10
        modbus_retry_delay_val = self.get_parameter('modbus_retry_delay_s').value
        self._modbus_retry_delay_s = float(modbus_retry_delay_val) if modbus_retry_delay_val is not None else 0.4
        self._modbus_index = -1

        self._modbus_create_cli = self.create_client(ModbusCreate, '/dobot_bringup_v3/srv/ModbusCreate')
        self._set_hold_regs_cli = self.create_client(SetHoldRegs, '/dobot_bringup_v3/srv/SetHoldRegs')
        self._modbus_close_cli = self.create_client(ModbusClose, '/dobot_bringup_v3/srv/ModbusClose')
        self._gripper_joint_pub = self.create_publisher(JointState, '/gripper/joint_states', 10)
        while not self._modbus_create_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ModbusCreate service not available, waiting again...')
        while not self._set_hold_regs_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetHoldRegs service not available, waiting again...')
        while not self._modbus_close_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ModbusClose service not available, waiting again...')

        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_position_controller/gripper_cmd',
            self.execute_callback,
        )
        self.get_logger().info('GripperCommand action server is ready...')

    def _call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _create_modbus_channel(self) -> bool:
        # Clean up stale channels first, then retry creation.
        self._close_stale_channels()
        for attempt in range(1, self._modbus_create_retries + 1):
            req = ModbusCreate.Request()
            req.ip = self._modbus_host
            req.port = self._modbus_port
            req.slave_id = self._slave_id
            req.is_rtu = 1
            res = self._call_service(self._modbus_create_cli, req)
            if res is not None and res.res == 0 and str(res.index) != '':
                self._modbus_index = int(res.index)
                self.get_logger().info(f'Using Modbus index: {self._modbus_index}')
                return True
            self.get_logger().warning(f'ModbusCreate attempt {attempt}/{self._modbus_create_retries} failed: {res}')
            time.sleep(self._modbus_retry_delay_s)
        return False

    def _close_stale_channels(self):
        for idx in range(0, 4):
            try:
                req = ModbusClose.Request()
                req.index = idx
                self._call_service(self._modbus_close_cli, req)
            except Exception:
                pass

    def _write_reg(self, addr: int, value: int) -> bool:
        req = SetHoldRegs.Request()
        req.index = self._modbus_index
        req.addr = int(addr)
        req.count = 1
        req.val_tab = str(int(value))
        req.val_type = self._val_type
        res = self._call_service(self._set_hold_regs_cli, req)
        ok = res is not None and res.res == 0
        self.get_logger().info(f'SetHoldRegs addr={addr} value={value} res={res}')
        return ok

    def _publish_gripper_joint_state(self, joint_position: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joint_states'
        msg.name = list(_GRIPPER_JOINTS)
        msg.position = [joint_position * multiplier for multiplier in _GRIPPER_MIMIC]
        self._gripper_joint_pub.publish(msg)

    async def execute_callback(self, goal_handle):
        target = float(goal_handle.request.command.position)
        reg_pos = _joint_to_reg(target)
        self.get_logger().info(f'Received goal position={target:.1f}% -> reg={reg_pos}')

        if self._modbus_index < 0 and not self._create_modbus_channel():
            goal_handle.abort()
            result = GripperCommand.Result()
            result.position = target
            result.effort = 0.0
            result.stalled = True
            result.reached_goal = False
            return result

        ok = (
            self._write_reg(_REG_ACTION, _ACT_GOTO)
            and self._write_reg(_REG_POS, reg_pos)
            and self._write_reg(_REG_SPD_FRC, 65535)
        )
        time.sleep(1.0)
        if not ok:
            # Force a fresh ModbusCreate on next goal attempt.
            self._modbus_index = -1

        result = GripperCommand.Result()
        result.position = target
        result.effort = 255.0 if ok else 0.0
        result.stalled = not ok
        result.reached_goal = ok
        if ok:
            self._publish_gripper_joint_state(target)
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def destroy_node(self):
        if self._modbus_index >= 0:
            try:
                req = ModbusClose.Request()
                req.index = int(self._modbus_index)
                self._call_service(self._modbus_close_cli, req)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Robotiq2F85ActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
