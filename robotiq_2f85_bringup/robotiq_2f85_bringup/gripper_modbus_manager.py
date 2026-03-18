#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Single supported Robotiq Modbus owner for the current bringup.

This node is responsible for:
  - reusing a pre-existing Dobot Modbus channel when possible
  - creating a new channel only when needed
  - publishing /gripper/joint_states
  - publishing /web_feedback for web-facing gripper readback
  - serving the internal gripper action used by the public proxy
"""

from functools import partial

import rclpy
from control_msgs.action import GripperCommand
from dobot_msgs_v3.srv import GetHoldRegs, ModbusClose, ModbusCreate, SetHoldRegs
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robotiq_2f85_bringup.gripper_modbus import _ACT_GOTO
from robotiq_2f85_bringup.gripper_modbus import _FEEDBACK_POLL_INTERVAL_S
from robotiq_2f85_bringup.gripper_modbus import _FEEDBACK_REG_ADDR
from robotiq_2f85_bringup.gripper_modbus import _FEEDBACK_REG_COUNT
from robotiq_2f85_bringup.gripper_modbus import _FEEDBACK_TIMEOUT_S
from robotiq_2f85_bringup.gripper_modbus import _FEEDBACK_VAL_TYPE
from robotiq_2f85_bringup.gripper_modbus import _INTERNAL_ACTION_NAME
from robotiq_2f85_bringup.gripper_modbus import _JOINT_CLOSED
from robotiq_2f85_bringup.gripper_modbus import _JOINT_OPEN
from robotiq_2f85_bringup.gripper_modbus import _REG_ACTION
from robotiq_2f85_bringup.gripper_modbus import _REG_POS
from robotiq_2f85_bringup.gripper_modbus import _REG_SPD_FRC
from robotiq_2f85_bringup.gripper_modbus import RobotiqFeedback
from robotiq_2f85_bringup.gripper_modbus import decode_feedback_registers
from robotiq_2f85_bringup.gripper_modbus import feedback_is_terminal
from robotiq_2f85_bringup.gripper_modbus import joint_to_reg
from robotiq_2f85_bringup.gripper_modbus import parse_register_payload

_CREATE_RETRY_DELAY_S = 1.0
_MODBUS_CLOSE_TIMEOUT_S = 1.0
_GRIPPER_JOINTS = [
    'robotiq_85_left_knuckle_joint',
    'robotiq_85_right_knuckle_joint',
    'robotiq_85_left_inner_knuckle_joint',
    'robotiq_85_right_inner_knuckle_joint',
    'robotiq_85_left_finger_tip_joint',
    'robotiq_85_right_finger_tip_joint',
]
_GRIPPER_MIMIC = [1.0, -1.0, 1.0, -1.0, -1.0, 1.0]
_WEB_FEEDBACK_TOPIC = '/web_feedback'
_WEB_FEEDBACK_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


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


def _build_joint_state_message(joint_position: float, stamp) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.header.frame_id = 'joint_states'
    msg.name = list(_GRIPPER_JOINTS)
    msg.position = [joint_position * multiplier for multiplier in _GRIPPER_MIMIC]
    msg.velocity = []
    msg.effort = []
    return msg


def _joint_position_to_web_feedback_percent(joint_position: float) -> int:
    span = _JOINT_CLOSED - _JOINT_OPEN
    if span <= 0.0:
        return 0

    clamped = max(_JOINT_OPEN, min(_JOINT_CLOSED, float(joint_position)))
    normalized = (clamped - _JOINT_OPEN) / span
    return int(round((1.0 - normalized) * 100.0))


def _build_web_feedback_message(joint_position: float) -> String:
    msg = String()
    msg.data = str(_joint_position_to_web_feedback_percent(joint_position))
    return msg


class GripperModbusManager(Node):
    def __init__(self):
        super().__init__('gripper_modbus_manager')

        self.declare_parameter('modbus_host', '127.0.0.1')
        self.declare_parameter('modbus_port', 60000)
        self.declare_parameter('slave_id', 9)
        self.declare_parameter('modbus_index', -1)
        self.declare_parameter('val_type', '')
        self.declare_parameter('feedback_period_s', 0.1)

        self._modbus_host = str(self.get_parameter('modbus_host').value)
        modbus_port_value = self.get_parameter('modbus_port').value
        self._modbus_port = int(modbus_port_value) if modbus_port_value is not None else 60000
        slave_id_value = self.get_parameter('slave_id').value
        self._slave_id = int(slave_id_value) if slave_id_value is not None else 9
        modbus_index_value = self.get_parameter('modbus_index').value
        self._modbus_index = int(modbus_index_value) if modbus_index_value is not None else -1
        self._val_type = str(self.get_parameter('val_type').value)
        feedback_period_value = self.get_parameter('feedback_period_s').value
        self._feedback_period_s = float(feedback_period_value) if feedback_period_value is not None else 0.1

        self._owns_modbus_channel = self._modbus_index < 0
        self._last_measured_joint_position = None
        self._create_future = None
        self._read_future = None
        self._close_future = None
        self._action_active = False
        self._next_create_attempt_ns = 0
        self._close_deadline_ns = 0

        self._modbus_create_cli = self.create_client(
            ModbusCreate,
            '/dobot_bringup_v3/srv/ModbusCreate',
        )
        self._get_hold_regs_cli = self.create_client(
            GetHoldRegs,
            '/dobot_bringup_v3/srv/GetHoldRegs',
        )
        self._set_hold_regs_cli = self.create_client(
            SetHoldRegs,
            '/dobot_bringup_v3/srv/SetHoldRegs',
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
        self._web_feedback_pub = self.create_publisher(
            String,
            _WEB_FEEDBACK_TOPIC,
            _WEB_FEEDBACK_QOS,
        )

        while not self._modbus_create_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ModbusCreate service not available, waiting again...')
        while not self._get_hold_regs_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetHoldRegs service not available, waiting again...')
        while not self._set_hold_regs_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetHoldRegs service not available, waiting again...')
        while not self._modbus_close_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ModbusClose service not available, waiting again...')

        self._action_server = ActionServer(
            self,
            GripperCommand,
            _INTERNAL_ACTION_NAME,
            self.execute_callback,
        )
        self._feedback_timer = self.create_timer(
            self._feedback_period_s,
            self._feedback_timer_cb,
        )

        self.get_logger().info(
            'Shared gripper Modbus manager is ready '
            f'and will publish /gripper/joint_states and {_WEB_FEEDBACK_TOPIC} '
            f'every {self._feedback_period_s:.3f} s'
        )
        if self._modbus_index >= 0:
            self.get_logger().info(
                f'Reusing configured gripper Modbus index: {self._modbus_index}'
            )
        else:
            self._request_modbus_create()

    def _schedule_create_retry(self) -> None:
        self._next_create_attempt_ns = (
            self.get_clock().now().nanoseconds + int(_CREATE_RETRY_DELAY_S * 1e9)
        )

    def _clear_close_state(self, future=None) -> None:
        if future is not None and self._close_future is not future:
            return
        self._close_future = None
        self._close_deadline_ns = 0

    async def _sleep_ros(self, duration_s: float) -> None:
        future = Future()

        def complete_once() -> None:
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(duration_s, complete_once)
        try:
            await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    async def _call_service(self, client, request):
        return await client.call_async(request)

    def _publish_joint_position(self, joint_position: float) -> None:
        self._last_measured_joint_position = joint_position
        joint_msg = _build_joint_state_message(
            joint_position,
            self.get_clock().now().to_msg(),
        )
        self._gripper_joint_pub.publish(joint_msg)
        self._web_feedback_pub.publish(_build_web_feedback_message(joint_position))

    def _request_modbus_create(self) -> None:
        if self._modbus_index >= 0 or self._create_future is not None:
            return

        request = ModbusCreate.Request()
        request.ip = self._modbus_host
        request.port = self._modbus_port
        request.slave_id = self._slave_id
        request.is_rtu = 1

        try:
            self._create_future = self._modbus_create_cli.call_async(request)
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to request ModbusCreate for the gripper manager: {exc}',
                throttle_duration_sec=5.0,
            )
            self._create_future = None
            self._schedule_create_retry()
            return

        self._create_future.add_done_callback(self._handle_modbus_create_done)

    def _handle_modbus_create_done(self, future) -> None:
        self._create_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to create the gripper Modbus channel: {exc}',
                throttle_duration_sec=5.0,
            )
            self._schedule_create_retry()
            return

        if response is None or response.res != 0 or str(response.index) == '':
            self.get_logger().warning(
                f'Failed to create the gripper Modbus channel: {response}',
                throttle_duration_sec=5.0,
            )
            self._schedule_create_retry()
            return

        self._modbus_index = int(response.index)
        self._owns_modbus_channel = True
        self._next_create_attempt_ns = 0
        self.get_logger().info(
            f'Connected to the gripper on Modbus index {self._modbus_index}'
        )

    def _invalidate_modbus_channel(self) -> None:
        closing_index = self._modbus_index
        owns_channel = self._owns_modbus_channel
        self._modbus_index = -1
        self._owns_modbus_channel = False
        if owns_channel and closing_index >= 0:
            self._request_modbus_close_async(int(closing_index))
        self._schedule_create_retry()

    def _request_modbus_close(self) -> None:
        if self._modbus_index < 0 or not self._owns_modbus_channel:
            return

        request = ModbusClose.Request()
        request.index = int(self._modbus_index)
        closing_index = self._modbus_index
        self._modbus_index = -1
        self._owns_modbus_channel = False

        try:
            future = self._modbus_close_cli.call_async(request)
        except Exception as exc:
            self.get_logger().debug(
                f'Failed to request ModbusClose for index {closing_index}: {exc}'
            )
            return

        try:
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=_MODBUS_CLOSE_TIMEOUT_S,
            )
        except Exception as exc:
            self.get_logger().debug(
                f'Failed while waiting for ModbusClose on index {closing_index}: {exc}'
            )
            return

        if not future.done():
            self.get_logger().debug(
                f'ModbusClose timed out for index {closing_index} '
                f'after {_MODBUS_CLOSE_TIMEOUT_S:.1f} s'
            )
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().debug(
                f'ModbusClose failed for index {closing_index}: {exc}'
            )
            return

        if response is None or getattr(response, 'res', -1) != 0:
            self.get_logger().debug(
                f'ModbusClose returned non-success for index {closing_index}: {response}'
            )

    def _request_modbus_close_async(self, closing_index: int) -> None:
        if closing_index < 0 or self._close_future is not None:
            return

        request = ModbusClose.Request()
        request.index = int(closing_index)

        try:
            future = self._modbus_close_cli.call_async(request)
        except Exception as exc:
            self.get_logger().debug(
                f'Failed to request ModbusClose for index {closing_index}: {exc}'
            )
            return

        self._close_future = future
        self._close_deadline_ns = (
            self.get_clock().now().nanoseconds + int(_MODBUS_CLOSE_TIMEOUT_S * 1e9)
        )
        future.add_done_callback(
            partial(self._handle_modbus_close_done, closing_index=int(closing_index))
        )

    def _handle_modbus_close_done(self, future, *, closing_index: int) -> None:
        self._clear_close_state(future)

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().debug(
                f'ModbusClose failed for index {closing_index}: {exc}'
            )
            return

        if response is None or getattr(response, 'res', -1) != 0:
            self.get_logger().debug(
                f'ModbusClose returned non-success for index {closing_index}: {response}'
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

    async def _wait_for_background_requests(self) -> None:
        create_future = self._create_future
        if create_future is not None:
            try:
                await create_future
            except Exception:
                pass

        read_future = self._read_future
        if read_future is not None:
            try:
                await read_future
            except Exception:
                pass

    async def _write_reg(self, addr: int, value: int) -> bool:
        request = SetHoldRegs.Request()
        request.index = self._modbus_index
        request.addr = int(addr)
        request.count = 1
        request.val_tab = str(int(value))
        request.val_type = self._val_type

        response = await self._call_service(self._set_hold_regs_cli, request)
        success = response is not None and response.res == 0
        self.get_logger().info(
            f'SetHoldRegs addr={addr} value={value} res={response}'
        )
        return success

    async def _read_feedback(self) -> RobotiqFeedback:
        if self._modbus_index < 0:
            raise RuntimeError('No active Modbus channel for gripper feedback')

        request = GetHoldRegs.Request()
        request.index = int(self._modbus_index)
        request.addr = _FEEDBACK_REG_ADDR
        request.count = _FEEDBACK_REG_COUNT
        request.val_type = _FEEDBACK_VAL_TYPE

        response = await self._call_service(self._get_hold_regs_cli, request)
        if response is None or response.res != 0:
            raise RuntimeError(f'GetHoldRegs failed: {response}')

        registers = parse_register_payload(response.value)
        return decode_feedback_registers(registers)

    async def _poll_feedback_until_terminal(self) -> RobotiqFeedback:
        deadline_ns = self.get_clock().now().nanoseconds + int(_FEEDBACK_TIMEOUT_S * 1e9)
        last_feedback = None

        while self.get_clock().now().nanoseconds < deadline_ns:
            feedback = await self._read_feedback()
            self._publish_joint_position(feedback.joint_position)
            last_feedback = feedback

            if feedback_is_terminal(feedback):
                return feedback

            await self._sleep_ros(_FEEDBACK_POLL_INTERVAL_S)

        if last_feedback is None:
            raise TimeoutError('Timed out waiting for gripper feedback')

        raise TimeoutError(
            'Timed out waiting for terminal gripper feedback '
            f'(last gobj={last_feedback.gobj}, gpo={last_feedback.gpo})'
        )

    async def execute_callback(self, goal_handle):
        target = float(goal_handle.request.command.position)
        reg_pos = joint_to_reg(target)
        self.get_logger().info(
            f'Received manager goal position={target:.4f} rad -> reg={reg_pos}'
        )

        self._action_active = True
        try:
            await self._wait_for_background_requests()

            if self._modbus_index < 0:
                self.get_logger().error(
                    'Gripper Modbus manager is not connected yet; aborting goal'
                )
                goal_handle.abort()
                return _build_result(
                    self._last_measured_joint_position or _JOINT_OPEN,
                    0.0,
                    True,
                    False,
                )

            ok = (
                await self._write_reg(_REG_ACTION, _ACT_GOTO)
                and await self._write_reg(_REG_POS, reg_pos)
                and await self._write_reg(_REG_SPD_FRC, 65535)
            )
            if not ok:
                self.get_logger().error('Failed to send the gripper Modbus write sequence')
                self._invalidate_modbus_channel()
                goal_handle.abort()
                return _build_result(
                    self._last_measured_joint_position or _JOINT_OPEN,
                    0.0,
                    True,
                    False,
                )

            try:
                feedback = await self._poll_feedback_until_terminal()
            except (RuntimeError, ValueError, TimeoutError) as exc:
                self.get_logger().error(f'Failed to read gripper feedback: {exc}')
                self._invalidate_modbus_channel()
                goal_handle.abort()
                return _build_result(
                    self._last_measured_joint_position or _JOINT_OPEN,
                    0.0,
                    True,
                    False,
                )
        finally:
            self._action_active = False

        goal_handle.succeed()
        return _build_result(feedback.joint_position, 255.0, False, True)

    def _feedback_timer_cb(self) -> None:
        if self._action_active:
            return

        now_ns = self.get_clock().now().nanoseconds

        if self._close_future is not None:
            if self._close_future.done():
                self._clear_close_state(self._close_future)
            elif now_ns < self._close_deadline_ns:
                return
            else:
                self.get_logger().warning(
                    'Timed out waiting for ModbusClose during gripper recovery; '
                    'continuing with reconnect attempts',
                    throttle_duration_sec=5.0,
                )
                self._clear_close_state(self._close_future)

        if (
            self._create_future is not None
            or self._read_future is not None
        ):
            return

        if self._modbus_index >= 0:
            self._request_feedback_read()
            return

        if now_ns >= self._next_create_attempt_ns:
            self._request_modbus_create()

    def destroy_node(self):
        self._feedback_timer.cancel()
        self._request_modbus_close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperModbusManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
