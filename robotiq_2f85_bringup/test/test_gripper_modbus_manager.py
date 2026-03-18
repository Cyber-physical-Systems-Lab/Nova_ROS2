import asyncio
from pathlib import Path
import sys

import pytest

pytest.importorskip('rclpy')
Time = pytest.importorskip('builtin_interfaces.msg').Time
pytest.importorskip('control_msgs.action')
pytest.importorskip('dobot_msgs_v3.srv')
pytest.importorskip('sensor_msgs.msg')
pytest.importorskip('std_msgs.msg')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotiq_2f85_bringup import gripper_modbus
from robotiq_2f85_bringup import gripper_modbus_manager


class _DummyLogger:
    def debug(self, *args, **kwargs):
        return None

    info = debug
    warning = debug
    error = debug


class _FakeFuture:
    def __init__(self):
        self._callbacks = []
        self._result = None
        self._exception = None
        self._done = False

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done:
            callback(self)

    def set_result(self, result):
        self._result = result
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def set_exception(self, exception):
        self._exception = exception
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def done(self):
        return self._done

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result


class _FakeClient:
    def __init__(self):
        self.requests = []
        self._queued_futures = []

    def queue_future(self, future):
        self._queued_futures.append(future)

    def call_async(self, request):
        self.requests.append(request)
        future = self._queued_futures.pop(0)
        return future


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeNow:
    def __init__(self, nanoseconds, stamp):
        self.nanoseconds = nanoseconds
        self._stamp = stamp

    def to_msg(self):
        return self._stamp


class _FakeClock:
    def __init__(self, *, nanoseconds=0, stamp=None):
        self.nanoseconds = nanoseconds
        self._stamp = stamp if stamp is not None else Time(sec=1, nanosec=2)

    def now(self):
        return _FakeNow(self.nanoseconds, self._stamp)


class _Response:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class _FakeTimerManagerNode:
    def __init__(self):
        self._modbus_host = '127.0.0.1'
        self._modbus_port = 60000
        self._slave_id = 9
        self._modbus_index = -1
        self._owns_modbus_channel = False
        self._create_future = None
        self._read_future = None
        self._close_future = None
        self._action_active = False
        self._next_create_attempt_ns = 0
        self._close_deadline_ns = 0
        self._modbus_create_cli = _FakeClient()
        self._get_hold_regs_cli = _FakeClient()
        self._modbus_close_cli = _FakeClient()
        self._gripper_joint_pub = _FakePublisher()
        self._web_feedback_pub = _FakePublisher()
        self._clock = _FakeClock()

    def get_logger(self):
        return _DummyLogger()

    def get_clock(self):
        return self._clock

    def _schedule_create_retry(self):
        return gripper_modbus_manager.GripperModbusManager._schedule_create_retry(self)

    def _clear_close_state(self, future=None):
        return gripper_modbus_manager.GripperModbusManager._clear_close_state(
            self,
            future,
        )

    def _invalidate_modbus_channel(self):
        return gripper_modbus_manager.GripperModbusManager._invalidate_modbus_channel(self)

    def _publish_joint_position(self, joint_position):
        return gripper_modbus_manager.GripperModbusManager._publish_joint_position(
            self,
            joint_position,
        )

    def _request_modbus_create(self):
        return gripper_modbus_manager.GripperModbusManager._request_modbus_create(
            self
        )

    def _handle_modbus_create_done(self, future):
        return gripper_modbus_manager.GripperModbusManager._handle_modbus_create_done(
            self,
            future,
        )

    def _request_feedback_read(self):
        return gripper_modbus_manager.GripperModbusManager._request_feedback_read(
            self
        )

    def _handle_feedback_read_done(self, future):
        return gripper_modbus_manager.GripperModbusManager._handle_feedback_read_done(
            self,
            future,
        )

    def _request_modbus_close_async(self, closing_index):
        return gripper_modbus_manager.GripperModbusManager._request_modbus_close_async(
            self,
            closing_index,
        )

    def _handle_modbus_close_done(self, future, closing_index):
        return gripper_modbus_manager.GripperModbusManager._handle_modbus_close_done(
            self,
            future,
            closing_index=closing_index,
        )


class _DummyGoalHandle:
    def __init__(self, position: float):
        self.request = type(
            'Request',
            (),
            {'command': type('Command', (), {'position': position})()},
        )()
        self.aborted = False
        self.succeeded = False

    def abort(self):
        self.aborted = True

    def succeed(self):
        self.succeeded = True


class _FakeManagerActionServer:
    def __init__(
        self,
        *,
        poll_feedback=None,
        write_ok=True,
        modbus_index=7,
        last_measured_joint_position=None,
    ):
        self._poll_feedback = poll_feedback
        self._write_ok = write_ok
        self._modbus_index = modbus_index
        self._last_measured_joint_position = last_measured_joint_position
        self._action_active = False
        self._create_future = None
        self._read_future = None
        self.invalidated = False
        self.write_calls = []
        self.published_positions = []

    def get_logger(self):
        return _DummyLogger()

    async def _wait_for_background_requests(self):
        return None

    async def _write_reg(self, addr, value):
        self.write_calls.append((addr, value))
        return self._write_ok

    def _publish_joint_position(self, joint_position):
        self.published_positions.append(joint_position)
        self._last_measured_joint_position = joint_position

    def _invalidate_modbus_channel(self):
        self.invalidated = True
        self._modbus_index = -1

    async def _poll_feedback_until_terminal(self):
        if isinstance(self._poll_feedback, Exception):
            raise self._poll_feedback
        self._publish_joint_position(self._poll_feedback.joint_position)
        return self._poll_feedback


def test_build_joint_state_message_uses_mimic_layout():
    stamp = Time(sec=1, nanosec=2)
    msg = gripper_modbus_manager._build_joint_state_message(0.25, stamp)

    assert msg.header.frame_id == 'joint_states'
    assert msg.header.stamp.sec == 1
    assert msg.header.stamp.nanosec == 2
    assert msg.name == gripper_modbus_manager._GRIPPER_JOINTS
    assert msg.position == pytest.approx([0.25, -0.25, 0.25, -0.25, -0.25, 0.25])


@pytest.mark.parametrize(
    ('joint_position', 'expected'),
    [
        (gripper_modbus._JOINT_OPEN, 100),
        (gripper_modbus._JOINT_CLOSED, 0),
        (gripper_modbus.reg_to_joint(0x44), 73),
    ],
)
def test_joint_position_to_web_feedback_percent(joint_position, expected):
    assert (
        gripper_modbus_manager._joint_position_to_web_feedback_percent(joint_position)
        == expected
    )


def test_build_web_feedback_message_uses_string_payload():
    msg = gripper_modbus_manager._build_web_feedback_message(gripper_modbus._JOINT_OPEN)

    assert msg.data == '100'


def test_publish_joint_position_publishes_joint_state_and_web_feedback():
    node = _FakeTimerManagerNode()
    expected_joint = gripper_modbus.reg_to_joint(0x44)

    node._publish_joint_position(expected_joint)

    assert len(node._gripper_joint_pub.messages) == 1
    assert len(node._web_feedback_pub.messages) == 1
    assert node._web_feedback_pub.messages[0].data == '73'


def test_feedback_timer_creates_channel_when_disconnected():
    node = _FakeTimerManagerNode()
    future = _FakeFuture()
    node._modbus_create_cli.queue_future(future)

    gripper_modbus_manager.GripperModbusManager._feedback_timer_cb(node)

    assert len(node._modbus_create_cli.requests) == 1
    assert node._create_future is future
    assert node._get_hold_regs_cli.requests == []


def test_modbus_create_done_stores_index():
    node = _FakeTimerManagerNode()
    future = _FakeFuture()
    node._modbus_create_cli.queue_future(future)

    gripper_modbus_manager.GripperModbusManager._feedback_timer_cb(node)
    future.set_result(_Response(res=0, index=7))

    assert node._create_future is None
    assert node._modbus_index == 7
    assert node._owns_modbus_channel is True


def test_modbus_create_failure_schedules_retry_without_blocking():
    node = _FakeTimerManagerNode()
    future = _FakeFuture()
    node._modbus_create_cli.queue_future(future)

    gripper_modbus_manager.GripperModbusManager._feedback_timer_cb(node)
    future.set_result(_Response(res=-1, index=''))

    assert node._create_future is None
    assert node._modbus_index == -1
    assert node._next_create_attempt_ns > 0


def test_feedback_read_done_publishes_joint_state():
    node = _FakeTimerManagerNode()
    node._modbus_index = 3
    future = _FakeFuture()
    node._get_hold_regs_cli.queue_future(future)

    gripper_modbus_manager.GripperModbusManager._feedback_timer_cb(node)
    future.set_result(_Response(res=0, value='63744, 32, 17424'))

    assert node._read_future is None
    assert len(node._gripper_joint_pub.messages) == 1
    assert len(node._web_feedback_pub.messages) == 1
    assert node._web_feedback_pub.messages[0].data == '73'
    msg = node._gripper_joint_pub.messages[0]
    expected_joint = gripper_modbus.reg_to_joint(0x44)
    assert msg.position == pytest.approx(
        [
            expected_joint,
            -expected_joint,
            expected_joint,
            -expected_joint,
            -expected_joint,
            expected_joint,
        ]
    )


def test_feedback_read_failure_invalidates_channel_and_schedules_retry():
    node = _FakeTimerManagerNode()
    node._modbus_index = 4
    node._owns_modbus_channel = True
    future = _FakeFuture()
    close_future = _FakeFuture()
    node._get_hold_regs_cli.queue_future(future)
    node._modbus_close_cli.queue_future(close_future)

    gripper_modbus_manager.GripperModbusManager._feedback_timer_cb(node)
    future.set_exception(RuntimeError('boom'))

    assert node._read_future is None
    assert node._modbus_index == -1
    assert node._next_create_attempt_ns > 0
    assert node._gripper_joint_pub.messages == []
    assert node._web_feedback_pub.messages == []
    assert len(node._modbus_close_cli.requests) == 1
    assert node._close_future is close_future


def test_invalidate_modbus_channel_does_not_close_reused_channel():
    node = _FakeTimerManagerNode()
    node._modbus_index = 6
    node._owns_modbus_channel = False

    node._invalidate_modbus_channel()

    assert node._modbus_close_cli.requests == []
    assert node._modbus_index == -1
    assert node._next_create_attempt_ns > 0


def test_feedback_timer_waits_for_pending_close_before_reconnect():
    node = _FakeTimerManagerNode()
    close_future = _FakeFuture()
    node._close_future = close_future
    node._close_deadline_ns = int(10 * 1e9)
    node._clock.nanoseconds = 1
    create_future = _FakeFuture()
    node._modbus_create_cli.queue_future(create_future)

    gripper_modbus_manager.GripperModbusManager._feedback_timer_cb(node)

    assert node._modbus_create_cli.requests == []
    assert node._get_hold_regs_cli.requests == []


def test_feedback_timer_recovers_after_close_timeout():
    node = _FakeTimerManagerNode()
    close_future = _FakeFuture()
    node._close_future = close_future
    node._close_deadline_ns = int(1 * 1e9)
    node._clock.nanoseconds = int(2 * 1e9)
    create_future = _FakeFuture()
    node._modbus_create_cli.queue_future(create_future)

    gripper_modbus_manager.GripperModbusManager._feedback_timer_cb(node)

    assert node._close_future is None
    assert len(node._modbus_create_cli.requests) == 1
    assert node._create_future is create_future


def test_request_modbus_close_waits_for_completion(monkeypatch):
    node = _FakeTimerManagerNode()
    node._modbus_index = 4
    node._owns_modbus_channel = True
    future = _FakeFuture()
    future.set_result(_Response(res=0))
    node._modbus_close_cli.queue_future(future)
    spin_calls = []

    monkeypatch.setattr(
        gripper_modbus_manager.rclpy,
        'spin_until_future_complete',
        lambda current_node, current_future, timeout_sec=None: spin_calls.append(
            (current_node, current_future, timeout_sec)
        ),
    )

    gripper_modbus_manager.GripperModbusManager._request_modbus_close(node)

    assert len(node._modbus_close_cli.requests) == 1
    assert node._modbus_index == -1
    assert node._owns_modbus_channel is False
    assert spin_calls == [
        (
            node,
            future,
            gripper_modbus_manager._MODBUS_CLOSE_TIMEOUT_S,
        )
    ]


def test_feedback_timer_does_not_create_while_action_is_active():
    node = _FakeTimerManagerNode()
    node._action_active = True
    future = _FakeFuture()
    node._modbus_create_cli.queue_future(future)

    gripper_modbus_manager.GripperModbusManager._feedback_timer_cb(node)

    assert node._modbus_create_cli.requests == []
    assert node._get_hold_regs_cli.requests == []


def test_execute_callback_aborts_when_manager_not_connected():
    server = _FakeManagerActionServer(modbus_index=-1)
    goal_handle = _DummyGoalHandle(gripper_modbus._JOINT_CLOSED)

    result = asyncio.run(
        gripper_modbus_manager.GripperModbusManager.execute_callback(server, goal_handle)
    )

    assert goal_handle.succeeded is False
    assert goal_handle.aborted is True
    assert result.reached_goal is False
    assert result.stalled is True
    assert server.write_calls == []


def test_execute_callback_uses_existing_startup_channel_without_creating_new_one():
    measured_joint = gripper_modbus.reg_to_joint(96)
    feedback = gripper_modbus.RobotiqFeedback(
        gobj=3,
        gpo=96,
        gflt=0,
        gpr=100,
        current=8,
        joint_position=measured_joint,
    )
    server = _FakeManagerActionServer(
        poll_feedback=feedback,
        modbus_index=7,
    )
    goal_handle = _DummyGoalHandle(gripper_modbus._JOINT_CLOSED)

    result = asyncio.run(
        gripper_modbus_manager.GripperModbusManager.execute_callback(server, goal_handle)
    )

    assert goal_handle.succeeded is True
    assert goal_handle.aborted is False
    assert result.reached_goal is True
    assert result.position == pytest.approx(measured_joint)
    assert server.published_positions == pytest.approx([measured_joint])
    assert server.write_calls == [
        (gripper_modbus._REG_ACTION, gripper_modbus._ACT_GOTO),
        (gripper_modbus._REG_POS, gripper_modbus.joint_to_reg(gripper_modbus._JOINT_CLOSED)),
        (gripper_modbus._REG_SPD_FRC, 65535),
    ]


def test_execute_callback_aborts_on_feedback_timeout_and_invalidates_channel():
    last_measured_joint = gripper_modbus.reg_to_joint(48)
    server = _FakeManagerActionServer(
        poll_feedback=TimeoutError('timed out'),
        modbus_index=7,
        last_measured_joint_position=last_measured_joint,
    )
    goal_handle = _DummyGoalHandle(gripper_modbus._JOINT_CLOSED)

    result = asyncio.run(
        gripper_modbus_manager.GripperModbusManager.execute_callback(server, goal_handle)
    )

    assert goal_handle.succeeded is False
    assert goal_handle.aborted is True
    assert result.reached_goal is False
    assert result.stalled is True
    assert result.position == pytest.approx(last_measured_joint)
    assert server.invalidated is True
