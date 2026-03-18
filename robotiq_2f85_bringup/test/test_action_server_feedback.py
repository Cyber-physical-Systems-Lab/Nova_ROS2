import asyncio
from pathlib import Path
import sys

import pytest

pytest.importorskip('rclpy')
GoalStatus = pytest.importorskip('action_msgs.msg').GoalStatus
pytest.importorskip('control_msgs.action')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotiq_2f85_bringup import action_server
from robotiq_2f85_bringup import gripper_modbus


class _DummyLogger:
    def debug(self, *args, **kwargs):
        return None

    info = debug
    warning = debug
    error = debug


class _DummyGoalHandle:
    def __init__(self, position: float, max_effort: float = 50.0):
        self.request = type(
            'Request',
            (),
            {
                'command': type(
                    'Command',
                    (),
                    {
                        'position': position,
                        'max_effort': max_effort,
                    },
                )(),
            },
        )()
        self.aborted = False
        self.succeeded = False
        self.was_canceled = False

    def abort(self):
        self.aborted = True

    def succeed(self):
        self.succeeded = True

    def canceled(self):
        self.was_canceled = True


class _WrappedResult:
    def __init__(self, status, result):
        self.status = status
        self.result = result


class _FakeManagerGoalHandle:
    def __init__(self, *, accepted=True, wrapped_result=None, result_exception=None):
        self.accepted = accepted
        self._wrapped_result = wrapped_result
        self._result_exception = result_exception

    async def get_result_async(self):
        if self._result_exception is not None:
            raise self._result_exception
        return self._wrapped_result


class _FakeProxyActionServer:
    def __init__(
        self,
        *,
        manager_available=True,
        manager_goal_handle=None,
        send_exception=None,
    ):
        self._manager_is_available = manager_available
        self._manager_goal_handle = manager_goal_handle
        self._send_exception = send_exception
        self.forwarded_goals = []

    def get_logger(self):
        return _DummyLogger()

    def _manager_available(self, timeout_sec: float = 1.0):
        self.timeout_sec = timeout_sec
        return self._manager_is_available

    async def _send_manager_goal(self, goal_msg):
        self.forwarded_goals.append(goal_msg)
        if self._send_exception is not None:
            raise self._send_exception
        return self._manager_goal_handle

    async def _get_manager_result(self, goal_handle):
        return await goal_handle.get_result_async()


def _result(position=0.0, effort=0.0, stalled=False, reached_goal=False):
    return action_server._build_result(position, effort, stalled, reached_goal)


@pytest.mark.parametrize('reg_value', [0, 127, 255])
def test_joint_register_round_trip(reg_value):
    joint = gripper_modbus.reg_to_joint(reg_value)
    assert gripper_modbus.joint_to_reg(joint) == reg_value


@pytest.mark.parametrize(
    ('payload', 'expected'),
    [
        ('63744, 32, 17424', [0xF900, 0x0020, 0x4410]),
        ('0xF900 0x0020 0x4410', [0xF900, 0x0020, 0x4410]),
    ],
)
def test_parse_register_payload_accepts_decimal_and_hex(payload, expected):
    assert gripper_modbus.parse_register_payload(payload)[:3] == expected


@pytest.mark.parametrize('payload', ['0xF900 0x0020', 'hello,0x20,0x30'])
def test_parse_register_payload_rejects_bad_payloads(payload):
    with pytest.raises(ValueError):
        gripper_modbus.parse_register_payload(payload)


def test_decode_feedback_registers_extracts_gobj_and_gpo():
    feedback = gripper_modbus.decode_feedback_registers([0xF900, 0x0020, 0x4410])

    assert feedback.gobj == 3
    assert feedback.gflt == 0
    assert feedback.gpr == 0x20
    assert feedback.gpo == 0x44
    assert feedback.current == 0x10
    assert feedback.joint_position == pytest.approx(gripper_modbus.reg_to_joint(0x44))


def test_feedback_registers_look_like_robotiq_accepts_known_good_block():
    assert gripper_modbus.feedback_registers_look_like_robotiq([0xF900, 0x0020, 0x4410]) is True


@pytest.mark.parametrize(
    'registers',
    [
        [0x0000, 0x0000, 0x0000],
        [0xF901, 0x0020, 0x4410],
        [0xFD00, 0x0020, 0x4410],
        [0xF900, 0x1020, 0x4410],
    ],
)
def test_feedback_registers_look_like_robotiq_rejects_bad_blocks(registers):
    assert gripper_modbus.feedback_registers_look_like_robotiq(registers) is False


@pytest.mark.parametrize('gobj', [1, 2, 3])
def test_feedback_terminal_states(gobj):
    feedback = gripper_modbus.RobotiqFeedback(
        gobj=gobj,
        gpo=10,
        gflt=0,
        gpr=10,
        current=5,
        joint_position=gripper_modbus.reg_to_joint(10),
    )
    assert gripper_modbus.feedback_is_terminal(feedback) is True


def test_feedback_moving_state_is_not_terminal():
    feedback = gripper_modbus.RobotiqFeedback(
        gobj=0,
        gpo=10,
        gflt=0,
        gpr=10,
        current=5,
        joint_position=gripper_modbus.reg_to_joint(10),
    )
    assert gripper_modbus.feedback_is_terminal(feedback) is False


def test_execute_callback_forwards_goal_and_returns_manager_result():
    manager_result = _result(position=0.42, effort=255.0, stalled=False, reached_goal=True)
    server = _FakeProxyActionServer(
        manager_goal_handle=_FakeManagerGoalHandle(
            wrapped_result=_WrappedResult(GoalStatus.STATUS_SUCCEEDED, manager_result)
        )
    )
    goal_handle = _DummyGoalHandle(gripper_modbus._JOINT_CLOSED, max_effort=60.0)

    result = asyncio.run(
        action_server.Robotiq2F85ActionServer.execute_callback(server, goal_handle)
    )

    assert goal_handle.succeeded is True
    assert goal_handle.aborted is False
    assert result is manager_result
    assert len(server.forwarded_goals) == 1
    forwarded_goal = server.forwarded_goals[0]
    assert forwarded_goal.command.position == pytest.approx(gripper_modbus._JOINT_CLOSED)
    assert forwarded_goal.command.max_effort == pytest.approx(60.0)


def test_execute_callback_aborts_when_manager_is_unavailable():
    server = _FakeProxyActionServer(manager_available=False)
    goal_handle = _DummyGoalHandle(0.25)

    result = asyncio.run(
        action_server.Robotiq2F85ActionServer.execute_callback(server, goal_handle)
    )

    assert goal_handle.succeeded is False
    assert goal_handle.aborted is True
    assert result.reached_goal is False
    assert result.stalled is True
    assert server.forwarded_goals == []


def test_execute_callback_aborts_when_manager_rejects_goal():
    server = _FakeProxyActionServer(
        manager_goal_handle=_FakeManagerGoalHandle(accepted=False)
    )
    goal_handle = _DummyGoalHandle(0.25)

    result = asyncio.run(
        action_server.Robotiq2F85ActionServer.execute_callback(server, goal_handle)
    )

    assert goal_handle.succeeded is False
    assert goal_handle.aborted is True
    assert result.reached_goal is False
    assert result.stalled is True


def test_execute_callback_marks_canceled_when_manager_cancels():
    manager_result = _result(position=0.15, effort=10.0, stalled=False, reached_goal=False)
    server = _FakeProxyActionServer(
        manager_goal_handle=_FakeManagerGoalHandle(
            wrapped_result=_WrappedResult(GoalStatus.STATUS_CANCELED, manager_result)
        )
    )
    goal_handle = _DummyGoalHandle(0.15)

    result = asyncio.run(
        action_server.Robotiq2F85ActionServer.execute_callback(server, goal_handle)
    )

    assert goal_handle.was_canceled is True
    assert goal_handle.aborted is False
    assert goal_handle.succeeded is False
    assert result is manager_result


def test_execute_callback_aborts_when_manager_result_fails():
    server = _FakeProxyActionServer(
        manager_goal_handle=_FakeManagerGoalHandle(
            result_exception=RuntimeError('boom')
        )
    )
    goal_handle = _DummyGoalHandle(0.25)

    result = asyncio.run(
        action_server.Robotiq2F85ActionServer.execute_callback(server, goal_handle)
    )

    assert goal_handle.aborted is True
    assert result.reached_goal is False
    assert result.stalled is True
