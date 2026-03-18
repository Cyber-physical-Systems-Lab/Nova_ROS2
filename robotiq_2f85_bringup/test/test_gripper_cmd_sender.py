from pathlib import Path
import sys

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('control_msgs.action')
pytest.importorskip('std_msgs.msg')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from std_msgs.msg import String

from robotiq_2f85_bringup import gripper_cmd_sender


@pytest.mark.parametrize(
    ('text', 'expected'),
    [
        ('gripper_pos:83', 83.0),
        ('gripper_pos: 83', 83.0),
    ],
)
def test_parse_web_command_accepts_numeric_text(text, expected):
    assert gripper_cmd_sender._parse_web_command(text) == pytest.approx(expected)

@pytest.mark.parametrize('text', ['start_grasp', 'stop_grasp'])
def test_parse_web_command_ignores_arm_task_commands(text):
    assert gripper_cmd_sender._parse_web_command(text) is None


@pytest.mark.parametrize('text', ['', ' ', 'open', '12 percent', '0', '50', 'start_pose'])
def test_parse_web_command_rejects_invalid_text(text):
    with pytest.raises(ValueError):
        gripper_cmd_sender._parse_web_command(text)


@pytest.mark.parametrize(
    ('percent', 'expected_reg_fraction'),
    [
        (0.0, 1.0),
        (50.0, 0.5),
        (100.0, 0.0),
        (-10.0, 1.0),
        (140.0, 0.0),
    ],
)
def test_percent_to_joint_clamps_into_gripper_range(percent, expected_reg_fraction):
    joint = gripper_cmd_sender._percent_to_joint(percent)
    expected = (
        gripper_cmd_sender._JOINT_OPEN
        + expected_reg_fraction
        * (gripper_cmd_sender._JOINT_CLOSED - gripper_cmd_sender._JOINT_OPEN)
    )
    assert joint == pytest.approx(expected)


class _FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message, **kwargs):
        self.warnings.append(message)


def test_web_command_cb_ignores_arm_task_commands_without_warning():
    node = gripper_cmd_sender.GripperCmdSender.__new__(gripper_cmd_sender.GripperCmdSender)
    logger = _FakeLogger()
    queued = []

    node._command_topic = '/web_command'
    node.get_logger = lambda: logger
    node._queue_command = lambda percent, source: queued.append((percent, source))

    node._web_command_cb(String(data='start_grasp'))
    node._web_command_cb(String(data='stop_grasp'))
    assert queued == []
    assert logger.warnings == []
