from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotiq_2f85_bringup import web_command


@pytest.mark.parametrize(
    ('text', 'expected_kind', 'expected_value'),
    [
        ('gripper_pos:83', web_command.GRIPPER_POSITION_COMMAND, 83.0),
        ('gripper_pos: 83', web_command.GRIPPER_POSITION_COMMAND, 83.0),
        ('start_grasp', web_command.START_GRASP_COMMAND, None),
        ('stop_grasp', web_command.STOP_GRASP_COMMAND, None),
        ('go_home', web_command.GO_HOME_COMMAND, None),
    ],
)
def test_parse_web_command_accepts_supported_payloads(
    text,
    expected_kind,
    expected_value,
):
    command = web_command.parse_web_command(text)
    assert command.kind == expected_kind
    assert command.value == pytest.approx(expected_value)


@pytest.mark.parametrize(
    'text',
    [
        '',
        ' ',
        '83',
        'gripper_pos:',
        'start_grasp:',
        'stop_grasp:',
        'go_home:',
        'foo:1',
        'gripper_pos:abc',
        'start_grasp:1',
        'stop_grasp:1',
        'go_home:1',
        'start_pose',
        'start_pose:',
        'start_pose:1',
    ],
)
def test_parse_web_command_rejects_invalid_payloads(text):
    with pytest.raises(ValueError):
        web_command.parse_web_command(text)
