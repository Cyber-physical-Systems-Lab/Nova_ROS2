from pathlib import Path
import sys

import pytest

pytest.importorskip('geometry_msgs.msg')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import Point, PoseStamped, Quaternion

from robotiq_2f85_bringup import apriltag_arm_planner


def _pose(x: float, y: float, z: float, q: Quaternion | None = None) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position = Point(x=x, y=y, z=z)
    pose.pose.orientation = q if q is not None else Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return pose


@pytest.mark.parametrize(
    ('raw_value', 'expected'),
    [
        (-1.0, 0.0),
        (0.0, 0.0),
        (0.2, 0.2),
        (1.0, 1.0),
        (2.0, 1.0),
    ],
)
def test_clamp_alpha(raw_value, expected):
    assert apriltag_arm_planner._clamp_alpha(raw_value) == pytest.approx(expected)


def test_blend_quaternions_normalizes_output():
    previous = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    current = Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)

    blended = apriltag_arm_planner._blend_quaternions(previous, current, 0.5)
    norm = (
        (blended.x * blended.x)
        + (blended.y * blended.y)
        + (blended.z * blended.z)
        + (blended.w * blended.w)
    ) ** 0.5

    assert norm == pytest.approx(1.0)


def test_blend_quaternions_handles_opposite_hemisphere():
    previous = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    current = Quaternion(x=0.0, y=0.0, z=0.0, w=-1.0)

    blended = apriltag_arm_planner._blend_quaternions(previous, current, 0.5)

    assert blended.w == pytest.approx(1.0)


def test_blend_tag_pose_smooths_translation():
    previous = _pose(0.0, 0.0, 0.0)
    current = _pose(1.0, 2.0, 3.0)

    blended = apriltag_arm_planner._blend_tag_pose(previous, current, 0.25)

    assert blended.pose.position.x == pytest.approx(0.25)
    assert blended.pose.position.y == pytest.approx(0.5)
    assert blended.pose.position.z == pytest.approx(0.75)
