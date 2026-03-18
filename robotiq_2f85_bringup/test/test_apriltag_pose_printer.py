from pathlib import Path
import sys

import pytest

pytest.importorskip('apriltag_msgs.msg')
pytest.importorskip('geometry_msgs.msg')
pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Quaternion

from robotiq_2f85_bringup import apriltag_pose_printer


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message, **kwargs):
        self.infos.append(message)

    def warning(self, message, **kwargs):
        self.warnings.append(message)


def _detection(tag_id: int) -> AprilTagDetection:
    msg = AprilTagDetection()
    msg.id = int(tag_id)
    return msg


def test_format_pose_xyz():
    pose = type(
        'PoseStampedLike',
        (),
        {
            'pose': type(
                'PoseLike',
                (),
                {
                    'position': type(
                        'PointLike',
                        (),
                        {'x': 0.1234, 'y': 0.5, 'z': -0.25},
                    )(),
                    'orientation': Quaternion(),
                },
            )(),
        },
    )()

    formatted = apriltag_pose_printer._format_pose_xyz(pose)

    assert formatted == 'x=0.123 y=0.500 z=-0.250'


def test_detection_cb_latches_anchor_and_logs_selected_tag_positions():
    node = apriltag_pose_printer.AprilTagPosePrinter.__new__(
        apriltag_pose_printer.AprilTagPosePrinter
    )
    logger = _FakeLogger()
    published = []
    anchor_pose = type(
        'PoseStampedLike',
        (),
        {
            'pose': type(
                'PoseLike',
                (),
                {
                    'position': type(
                        'PointLike',
                        (),
                        {'x': 0.4, 'y': 0.5, 'z': 0.6},
                    )(),
                    'orientation': Quaternion(),
                },
            )(),
        },
    )()
    relative_pose = type(
        'PoseStampedLike',
        (),
        {
            'pose': type(
                'PoseLike',
                (),
                {
                    'position': type(
                        'PointLike',
                        (),
                        {'x': 0.1, 'y': 0.2, 'z': 0.3},
                    )(),
                    'orientation': Quaternion(),
                },
            )(),
        },
    )()
    node._tag_ids = {0, 1, 2, 3, 4}
    node._anchor_tag_id = 11
    node._anchor_parent_frame = 'base_link'
    node._anchor_child_frame = 'latched_tag_11'
    node._anchor_pose = None
    node._frames_to_log = ['latched_tag_11', 'base_link']
    node._should_log_tag = lambda tag_id: True
    node.get_logger = lambda: logger
    node._publish_anchor_transform = lambda: published.append(True)
    node._lookup_tag_pose = lambda target_frame, tag_frame: (
        anchor_pose
        if (target_frame, tag_frame) == ('base_link', 'tag_11')
        else relative_pose
        if (target_frame, tag_frame) == ('latched_tag_11', 'tag_2')
        else type(
            'PoseStampedLike',
            (),
            {
                'pose': type(
                    'PoseLike',
                    (),
                    {
                        'position': type(
                            'PointLike',
                            (),
                            {'x': 1.1, 'y': 1.2, 'z': 1.3},
                        )(),
                        'orientation': Quaternion(),
                    },
                )(),
            },
        )()
        if (target_frame, tag_frame) == ('base_link', 'tag_2')
        else None
    )

    detections = AprilTagDetectionArray()
    detections.detections = [_detection(11), _detection(2)]

    node._detection_cb(detections)

    assert published != []
    assert any('Latched tag_11 in base_link' in message for message in logger.infos)
    assert any('tag_2 ->' in message for message in logger.infos)
    assert any('latched_tag_11: x=0.100 y=0.200 z=0.300' in message for message in logger.infos)
    assert any('base_link: x=1.100 y=1.200 z=1.300' in message for message in logger.infos)


def test_detection_cb_warns_when_anchor_has_not_been_latched_yet():
    node = apriltag_pose_printer.AprilTagPosePrinter.__new__(
        apriltag_pose_printer.AprilTagPosePrinter
    )
    logger = _FakeLogger()
    node._tag_ids = {0, 1, 2, 3, 4}
    node._anchor_tag_id = 11
    node._anchor_child_frame = 'latched_tag_11'
    node._anchor_pose = None
    node._frames_to_log = ['latched_tag_11']
    node._should_log_tag = lambda tag_id: True
    node.get_logger = lambda: logger
    node._lookup_tag_pose = lambda target_frame, tag_frame: None

    detections = AprilTagDetectionArray()
    detections.detections = [_detection(1)]

    node._detection_cb(detections)

    assert logger.infos == []
    assert any('anchor tag_11 has not been latched yet' in message for message in logger.warnings)
