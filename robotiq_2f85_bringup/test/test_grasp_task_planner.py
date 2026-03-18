from pathlib import Path
import sys

import pytest

pytest.importorskip('control_msgs.action')
pytest.importorskip('geometry_msgs.msg')
pytest.importorskip('moveit_msgs.action')
pytest.importorskip('moveit_msgs.srv')
pytest.importorskip('rclpy')
pytest.importorskip('std_msgs.msg')
pytest.importorskip('tf2_ros')
pytest.importorskip('trajectory_msgs.msg')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath
from std_msgs.msg import Header, Int32
from trajectory_msgs.msg import JointTrajectoryPoint

from robotiq_2f85_bringup import grasp_task_planner


def _pose(
    x: float,
    y: float,
    z: float,
    frame_id: str = 'base_link',
    quaternion: Quaternion | None = None,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position = Point(x=x, y=y, z=z)
    pose.pose.orientation = quaternion or Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    return pose


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message, **kwargs):
        self.infos.append(message)

    def warning(self, message, **kwargs):
        self.warnings.append(message)

    def error(self, message, **kwargs):
        self.errors.append(message)


class _FakeActionClient:
    def __init__(self):
        self.wait_calls = 0
        self.sent_goals = []

    def wait_for_server(self, timeout_sec):
        self.wait_calls += 1
        return True

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)

        class _Future:
            def add_done_callback(self, callback):
                self.callback = callback

        return _Future()


class _FakeServiceClient:
    def __init__(self):
        self.wait_calls = 0
        self.requests = []

    def wait_for_service(self, timeout_sec):
        self.wait_calls += 1
        return True

    def call_async(self, request):
        self.requests.append(request)

        class _Future:
            def add_done_callback(self, callback):
                self.callback = callback

        return _Future()


class _FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


def _execute_future(error_code: int = MoveItErrorCodes.SUCCESS):
    class _ExecuteFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {'error_code': MoveItErrorCodes(val=error_code)},
            )()
            return wrapper

    return _ExecuteFuture()


def _gripper_future(
    position: float = 0.0,
    reached_goal: bool = True,
    stalled: bool = False,
    effort: float = 10.0,
):
    class _GripperFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {
                    'position': position,
                    'reached_goal': reached_goal,
                    'stalled': stalled,
                    'effort': effort,
                },
            )()
            return wrapper

    return _GripperFuture()


def _make_node():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()

    node._state = grasp_task_planner.STATE_IDLE
    node._active_motion = grasp_task_planner.MOTION_NONE
    node._selected_object_id = None
    node._active_target_pose = None
    node._pending_grasp_pose = None
    node._pending_pre_grasp_pose = None
    node._pre_grasp_settle_timer = None
    node._pre_grasp_settle_time_s = 0.0
    node._pre_grasp_offset_m = 0.08
    node._lift_height_m = 0.08
    node._orientation_tolerance_rad = 0.01
    node._execute_timeout_position_tolerance_m = 0.02
    node._target_reached_tolerance_m = 0.01
    node._planning_group = 'nova5_group'
    node._base_frame = 'base_link'
    node._tip_link = 'tool_link'
    node._cartesian_max_step_m = 0.005
    node._cartesian_jump_threshold = 0.0
    node._cartesian_prismatic_jump_threshold = 0.0
    node._cartesian_revolute_jump_threshold = 0.0
    node._cartesian_min_fraction = 0.99
    node._cartesian_avoid_collisions = True
    node._cartesian_path_service = '/compute_cartesian_path'
    node._execute_trajectory_action = '/execute_trajectory'
    node._gripper_action = '/gripper_position_controller/gripper_cmd'
    node._auto_open_percent = 100.0
    node._gripper_max_effort = 50.0
    node._drop_pose = _pose(-0.5, 0.0, 0.15)
    node._home_pose = _pose(0.04, 0.3, 0.3)
    node._grasp_poses = {
        idx: _pose(-0.2 + (idx * 0.05), 0.4, 0.23)
        for idx in range(1, 6)
    }
    node._cartesian_path_client = _FakeServiceClient()
    node._execute_trajectory_client = _FakeActionClient()
    node._gripper_client = _FakeActionClient()
    node.create_timer = lambda period, callback: _FakeTimer(callback)
    node.get_logger = lambda: logger

    return node, logger


def test_build_target_pose_changes_only_z():
    current_pose = _pose(0.3, -0.1, 0.4)

    target_pose = grasp_task_planner._build_target_pose(current_pose, 0.08)

    assert target_pose.pose.position.x == pytest.approx(0.3)
    assert target_pose.pose.position.y == pytest.approx(-0.1)
    assert target_pose.pose.position.z == pytest.approx(0.48)


def test_compose_pose_applies_transform_offset():
    reference_pose = _pose(
        0.30,
        0.10,
        0.20,
        quaternion=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    relative_pose = _pose(0.18, 0.08, 0.02, frame_id='camera_link')

    composed = grasp_task_planner._compose_pose(reference_pose, relative_pose)

    assert composed.header.frame_id == 'base_link'
    assert composed.pose.position.x == pytest.approx(0.48)
    assert composed.pose.position.y == pytest.approx(0.18)
    assert composed.pose.position.z == pytest.approx(0.22)


def test_resolve_pose_in_base_frame_transforms_non_base_pose():
    node, logger = _make_node()
    node.get_logger = lambda: logger
    node._tf_buf = type(
        'Buffer',
        (),
        {
            'lookup_transform': staticmethod(
                lambda target, source, time, timeout: type(
                    'TF',
                    (),
                    {
                        'header': Header(frame_id='base_link'),
                        'transform': type(
                            'Transform',
                            (),
                            {
                                'translation': type(
                                    'Translation',
                                    (),
                                    {'x': 0.1, 'y': 0.2, 'z': 0.3},
                                )(),
                                'rotation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                            },
                        )(),
                    },
                )()
            )
        },
    )()
    pose = _pose(0.2, 0.3, 0.4, frame_id='camera_link')

    resolved = node._resolve_pose_in_base_frame(pose)

    assert resolved is not None
    assert resolved.header.frame_id == 'base_link'
    assert resolved.pose.position.x == pytest.approx(0.3)
    assert resolved.pose.position.y == pytest.approx(0.5)
    assert resolved.pose.position.z == pytest.approx(0.7)


def test_build_cartesian_path_request_keeps_orientation_constraint():
    target_pose = _pose(0.48, 0.18, 0.22)
    current_pose = _pose(0.48, 0.18, 0.30)

    request = grasp_task_planner._build_cartesian_path_request(
        current_pose=current_pose,
        target_pose=target_pose,
        planning_group='nova5_group',
        base_frame='base_link',
        tip_link='tool_link',
        orientation_tolerance_rad=0.05,
        max_step_m=0.005,
        jump_threshold=0.0,
        prismatic_jump_threshold=0.0,
        revolute_jump_threshold=0.0,
        avoid_collisions=True,
    )

    assert request.path_constraints.orientation_constraints != []
    assert request.waypoints[0].position.z == pytest.approx(0.22)


def test_derive_pre_grasp_pose_preserves_quaternion_and_offsets_z():
    final_grasp_pose = _pose(
        0.48,
        0.18,
        0.22,
        quaternion=Quaternion(x=0.2, y=0.3, z=0.4, w=0.5),
    )

    pre_grasp_pose = grasp_task_planner._derive_pre_grasp_pose(final_grasp_pose, 0.08)

    assert pre_grasp_pose.pose.position.x == pytest.approx(0.48)
    assert pre_grasp_pose.pose.position.y == pytest.approx(0.18)
    assert pre_grasp_pose.pose.position.z == pytest.approx(0.30)
    assert pre_grasp_pose.pose.orientation.x == pytest.approx(0.2)
    assert pre_grasp_pose.pose.orientation.y == pytest.approx(0.3)
    assert pre_grasp_pose.pose.orientation.z == pytest.approx(0.4)
    assert pre_grasp_pose.pose.orientation.w == pytest.approx(0.5)


def test_object_id_cb_accepts_ids_1_to_5():
    node, logger = _make_node()

    node._object_id_cb(Int32(data=3))

    assert node._selected_object_id == 3
    assert any('Selected grasp object id 3' in message for message in logger.infos)


def test_start_grasp_requires_selected_object():
    node, logger = _make_node()

    node._start_grasp()

    assert any('/grasp_object_id' in message for message in logger.warnings)


def test_start_grasp_sends_selected_pre_grasp_goal():
    node, logger = _make_node()
    grasp_pose = _pose(
        0.48,
        0.18,
        0.22,
        quaternion=Quaternion(x=0.5, y=0.4, z=0.3, w=0.2),
    )
    pre_grasp_pose = _pose(
        0.48,
        0.18,
        0.30,
        quaternion=Quaternion(x=0.5, y=0.4, z=0.3, w=0.2),
    )

    node._selected_object_id = 1
    node._prepare_grasp_poses = lambda object_id: (pre_grasp_pose, grasp_pose)
    node._lookup_current_pose = lambda: _pose(0.30, 0.10, 0.45)

    node._start_grasp()

    assert node._state == grasp_task_planner.STATE_MOVING_TO_PRE_GRASP
    assert node._active_motion == grasp_task_planner.MOTION_PRE_GRASP
    assert len(node._cartesian_path_client.requests) == 1
    waypoint = node._cartesian_path_client.requests[0].waypoints[0]
    assert waypoint.position.x == pytest.approx(0.48)
    assert waypoint.position.y == pytest.approx(0.18)
    assert waypoint.position.z == pytest.approx(0.30)
    orientation_constraints = (
        node._cartesian_path_client.requests[0]
        .path_constraints.orientation_constraints
    )
    assert len(orientation_constraints) == 1
    assert orientation_constraints[0].orientation.x == pytest.approx(0.5)
    assert node._pending_grasp_pose is grasp_pose
    assert any('Resolved pre-grasp target:' in message for message in logger.infos)
    assert any('Resolved grasp target:' in message for message in logger.infos)


def test_start_grasp_can_wait_before_sending_pre_grasp_goal():
    node, logger = _make_node()
    timer_holder = {}
    grasp_pose = _pose(0.48, 0.18, 0.22)
    pre_grasp_pose = _pose(0.48, 0.18, 0.30)

    node._selected_object_id = 1
    node._pre_grasp_settle_time_s = 1.5
    node._prepare_grasp_poses = lambda object_id: (pre_grasp_pose, grasp_pose)
    node._lookup_current_pose = lambda: _pose(0.20, 0.10, 0.50)
    node.create_timer = lambda period, callback: timer_holder.setdefault(
        'timer', _FakeTimer(callback)
    )

    node._start_grasp()

    assert node._state == grasp_task_planner.STATE_MOVING_TO_PRE_GRASP
    assert node._pending_grasp_pose is grasp_pose
    assert node._pending_pre_grasp_pose is pre_grasp_pose
    assert node._pre_grasp_settle_timer is timer_holder['timer']
    assert len(node._cartesian_path_client.requests) == 0

    timer_holder['timer'].callback()

    assert timer_holder['timer'].cancelled is True
    assert node._pending_pre_grasp_pose is None
    assert len(node._cartesian_path_client.requests) == 1


def test_execute_pre_grasp_success_triggers_cartesian_grasp_approach():
    node, logger = _make_node()
    grasp_pose = _pose(0.48, 0.18, 0.22)
    cartesian_calls = []

    node._active_motion = grasp_task_planner.MOTION_PRE_GRASP
    node._pending_grasp_pose = grasp_pose
    node._start_cartesian_motion = lambda target_pose, motion_kind, unavailable_message: (
        cartesian_calls.append((target_pose, motion_kind, unavailable_message))
    )

    node._execute_result_cb(_execute_future())

    assert node._state == grasp_task_planner.STATE_MOVING_TO_GRASP
    assert len(cartesian_calls) == 1
    assert cartesian_calls[0][0] is grasp_pose
    assert cartesian_calls[0][1] == grasp_task_planner.MOTION_GRASP_APPROACH
    assert any('starting Cartesian descend to the grasp pose' in message for message in logger.infos)


def test_execute_grasp_success_transitions_to_waiting_at_grasp():
    node, logger = _make_node()
    grasp_pose = _pose(0.48, 0.18, 0.22)

    node._active_motion = grasp_task_planner.MOTION_GRASP_APPROACH
    node._active_target_pose = grasp_pose
    node._pending_grasp_pose = grasp_pose
    node._lookup_current_pose = lambda: _pose(0.48, 0.18, 0.22)

    node._execute_result_cb(_execute_future())

    assert node._state == grasp_task_planner.STATE_WAITING_AT_GRASP
    assert node._active_motion == grasp_task_planner.MOTION_NONE
    assert node._pending_grasp_pose is None
    assert any('Reached the final grasp pose' in message for message in logger.infos)


def test_require_target_reached_failure_returns_to_idle():
    node, logger = _make_node()
    target_pose = _pose(0.48, 0.18, 0.22)
    node._lookup_current_pose = lambda: _pose(0.55, 0.18, 0.22)

    reached = node._require_target_reached(target_pose, 'home_pose')

    assert reached is False
    assert node._state == grasp_task_planner.STATE_IDLE
    assert any('home_pose is still' in message for message in logger.errors)
    assert any('returning the planner to IDLE' in message for message in logger.warnings)


def test_stop_grasp_requires_waiting_at_grasp():
    node, logger = _make_node()

    node._stop_grasp()

    assert any('Ignoring stop_grasp' in message for message in logger.warnings)


def test_stop_grasp_plans_cartesian_lift_first():
    node, logger = _make_node()
    current_pose = _pose(0.36, 0.20, 0.12)

    node._state = grasp_task_planner.STATE_WAITING_AT_GRASP
    node._lookup_current_pose = lambda: current_pose

    node._stop_grasp()

    assert node._state == grasp_task_planner.STATE_MOVING_TO_DROP
    assert node._active_motion == grasp_task_planner.MOTION_LIFT
    assert len(node._cartesian_path_client.requests) == 1
    assert node._cartesian_path_client.requests[0].waypoints[0].position.z == pytest.approx(0.20)


def test_execute_lift_success_triggers_drop_motion():
    node, logger = _make_node()
    drop_calls = []

    node._active_motion = grasp_task_planner.MOTION_LIFT
    node._start_cartesian_motion = (
        lambda target_pose, motion_kind, unavailable_message: drop_calls.append(
            (target_pose, motion_kind, unavailable_message)
        )
    )

    node._execute_result_cb(_execute_future())

    assert len(drop_calls) == 1
    assert drop_calls[0][0] is node._drop_pose
    assert drop_calls[0][1] == grasp_task_planner.MOTION_DROP
    assert any('Lift complete' in message for message in logger.infos)


def test_execute_drop_success_opens_gripper_and_moves_to_opening_state():
    node, logger = _make_node()
    auto_open_calls = []

    node._state = grasp_task_planner.STATE_MOVING_TO_DROP
    node._active_motion = grasp_task_planner.MOTION_DROP
    node._active_target_pose = node._drop_pose
    node._lookup_current_pose = lambda: _pose(-0.5, 0.0, 0.15)
    node._start_auto_open = lambda location_label: auto_open_calls.append(location_label)

    node._execute_result_cb(_execute_future())

    assert node._state == grasp_task_planner.STATE_OPENING_AT_DROP
    assert auto_open_calls == ['drop_pose']
    assert any('Reached the drop_pose' in message for message in logger.infos)


def test_gripper_open_at_drop_success_triggers_home_motion():
    node, logger = _make_node()
    home_calls = []

    node._state = grasp_task_planner.STATE_OPENING_AT_DROP
    node._start_home_motion = lambda unavailable_message: home_calls.append(unavailable_message)

    node._gripper_result_cb(_gripper_future())

    assert len(home_calls) == 1
    assert any('continuing to home_pose' in message for message in logger.infos)


def test_go_home_starts_cartesian_home_via_motion():
    node, logger = _make_node()
    node._lookup_current_pose = lambda: _pose(0.10, 0.20, 0.30)

    node._go_home()

    assert node._state == grasp_task_planner.STATE_MOVING_TO_HOME
    assert node._active_motion == grasp_task_planner.MOTION_HOME_VIA
    assert len(node._cartesian_path_client.requests) == 1
    assert node._cartesian_path_client.requests[0].waypoints[0].position.x == pytest.approx(-0.1)
    assert node._cartesian_path_client.requests[0].waypoints[0].position.z == pytest.approx(0.31)
    assert any('Received go_home command' in message for message in logger.infos)
    assert any('object 2 pre-grasp waypoint' in message for message in logger.infos)


def test_go_home_rejects_busy_state():
    node, logger = _make_node()
    node._state = grasp_task_planner.STATE_MOVING_TO_PRE_GRASP

    node._go_home()

    assert any('Ignoring go_home while planner state is MOVING_TO_PRE_GRASP' in message for message in logger.warnings)


def test_execute_home_success_opens_gripper_at_home():
    node, logger = _make_node()
    auto_open_calls = []

    node._state = grasp_task_planner.STATE_MOVING_TO_HOME
    node._active_motion = grasp_task_planner.MOTION_HOME
    node._active_target_pose = node._home_pose
    node._lookup_current_pose = lambda: _pose(0.04, 0.3, 0.3)
    node._start_auto_open = lambda location_label: auto_open_calls.append(location_label)

    node._execute_result_cb(_execute_future())

    assert node._state == grasp_task_planner.STATE_OPENING_AT_HOME
    assert auto_open_calls == ['home_pose']
    assert any('Reached the home_pose' in message for message in logger.infos)


def test_execute_home_via_success_triggers_final_home_leg():
    node, logger = _make_node()
    final_home_calls = []
    home_via_pose = _pose(-0.1, 0.4, 0.31)

    node._state = grasp_task_planner.STATE_MOVING_TO_HOME
    node._active_motion = grasp_task_planner.MOTION_HOME_VIA
    node._active_target_pose = home_via_pose
    node._lookup_current_pose = lambda: _pose(-0.1, 0.4, 0.31)
    node._start_final_home_leg = lambda: final_home_calls.append('home')

    node._execute_result_cb(_execute_future())

    assert node._state == grasp_task_planner.STATE_MOVING_TO_HOME
    assert node._active_motion == grasp_task_planner.MOTION_NONE
    assert final_home_calls == ['home']
    assert any('continuing to home_pose' in message for message in logger.infos)


def test_gripper_open_at_home_success_returns_to_idle():
    node, logger = _make_node()
    node._state = grasp_task_planner.STATE_OPENING_AT_HOME

    node._gripper_result_cb(_gripper_future())

    assert node._state == grasp_task_planner.STATE_IDLE
    assert node._active_motion == grasp_task_planner.MOTION_NONE
    assert any('planner is idle' in message for message in logger.infos)
