from pathlib import Path
import sys

import pytest

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
from std_msgs.msg import Header
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectoryPoint

from robotiq_2f85_bringup import grasp_task_planner


def _pose(
    x: float,
    y: float,
    z: float,
    frame_id: str = 'map',
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
    relative_pose = _pose(0.18, 0.08, 0.02, frame_id='base_link')

    composed = grasp_task_planner._compose_pose(reference_pose, relative_pose)

    assert composed.header.frame_id == 'map'
    assert composed.pose.position.x == pytest.approx(0.48)
    assert composed.pose.position.y == pytest.approx(0.18)
    assert composed.pose.position.z == pytest.approx(0.22)


def test_resolve_pose_in_base_frame_transforms_non_map_pose():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    node._base_frame = 'map'
    node.get_logger = lambda: _FakeLogger()
    node._tf_buf = type(
        'Buffer',
        (),
        {
            'lookup_transform': staticmethod(
                lambda target, source, time, timeout: type(
                    'TF',
                    (),
                    {
                        'header': Header(frame_id='map'),
                        'transform': type(
                            'Transform',
                            (),
                            {
                                'translation': type('Translation', (), {'x': 0.1, 'y': 0.2, 'z': 0.3})(),
                                'rotation': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                            },
                        )(),
                    },
                )()
            )
        },
    )()
    pose = _pose(0.2, 0.3, 0.4, frame_id='base_link')

    resolved = node._resolve_pose_in_base_frame(pose)

    assert resolved is not None
    assert resolved.header.frame_id == 'map'
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
        base_frame='map',
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


def test_build_move_group_goal_uses_goal_constraints():
    target_pose = _pose(0.48, 0.18, 0.30)

    goal = grasp_task_planner._build_move_group_goal(
        target_pose=target_pose,
        planning_group='nova5_group',
        base_frame='map',
        tip_link='tool_link',
        planning_time_s=5.0,
        num_planning_attempts=10,
        position_tolerance_m=0.01,
        orientation_tolerance_rad=0.01,
    )

    assert goal.request.group_name == 'nova5_group'
    assert goal.request.goal_constraints[0].position_constraints[0].link_name == 'tool_link'
    assert len(goal.request.goal_constraints[0].orientation_constraints) == 1
    assert goal.request.path_constraints.orientation_constraints == []


def test_build_move_group_goal_can_skip_orientation_constraint():
    target_pose = _pose(0.48, 0.18, 0.30)

    goal = grasp_task_planner._build_move_group_goal(
        target_pose=target_pose,
        planning_group='nova5_group',
        base_frame='map',
        tip_link='tool_link',
        planning_time_s=5.0,
        num_planning_attempts=10,
        position_tolerance_m=0.01,
        orientation_tolerance_rad=None,
    )

    assert goal.request.group_name == 'nova5_group'
    assert goal.request.goal_constraints[0].position_constraints[0].link_name == 'tool_link'
    assert goal.request.goal_constraints[0].orientation_constraints == []


def test_derive_pre_grasp_pose_offsets_final_grasp_in_z():
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
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    node._grasp_poses = {idx: _pose(0.3, 0.1, 0.8) for idx in range(1, 6)}
    logger = _FakeLogger()
    node.get_logger = lambda: logger

    node._object_id_cb(Int32(data=3))

    assert node._selected_object_id == 3
    assert any('Selected grasp object id 3' in message for message in logger.infos)


def test_start_grasp_requires_selected_object():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()

    node._state = grasp_task_planner.STATE_IDLE
    node._selected_object_id = None
    node._pre_grasp_settle_time_s = 0.0
    node.get_logger = lambda: logger

    node._start_grasp()

    assert any('/grasp_object_id' in message for message in logger.warnings)


def test_start_grasp_sends_selected_pre_grasp_goal():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()
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
    service_client = _FakeServiceClient()
    execute_client = _FakeActionClient()

    node._state = grasp_task_planner.STATE_IDLE
    node._selected_object_id = 1
    node._planning_group = 'nova5_group'
    node._base_frame = 'map'
    node._tip_link = 'tool_link'
    node._orientation_tolerance_rad = 0.01
    node._pre_grasp_offset_m = 0.08
    node._pre_grasp_settle_time_s = 0.0
    node._cartesian_max_step_m = 0.005
    node._cartesian_jump_threshold = 0.0
    node._cartesian_prismatic_jump_threshold = 0.0
    node._cartesian_revolute_jump_threshold = 0.0
    node._cartesian_avoid_collisions = True
    node._cartesian_path_service = '/compute_cartesian_path'
    node._execute_trajectory_action = '/execute_trajectory'
    node._cartesian_path_client = service_client
    node._execute_trajectory_client = execute_client
    node._pending_grasp_pose = None
    node._pending_pre_grasp_pose = None
    node._pre_grasp_settle_timer = None
    node._prepare_grasp_poses = lambda object_id: (pre_grasp_pose, grasp_pose)
    node._lookup_current_pose = lambda: _pose(
        0.30,
        0.10,
        0.45,
        quaternion=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9),
    )
    node._resolve_pose_in_base_frame = lambda pose: pose
    node.get_logger = lambda: logger

    node._start_grasp()

    assert node._state == grasp_task_planner.STATE_MOVING_TO_PRE_GRASP
    assert node._active_motion == grasp_task_planner.MOTION_PRE_GRASP
    assert len(service_client.requests) == 1
    waypoint = service_client.requests[0].waypoints[0]
    assert waypoint.position.x == pytest.approx(0.48)
    assert waypoint.position.y == pytest.approx(0.18)
    assert waypoint.position.z == pytest.approx(0.30)
    orientation_constraints = service_client.requests[0].path_constraints.orientation_constraints
    assert len(orientation_constraints) == 1
    assert orientation_constraints[0].orientation.x == pytest.approx(0.5)
    assert orientation_constraints[0].orientation.y == pytest.approx(0.4)
    assert orientation_constraints[0].orientation.z == pytest.approx(0.3)
    assert orientation_constraints[0].orientation.w == pytest.approx(0.2)
    assert node._pending_grasp_pose is grasp_pose
    assert any('Resolved pre-grasp target:' in message for message in logger.infos)
    assert any('Resolved grasp target:' in message for message in logger.infos)


def test_start_grasp_can_wait_before_sending_pre_grasp_goal():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()
    grasp_pose = _pose(0.48, 0.18, 0.22)
    pre_grasp_pose = _pose(0.48, 0.18, 0.30)
    service_client = _FakeServiceClient()
    execute_client = _FakeActionClient()
    timer_holder = {}

    node._state = grasp_task_planner.STATE_IDLE
    node._selected_object_id = 1
    node._planning_group = 'nova5_group'
    node._base_frame = 'map'
    node._tip_link = 'tool_link'
    node._orientation_tolerance_rad = 0.01
    node._pre_grasp_settle_time_s = 1.5
    node._cartesian_max_step_m = 0.005
    node._cartesian_jump_threshold = 0.0
    node._cartesian_prismatic_jump_threshold = 0.0
    node._cartesian_revolute_jump_threshold = 0.0
    node._cartesian_avoid_collisions = True
    node._cartesian_path_service = '/compute_cartesian_path'
    node._execute_trajectory_action = '/execute_trajectory'
    node._cartesian_path_client = service_client
    node._execute_trajectory_client = execute_client
    node._pending_grasp_pose = None
    node._pending_pre_grasp_pose = None
    node._pre_grasp_settle_timer = None
    node._prepare_grasp_poses = lambda object_id: (pre_grasp_pose, grasp_pose)
    node._lookup_current_pose = lambda: _pose(0.20, 0.10, 0.50)
    node._resolve_pose_in_base_frame = lambda pose: pose
    node.create_timer = lambda period, callback: timer_holder.setdefault(
        'timer', _FakeTimer(callback)
    )
    node.get_logger = lambda: logger

    node._start_grasp()

    assert node._state == grasp_task_planner.STATE_MOVING_TO_PRE_GRASP
    assert len(service_client.requests) == 0
    assert node._pending_grasp_pose is grasp_pose
    assert node._pending_pre_grasp_pose is pre_grasp_pose
    assert node._pre_grasp_settle_timer is timer_holder['timer']
    assert any('Waiting 1.50 s for the arm to settle' in message for message in logger.infos)

    timer_holder['timer'].callback()

    assert timer_holder['timer'].cancelled is True
    assert node._pending_pre_grasp_pose is None
    assert len(service_client.requests) == 1


def test_execute_pre_grasp_success_triggers_cartesian_grasp_approach():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()
    grasp_pose = _pose(0.48, 0.18, 0.22)
    cartesian_calls = []

    class _ExecuteFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {'error_code': MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)},
            )()
            return wrapper

    node._active_motion = grasp_task_planner.MOTION_PRE_GRASP
    node._pending_grasp_pose = grasp_pose
    node.get_logger = lambda: logger
    node._set_state = lambda state: setattr(node, '_state', state)
    node._start_cartesian_motion = (
        lambda target_pose, motion_kind, unavailable_message: cartesian_calls.append(
            (target_pose, motion_kind, unavailable_message)
        )
    )

    node._execute_result_cb(_ExecuteFuture())

    assert node._state == grasp_task_planner.STATE_MOVING_TO_GRASP
    assert len(cartesian_calls) == 1
    assert cartesian_calls[0][0] is grasp_pose
    assert cartesian_calls[0][1] == grasp_task_planner.MOTION_GRASP_APPROACH
    assert any('starting Cartesian descend to the grasp pose' in message for message in logger.infos)


def test_timeout_far_from_target_still_fails():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()
    target_pose = _pose(0.30, 0.45, 0.30)
    current_pose = _pose(0.40, 0.55, 0.40)

    class _ExecuteFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {'error_code': MoveItErrorCodes(val=-6)},
            )()
            return wrapper

    node._active_motion = grasp_task_planner.MOTION_LIFT
    node._active_target_pose = target_pose
    node._execute_timeout_position_tolerance_m = 0.02
    node._lookup_current_pose = lambda: current_pose
    node.get_logger = lambda: logger
    node._clear_motion = lambda state=grasp_task_planner.STATE_IDLE: setattr(node, '_state', state)

    node._execute_result_cb(_ExecuteFuture())

    assert node._state == grasp_task_planner.STATE_IDLE
    assert any('failed with error code -6' in message for message in logger.errors)


def test_execute_pre_grasp_failure_clears_motion():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()

    class _ExecuteFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {'error_code': MoveItErrorCodes(val=-1)},
            )()
            return wrapper

    node._active_motion = grasp_task_planner.MOTION_PRE_GRASP
    node.get_logger = lambda: logger
    node._clear_motion = lambda state=grasp_task_planner.STATE_IDLE: setattr(node, '_state', state)

    node._execute_result_cb(_ExecuteFuture())

    assert node._state == grasp_task_planner.STATE_IDLE
    assert any("ExecuteTrajectory motion 'pre_grasp' failed" in message for message in logger.errors)


def test_stop_grasp_requires_waiting_state():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()
    node._state = grasp_task_planner.STATE_IDLE
    node.get_logger = lambda: logger

    node._stop_grasp()

    assert any('Ignoring stop_grasp' in message for message in logger.warnings)


def test_stop_grasp_plans_cartesian_lift_first():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()
    service_client = _FakeServiceClient()
    execute_client = _FakeActionClient()
    current_pose = _pose(0.36, 0.20, 0.12)

    node._state = grasp_task_planner.STATE_WAITING_FOR_GRIPPER
    node._lift_height_m = 0.08
    node._planning_group = 'nova5_group'
    node._base_frame = 'map'
    node._tip_link = 'tool_link'
    node._orientation_tolerance_rad = 0.01
    node._cartesian_max_step_m = 0.005
    node._cartesian_jump_threshold = 0.0
    node._cartesian_prismatic_jump_threshold = 0.0
    node._cartesian_revolute_jump_threshold = 0.0
    node._cartesian_avoid_collisions = True
    node._cartesian_path_service = '/compute_cartesian_path'
    node._execute_trajectory_action = '/execute_trajectory'
    node._cartesian_path_client = service_client
    node._execute_trajectory_client = execute_client
    node._lookup_current_pose = lambda: current_pose
    node.get_logger = lambda: logger

    node._stop_grasp()

    assert node._state == grasp_task_planner.STATE_MOVING_TO_DROP
    assert node._active_motion == grasp_task_planner.MOTION_LIFT
    assert len(service_client.requests) == 1
    assert service_client.requests[0].waypoints[0].position.z == pytest.approx(0.20)


def test_cartesian_lift_success_triggers_drop_goal():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()
    execute_client = _FakeActionClient()
    drop_calls = []
    solution = RobotTrajectory()
    solution.joint_trajectory.points = [JointTrajectoryPoint()]

    class _Future:
        def result(self):
            return GetCartesianPath.Response(
                solution=solution,
                fraction=1.0,
                error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS),
            )

    node._active_motion = grasp_task_planner.MOTION_LIFT
    node._cartesian_min_fraction = 0.99
    node._execute_trajectory_client = execute_client
    node.get_logger = lambda: logger

    node._cartesian_path_response_cb(_Future())

    assert len(execute_client.sent_goals) == 1

    class _ExecuteFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {'error_code': MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)},
            )()
            return wrapper

    node._drop_pose = _pose(-0.5, 0.0, 0.15, frame_id='base_link')
    node._start_cartesian_motion = (
        lambda target_pose, motion_kind, unavailable_message, preserve_current_orientation=False: drop_calls.append(
            (target_pose, motion_kind, unavailable_message, preserve_current_orientation)
        )
    )

    node._execute_result_cb(_ExecuteFuture())

    assert len(drop_calls) == 1
    assert drop_calls[0][1] == grasp_task_planner.MOTION_DROP
    assert drop_calls[0][0].header.frame_id == 'base_link'
    assert drop_calls[0][3] is True


def test_execute_grasp_approach_success_transitions_to_waiting_for_gripper():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()

    class _ExecuteFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {'error_code': MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)},
            )()
            return wrapper

    node._active_motion = grasp_task_planner.MOTION_GRASP_APPROACH
    node._pending_grasp_pose = _pose(0.48, 0.18, 0.22)
    node.get_logger = lambda: logger
    node._clear_motion = lambda state=grasp_task_planner.STATE_IDLE: (
        setattr(node, '_active_motion', grasp_task_planner.MOTION_NONE),
        setattr(node, '_state', state),
    )

    node._execute_result_cb(_ExecuteFuture())

    assert node._state == grasp_task_planner.STATE_WAITING_FOR_GRIPPER
    assert node._pending_grasp_pose is None
    assert any('Reached the final grasp pose' in message for message in logger.infos)


def test_execute_grasp_approach_failure_returns_to_idle():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()

    class _ExecuteFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {'error_code': MoveItErrorCodes(val=-31)},
            )()
            return wrapper

    node._active_motion = grasp_task_planner.MOTION_GRASP_APPROACH
    node.get_logger = lambda: logger
    node._clear_motion = lambda state=grasp_task_planner.STATE_IDLE: setattr(node, '_state', state)

    node._execute_result_cb(_ExecuteFuture())

    assert node._state == grasp_task_planner.STATE_IDLE
    assert any("ExecuteTrajectory motion 'grasp_approach' failed" in message for message in logger.errors)


def test_execute_drop_success_returns_to_idle():
    node = grasp_task_planner.GraspTaskPlanner.__new__(grasp_task_planner.GraspTaskPlanner)
    logger = _FakeLogger()

    class _ExecuteFuture:
        def result(self):
            wrapper = type('ResultWrapper', (), {})()
            wrapper.result = type(
                'Result',
                (),
                {'error_code': MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)},
            )()
            return wrapper

    node._active_motion = grasp_task_planner.MOTION_DROP
    node.get_logger = lambda: logger
    node._clear_motion = lambda state=grasp_task_planner.STATE_IDLE: (
        setattr(node, '_active_motion', grasp_task_planner.MOTION_NONE),
        setattr(node, '_state', state),
    )

    node._execute_result_cb(_ExecuteFuture())

    assert node._state == grasp_task_planner.STATE_IDLE
    assert any('Planner is idle again' in message for message in logger.infos)
