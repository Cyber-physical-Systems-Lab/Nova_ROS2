#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
import math

import rclpy
import rclpy.duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32, String
import tf2_ros

from robotiq_2f85_bringup.web_command import (
    is_go_home,
    is_start_grasp,
    is_stop_grasp,
    parse_web_command,
)


STATE_IDLE = 'IDLE'
STATE_MOVING_TO_PRE_GRASP = 'MOVING_TO_PRE_GRASP'
STATE_MOVING_TO_GRASP = 'MOVING_TO_GRASP'
STATE_WAITING_AT_GRASP = 'WAITING_AT_GRASP'
STATE_MOVING_TO_DROP = 'MOVING_TO_DROP'
STATE_OPENING_AT_DROP = 'OPENING_AT_DROP'
STATE_MOVING_TO_HOME = 'MOVING_TO_HOME'
STATE_OPENING_AT_HOME = 'OPENING_AT_HOME'
STATE_ERROR = 'ERROR'

MOTION_NONE = 'none'
MOTION_PRE_GRASP = 'pre_grasp'
MOTION_GRASP_APPROACH = 'grasp_approach'
MOTION_LIFT = 'lift'
MOTION_DROP = 'drop'
MOTION_HOME_VIA = 'home_via'
MOTION_HOME = 'home'

_GRIPPER_OPEN_JOINT = 0.0
_GRIPPER_CLOSED_JOINT = 0.7929


@dataclass(frozen=True)
class PoseComponents:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def _copy_quaternion(quaternion: Quaternion) -> Quaternion:
    return Quaternion(
        x=quaternion.x,
        y=quaternion.y,
        z=quaternion.z,
        w=quaternion.w,
    )


def _normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float]:
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def _quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    x = (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y)
    y = (lhs.w * rhs.y) - (lhs.x * rhs.z) + (lhs.y * rhs.w) + (lhs.z * rhs.x)
    z = (lhs.w * rhs.z) + (lhs.x * rhs.y) - (lhs.y * rhs.x) + (lhs.z * rhs.w)
    w = (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z)
    nx, ny, nz, nw = _normalize_quaternion(x, y, z, w)
    return Quaternion(x=nx, y=ny, z=nz, w=nw)


def _rotate_point(point, quaternion: Quaternion):
    qx, qy, qz, qw = _normalize_quaternion(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    px, py, pz = float(point.x), float(point.y), float(point.z)

    tx = 2.0 * ((qy * pz) - (qz * py))
    ty = 2.0 * ((qz * px) - (qx * pz))
    tz = 2.0 * ((qx * py) - (qy * px))

    rx = px + (qw * tx) + ((qy * tz) - (qz * ty))
    ry = py + (qw * ty) + ((qz * tx) - (qx * tz))
    rz = pz + (qw * tz) + ((qx * ty) - (qy * tx))
    return rx, ry, rz


def _position_distance_m(lhs: PoseStamped, rhs: PoseStamped) -> float:
    dx = float(lhs.pose.position.x) - float(rhs.pose.position.x)
    dy = float(lhs.pose.position.y) - float(rhs.pose.position.y)
    dz = float(lhs.pose.position.z) - float(rhs.pose.position.z)
    return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))


def _pose_from_components(
    pose_components: PoseComponents,
    frame_id: str,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(pose_components.x)
    pose.pose.position.y = float(pose_components.y)
    pose.pose.position.z = float(pose_components.z)
    pose.pose.orientation = Quaternion(
        x=float(pose_components.qx),
        y=float(pose_components.qy),
        z=float(pose_components.qz),
        w=float(pose_components.qw),
    )
    return pose


def _compose_pose(reference_pose: PoseStamped, relative_pose: PoseStamped) -> PoseStamped:
    composed = PoseStamped()
    composed.header.frame_id = reference_pose.header.frame_id
    composed.header.stamp = reference_pose.header.stamp

    rx, ry, rz = _rotate_point(
        relative_pose.pose.position,
        reference_pose.pose.orientation,
    )
    composed.pose.position.x = reference_pose.pose.position.x + rx
    composed.pose.position.y = reference_pose.pose.position.y + ry
    composed.pose.position.z = reference_pose.pose.position.z + rz
    composed.pose.orientation = _quaternion_multiply(
        reference_pose.pose.orientation,
        relative_pose.pose.orientation,
    )
    return composed


def _transform_to_pose_stamped(tf_msg) -> PoseStamped:
    pose = PoseStamped()
    pose.header = tf_msg.header
    pose.pose.position.x = tf_msg.transform.translation.x
    pose.pose.position.y = tf_msg.transform.translation.y
    pose.pose.position.z = tf_msg.transform.translation.z
    pose.pose.orientation = _copy_quaternion(tf_msg.transform.rotation)
    return pose


def _build_target_pose(current_pose: PoseStamped, delta_z: float) -> PoseStamped:
    target = PoseStamped()
    target.header = current_pose.header
    target.pose.position.x = current_pose.pose.position.x
    target.pose.position.y = current_pose.pose.position.y
    target.pose.position.z = current_pose.pose.position.z + float(delta_z)
    target.pose.orientation = _copy_quaternion(current_pose.pose.orientation)
    return target


def _derive_pre_grasp_pose(
    grasp_pose: PoseStamped,
    pre_grasp_offset_m: float,
) -> PoseStamped:
    return _build_target_pose(grasp_pose, pre_grasp_offset_m)


def _format_pose_for_log(pose: PoseStamped) -> str:
    return (
        f"frame={pose.header.frame_id} "
        f"x={pose.pose.position.x:.3f} "
        f"y={pose.pose.position.y:.3f} "
        f"z={pose.pose.position.z:.3f} "
        f"qx={pose.pose.orientation.x:.3f} "
        f"qy={pose.pose.orientation.y:.3f} "
        f"qz={pose.pose.orientation.z:.3f} "
        f"qw={pose.pose.orientation.w:.3f}"
    )


def _build_orientation_constraint(
    target_pose: PoseStamped,
    base_frame: str,
    tip_link: str,
    orientation_tolerance_rad: float,
) -> OrientationConstraint:
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = base_frame
    orientation_constraint.link_name = tip_link
    orientation_constraint.orientation = _copy_quaternion(
        target_pose.pose.orientation
    )
    orientation_constraint.absolute_x_axis_tolerance = float(
        orientation_tolerance_rad
    )
    orientation_constraint.absolute_y_axis_tolerance = float(
        orientation_tolerance_rad
    )
    orientation_constraint.absolute_z_axis_tolerance = float(
        orientation_tolerance_rad
    )
    orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
    orientation_constraint.weight = 1.0
    return orientation_constraint


def _build_cartesian_path_request(
    current_pose: PoseStamped,
    target_pose: PoseStamped,
    planning_group: str,
    base_frame: str,
    tip_link: str,
    orientation_tolerance_rad: float,
    max_step_m: float,
    jump_threshold: float,
    prismatic_jump_threshold: float,
    revolute_jump_threshold: float,
    avoid_collisions: bool,
) -> GetCartesianPath.Request:
    orientation_constraint = _build_orientation_constraint(
        target_pose=target_pose,
        base_frame=base_frame,
        tip_link=tip_link,
        orientation_tolerance_rad=orientation_tolerance_rad,
    )

    path_constraints = Constraints()
    path_constraints.orientation_constraints = [orientation_constraint]

    request = GetCartesianPath.Request()
    request.header.frame_id = base_frame
    request.header.stamp = current_pose.header.stamp
    request.start_state = RobotState(is_diff=True)
    request.group_name = planning_group
    request.link_name = tip_link
    request.waypoints = [target_pose.pose]
    request.max_step = float(max_step_m)
    request.jump_threshold = float(jump_threshold)
    request.prismatic_jump_threshold = float(prismatic_jump_threshold)
    request.revolute_jump_threshold = float(revolute_jump_threshold)
    request.avoid_collisions = bool(avoid_collisions)
    request.path_constraints = path_constraints
    return request


def _percent_to_joint(percent: float) -> float:
    clamped = max(0.0, min(100.0, float(percent)))
    normalized = 1.0 - (clamped / 100.0)
    return _GRIPPER_OPEN_JOINT + normalized * (
        _GRIPPER_CLOSED_JOINT - _GRIPPER_OPEN_JOINT
    )


class GraspTaskPlanner(Node):
    def __init__(self):
        super().__init__('grasp_task_planner')

        self.declare_parameter('command_topic', '/web_command')
        self.declare_parameter('object_id_topic', '/grasp_object_id')
        self.declare_parameter('execute_trajectory_action', '/execute_trajectory')
        self.declare_parameter('cartesian_path_service', '/compute_cartesian_path')
        self.declare_parameter(
            'gripper_action',
            '/gripper_position_controller/gripper_cmd',
        )
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tip_link', 'tool_link')
        self.declare_parameter('planning_group', 'nova5_group')
        self.declare_parameter('orientation_tolerance_rad', 0.01)
        self.declare_parameter('execute_timeout_position_tolerance_m', 0.02)
        self.declare_parameter('target_reached_tolerance_m', 0.01)
        self.declare_parameter('cartesian_max_step_m', 0.005)
        self.declare_parameter('cartesian_jump_threshold', 0.0)
        self.declare_parameter('cartesian_prismatic_jump_threshold', 0.0)
        self.declare_parameter('cartesian_revolute_jump_threshold', 0.0)
        self.declare_parameter('cartesian_min_fraction', 0.99)
        self.declare_parameter('cartesian_avoid_collisions', True)
        self.declare_parameter('lift_height_m', 0.08)
        self.declare_parameter('pre_grasp_offset_m', 0.08)
        self.declare_parameter('pre_grasp_settle_time_s', 0.0)
        self.declare_parameter('auto_open_percent', 100.0)
        self.declare_parameter('gripper_max_effort', 50.0)

        self._command_topic = str(self.get_parameter('command_topic').value)
        self._object_id_topic = str(self.get_parameter('object_id_topic').value)
        self._execute_trajectory_action = str(
            self.get_parameter('execute_trajectory_action').value
        )
        self._cartesian_path_service = str(
            self.get_parameter('cartesian_path_service').value
        )
        self._gripper_action = str(self.get_parameter('gripper_action').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tip_link = str(self.get_parameter('tip_link').value)
        self._planning_group = str(self.get_parameter('planning_group').value)
        self._orientation_tolerance_rad = float(
            self.get_parameter('orientation_tolerance_rad').value
        )
        self._execute_timeout_position_tolerance_m = float(
            self.get_parameter('execute_timeout_position_tolerance_m').value
        )
        self._target_reached_tolerance_m = float(
            self.get_parameter('target_reached_tolerance_m').value
        )
        self._cartesian_max_step_m = float(
            self.get_parameter('cartesian_max_step_m').value
        )
        self._cartesian_jump_threshold = float(
            self.get_parameter('cartesian_jump_threshold').value
        )
        self._cartesian_prismatic_jump_threshold = float(
            self.get_parameter('cartesian_prismatic_jump_threshold').value
        )
        self._cartesian_revolute_jump_threshold = float(
            self.get_parameter('cartesian_revolute_jump_threshold').value
        )
        self._cartesian_min_fraction = float(
            self.get_parameter('cartesian_min_fraction').value
        )
        self._cartesian_avoid_collisions = bool(
            self.get_parameter('cartesian_avoid_collisions').value
        )
        self._lift_height_m = float(self.get_parameter('lift_height_m').value)
        self._pre_grasp_offset_m = float(
            self.get_parameter('pre_grasp_offset_m').value
        )
        self._pre_grasp_settle_time_s = float(
            self.get_parameter('pre_grasp_settle_time_s').value
        )
        self._auto_open_percent = float(
            self.get_parameter('auto_open_percent').value
        )
        self._gripper_max_effort = float(
            self.get_parameter('gripper_max_effort').value
        )

        self._drop_pose = self._load_pose('drop_pose', 'base_link')
        self._home_pose = self._load_pose('home_pose', 'base_link')
        self._grasp_poses = {
            object_id: self._load_pose(f'grasp_pose_{object_id}', self._base_frame)
            for object_id in range(1, 6)
        }

        self._state = STATE_IDLE
        self._active_motion = MOTION_NONE
        self._selected_object_id = None
        self._active_target_pose = None
        self._pending_grasp_pose = None
        self._pending_pre_grasp_pose = None
        self._pre_grasp_settle_timer = None

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        self._cartesian_path_client = self.create_client(
            GetCartesianPath,
            self._cartesian_path_service,
        )
        self._execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            self._execute_trajectory_action,
        )
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            self._gripper_action,
        )

        self.create_subscription(String, self._command_topic, self._web_command_cb, 10)
        self.create_subscription(Int32, self._object_id_topic, self._object_id_cb, 10)

        self.get_logger().info(
            f'Grasp task planner ready on {self._command_topic} with object topic '
            f'{self._object_id_topic}. Planning frame: {self._base_frame}.'
        )

    def _declare_pose_parameter(
        self,
        prefix: str,
        defaults: PoseComponents,
    ) -> None:
        self.declare_parameter(f'{prefix}.position.x', defaults.x)
        self.declare_parameter(f'{prefix}.position.y', defaults.y)
        self.declare_parameter(f'{prefix}.position.z', defaults.z)
        self.declare_parameter(f'{prefix}.orientation.x', defaults.qx)
        self.declare_parameter(f'{prefix}.orientation.y', defaults.qy)
        self.declare_parameter(f'{prefix}.orientation.z', defaults.qz)
        self.declare_parameter(f'{prefix}.orientation.w', defaults.qw)

    def _load_pose(self, prefix: str, frame_id: str) -> PoseStamped:
        defaults = {
            'drop_pose': PoseComponents(-0.5, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0),
            'home_pose': PoseComponents(0.04, 0.3, 0.3, 1.0, 0.0, 0.0, 0.0),
            'grasp_pose_1': PoseComponents(-0.25, 0.41, 0.23, 1.0, 0.0, 0.0, 0.0),
            'grasp_pose_2': PoseComponents(-0.087, 0.41, 0.23, 1.0, 0.0, 0.0, 0.0),
            'grasp_pose_3': PoseComponents(0.064, 0.466, 0.23, 1.0, 0.0, 0.0, 0.0),
            'grasp_pose_4': PoseComponents(-0.046, 0.572, 0.23, 1.0, 0.0, 0.0, 0.0),
            'grasp_pose_5': PoseComponents(0.115, 0.616, 0.23, 1.0, 0.0, 0.0, 0.0),
        }
        default_pose = defaults[prefix]
        self._declare_pose_parameter(prefix, default_pose)
        return _pose_from_components(
            PoseComponents(
                x=float(self.get_parameter(f'{prefix}.position.x').value),
                y=float(self.get_parameter(f'{prefix}.position.y').value),
                z=float(self.get_parameter(f'{prefix}.position.z').value),
                qx=float(self.get_parameter(f'{prefix}.orientation.x').value),
                qy=float(self.get_parameter(f'{prefix}.orientation.y').value),
                qz=float(self.get_parameter(f'{prefix}.orientation.z').value),
                qw=float(self.get_parameter(f'{prefix}.orientation.w').value),
            ),
            frame_id,
        )

    def _set_state(self, state: str) -> None:
        self._state = state

    def _cancel_pre_grasp_timer(self) -> None:
        if self._pre_grasp_settle_timer is not None:
            self._pre_grasp_settle_timer.cancel()
            self._pre_grasp_settle_timer = None

    def _clear_active_motion(self) -> None:
        self._cancel_pre_grasp_timer()
        self._active_motion = MOTION_NONE
        self._active_target_pose = None

    def _reset_task(self, state: str = STATE_IDLE) -> None:
        self._clear_active_motion()
        self._pending_grasp_pose = None
        self._pending_pre_grasp_pose = None
        self._set_state(state)

    def _fail_task(self, message: str) -> None:
        self.get_logger().error(message)
        self.get_logger().warning(
            'Aborting the current task and returning the planner to IDLE so new commands can be accepted.'
        )
        self._reset_task(state=STATE_IDLE)

    def _object_id_cb(self, msg: Int32) -> None:
        object_id = int(msg.data)
        if object_id not in self._grasp_poses:
            self.get_logger().warning(
                f'Ignoring unsupported grasp object id {object_id}; expected 1..5.'
            )
            return

        self._selected_object_id = object_id
        self.get_logger().info(f'Selected grasp object id {object_id}.')

    def _lookup_current_pose(self) -> PoseStamped | None:
        try:
            tf_msg = self._tf_buf.lookup_transform(
                self._base_frame,
                self._tip_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warning(
                f'Failed to look up current {self._tip_link} pose in '
                f'{self._base_frame}: {exc}'
            )
            return None

        return _transform_to_pose_stamped(tf_msg)

    def _resolve_pose_in_base_frame(self, pose: PoseStamped) -> PoseStamped | None:
        if pose.header.frame_id == self._base_frame:
            return pose

        try:
            tf_msg = self._tf_buf.lookup_transform(
                self._base_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warning(
                f'Failed to resolve {pose.header.frame_id} into {self._base_frame}: {exc}'
            )
            return None

        return _compose_pose(_transform_to_pose_stamped(tf_msg), pose)

    def _resolve_grasp_pose(self, object_id: int) -> PoseStamped:
        return self._grasp_poses[object_id]

    def _prepare_grasp_poses(
        self,
        object_id: int,
    ) -> tuple[PoseStamped, PoseStamped]:
        grasp_pose = self._resolve_grasp_pose(object_id)
        pre_grasp_pose = _derive_pre_grasp_pose(
            grasp_pose,
            self._pre_grasp_offset_m,
        )
        return pre_grasp_pose, grasp_pose

    def _prepare_home_via_pose(self) -> PoseStamped:
        pre_grasp_pose, _ = self._prepare_grasp_poses(2)
        return pre_grasp_pose

    def _plan_cartesian_segment(
        self,
        current_pose: PoseStamped,
        target_pose: PoseStamped,
        motion_kind: str,
    ) -> None:
        while not self._cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Cartesian path service {self._cartesian_path_service} not available, waiting again...'
            )

        while not self._execute_trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                f'ExecuteTrajectory action {self._execute_trajectory_action} not available, waiting again...'
            )

        request = _build_cartesian_path_request(
            current_pose=current_pose,
            target_pose=target_pose,
            planning_group=self._planning_group,
            base_frame=self._base_frame,
            tip_link=self._tip_link,
            orientation_tolerance_rad=self._orientation_tolerance_rad,
            max_step_m=self._cartesian_max_step_m,
            jump_threshold=self._cartesian_jump_threshold,
            prismatic_jump_threshold=self._cartesian_prismatic_jump_threshold,
            revolute_jump_threshold=self._cartesian_revolute_jump_threshold,
            avoid_collisions=self._cartesian_avoid_collisions,
        )

        self._active_motion = motion_kind
        self._active_target_pose = target_pose
        future = self._cartesian_path_client.call_async(request)
        future.add_done_callback(self._cartesian_path_response_cb)

    def _start_cartesian_motion(
        self,
        target_pose: PoseStamped,
        motion_kind: str,
        unavailable_message: str,
    ) -> None:
        current_pose = self._lookup_current_pose()
        if current_pose is None:
            self._fail_task(unavailable_message)
            return

        resolved_target_pose = self._resolve_pose_in_base_frame(target_pose)
        if resolved_target_pose is None:
            self._fail_task(
                f'Cannot resolve target pose from {target_pose.header.frame_id} into '
                f'{self._base_frame}.'
            )
            return

        self._plan_cartesian_segment(
            current_pose,
            resolved_target_pose,
            motion_kind=motion_kind,
        )

    def _begin_pre_grasp_motion(self) -> None:
        if self._pending_pre_grasp_pose is None:
            self._fail_task(
                'Cannot start pre-grasp motion because no pending pre-grasp pose is available.'
            )
            return

        self._cancel_pre_grasp_timer()
        pending_pre_grasp_pose = self._pending_pre_grasp_pose
        self._pending_pre_grasp_pose = None
        self.get_logger().info(
            'Planning Cartesian move to the pre-grasp pose after settle wait.'
        )
        self._start_cartesian_motion(
            pending_pre_grasp_pose,
            motion_kind=MOTION_PRE_GRASP,
            unavailable_message=(
                'Ignoring start_grasp because the current tool pose is unavailable '
                'for the Cartesian pre-grasp motion.'
            ),
        )

    def _pre_grasp_settle_timer_cb(self) -> None:
        self._begin_pre_grasp_motion()

    def _should_accept_timeout_as_success(self, active_motion: str) -> bool:
        if self._active_target_pose is None:
            return False

        current_pose = self._lookup_current_pose()
        if current_pose is None:
            return False

        distance_m = _position_distance_m(current_pose, self._active_target_pose)
        if distance_m > self._execute_timeout_position_tolerance_m:
            self.get_logger().warning(
                f"ExecuteTrajectory motion {active_motion!r} timed out and the tool "
                f"is still {distance_m:.4f} m from target; not accepting timeout."
            )
            return False

        self.get_logger().warning(
            f"ExecuteTrajectory motion {active_motion!r} timed out, but the tool "
            f"is within {distance_m:.4f} m of target; treating it as success."
        )
        return True

    def _current_pose_distance_to_target(
        self,
        target_pose: PoseStamped,
    ) -> float | None:
        current_pose = self._lookup_current_pose()
        if current_pose is None:
            return None
        return _position_distance_m(current_pose, target_pose)

    def _require_target_reached(self, target_pose: PoseStamped, label: str) -> bool:
        distance_m = self._current_pose_distance_to_target(target_pose)
        if distance_m is None:
            self._fail_task(
                f'Arm reported success, but the current tool pose is unavailable for {label}.'
            )
            return False

        if distance_m > self._target_reached_tolerance_m:
            self._fail_task(
                f'Arm reported success, but {label} is still {distance_m:.4f} m away; '
                f'expected within {self._target_reached_tolerance_m:.4f} m.'
            )
            return False

        return True

    def _start_auto_open(self, location_label: str) -> None:
        while not self._gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                f'Gripper action {self._gripper_action} not available, waiting again...'
            )

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = _percent_to_joint(self._auto_open_percent)
        goal_msg.command.max_effort = self._gripper_max_effort

        self.get_logger().info(
            f'Opening gripper to {self._auto_open_percent:.2f}% at {location_label}.'
        )
        future = self._gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(self._gripper_goal_response_cb)

    def _start_home_motion(self, unavailable_message: str) -> None:
        self._clear_active_motion()
        self._pending_grasp_pose = None
        self._pending_pre_grasp_pose = None
        self._set_state(STATE_MOVING_TO_HOME)
        home_via_pose = self._prepare_home_via_pose()
        self.get_logger().info(
            'Moving to home via the object 2 pre-grasp waypoint: '
            f'{_format_pose_for_log(home_via_pose)}'
        )
        self._start_cartesian_motion(
            home_via_pose,
            motion_kind=MOTION_HOME_VIA,
            unavailable_message=unavailable_message,
        )
        return

    def _start_final_home_leg(self) -> None:
        self.get_logger().info(
            f'Moving from the object 2 pre-grasp waypoint to home_pose: '
            f'{_format_pose_for_log(self._home_pose)}'
        )
        self._start_cartesian_motion(
            self._home_pose,
            motion_kind=MOTION_HOME,
            unavailable_message=(
                'Cannot continue from the object 2 pre-grasp waypoint because '
                'the current tool pose is unavailable for the Cartesian home motion.'
            ),
        )

    def _start_grasp(self) -> None:
        if self._state != STATE_IDLE:
            self.get_logger().warning(
                f'Ignoring start_grasp while planner state is {self._state}.'
            )
            return

        if self._selected_object_id is None:
            self.get_logger().warning(
                'Ignoring start_grasp because no /grasp_object_id has been received yet.'
            )
            return

        pre_grasp_pose, grasp_pose = self._prepare_grasp_poses(self._selected_object_id)
        self._pending_grasp_pose = grasp_pose
        self._pending_pre_grasp_pose = pre_grasp_pose
        self._set_state(STATE_MOVING_TO_PRE_GRASP)
        self.get_logger().info(
            f'Starting grasp task for object {self._selected_object_id}: '
            f'pre-grasp x={pre_grasp_pose.pose.position.x:.3f} '
            f'y={pre_grasp_pose.pose.position.y:.3f} '
            f'z={pre_grasp_pose.pose.position.z:.3f}, '
            f'final z={grasp_pose.pose.position.z:.3f}'
        )
        self.get_logger().info(
            f'Resolved pre-grasp target: {_format_pose_for_log(pre_grasp_pose)}'
        )
        self.get_logger().info(
            f'Resolved grasp target: {_format_pose_for_log(grasp_pose)}'
        )
        if self._pre_grasp_settle_time_s > 0.0:
            self.get_logger().info(
                f'Waiting {self._pre_grasp_settle_time_s:.2f} s for the arm to settle before planning pre-grasp.'
            )
            self._pre_grasp_settle_timer = self.create_timer(
                self._pre_grasp_settle_time_s,
                self._pre_grasp_settle_timer_cb,
            )
            return

        self._begin_pre_grasp_motion()

    def _stop_grasp(self) -> None:
        if self._state != STATE_WAITING_AT_GRASP:
            self.get_logger().warning(
                f'Ignoring stop_grasp while planner state is {self._state}.'
            )
            return

        current_pose = self._lookup_current_pose()
        if current_pose is None:
            self._fail_task(
                'Ignoring stop_grasp because the current tool pose is unavailable.'
            )
            return

        lift_pose = _build_target_pose(current_pose, self._lift_height_m)
        self._set_state(STATE_MOVING_TO_DROP)
        self.get_logger().info(
            f'Stopping grasp task: lifting {self._lift_height_m:.3f} m before moving to drop_pose.'
        )
        self._plan_cartesian_segment(
            current_pose,
            lift_pose,
            motion_kind=MOTION_LIFT,
        )

    def _go_home(self) -> None:
        if self._state not in (STATE_IDLE, STATE_WAITING_AT_GRASP, STATE_ERROR):
            self.get_logger().warning(
                f'Ignoring go_home while planner state is {self._state}.'
            )
            return

        self.get_logger().info('Received go_home command.')
        self._start_home_motion(
            unavailable_message=(
                'Ignoring go_home because the current tool pose is unavailable for '
                'the Cartesian home motion.'
            )
        )

    def _cartesian_path_response_cb(self, future) -> None:
        active_motion = self._active_motion
        motion_label = 'Cartesian segment'
        if active_motion == MOTION_GRASP_APPROACH:
            motion_label = 'Cartesian grasp approach'
        elif active_motion == MOTION_LIFT:
            motion_label = 'Cartesian lift'
        elif active_motion == MOTION_DROP:
            motion_label = 'Cartesian drop motion'
        elif active_motion == MOTION_HOME_VIA:
            motion_label = 'Cartesian home-via motion'
        elif active_motion == MOTION_HOME:
            motion_label = 'Cartesian home motion'

        try:
            response = future.result()
        except Exception as exc:
            self._fail_task(f'Failed to compute {motion_label.lower()} path: {exc}')
            return

        if int(response.error_code.val) != MoveItErrorCodes.SUCCESS:
            self._fail_task(
                f'{motion_label} path failed with MoveIt error code '
                f'{int(response.error_code.val)}'
            )
            return

        if float(response.fraction) < self._cartesian_min_fraction:
            self._fail_task(
                f'{motion_label} path only achieved fraction='
                f'{float(response.fraction):.3f}; expected at least '
                f'{self._cartesian_min_fraction:.3f}'
            )
            return

        if not response.solution.joint_trajectory.points:
            self._fail_task(f'{motion_label} path returned an empty trajectory')
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        future = self._execute_trajectory_client.send_goal_async(goal)
        future.add_done_callback(self._execute_goal_response_cb)

    def _execute_goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_task(f'Failed to send ExecuteTrajectory goal: {exc}')
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_task('ExecuteTrajectory goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._execute_result_cb)

    def _gripper_goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_task(f'Failed to send automatic gripper-open goal: {exc}')
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_task('Automatic gripper-open goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result_cb)

    def _gripper_result_cb(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self._fail_task(f'Failed to receive automatic gripper-open result: {exc}')
            return

        self.get_logger().info(
            f'Automatic gripper-open completed with position={result.position:.4f} rad '
            f'reached_goal={result.reached_goal} stalled={result.stalled} '
            f'effort={result.effort:.1f}'
        )

        if self._state == STATE_OPENING_AT_DROP:
            self.get_logger().info(
                'Drop open complete; automatically continuing to home_pose.'
            )
            self._start_home_motion(
                unavailable_message=(
                    'Cannot continue from drop_pose because the current tool pose is '
                    'unavailable for the Cartesian home motion.'
                )
            )
            return

        if self._state == STATE_OPENING_AT_HOME:
            self.get_logger().info(
                'Home open complete; planner is idle and ready for the next start_grasp.'
            )
            self._reset_task(state=STATE_IDLE)
            return

        self._fail_task(
            f'Received an automatic gripper-open result while planner state is {self._state}.'
        )

    def _execute_result_cb(self, future) -> None:
        active_motion = self._active_motion

        try:
            result = future.result().result
        except Exception as exc:
            self._fail_task(f'Failed to receive ExecuteTrajectory result: {exc}')
            return

        error_code = int(result.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            timed_out_code = int(getattr(MoveItErrorCodes, 'TIMED_OUT', -6))
            if error_code == timed_out_code and self._should_accept_timeout_as_success(active_motion):
                error_code = MoveItErrorCodes.SUCCESS
            else:
                self._fail_task(
                    f'ExecuteTrajectory motion {active_motion!r} failed with '
                    f'error code {int(result.error_code.val)}'
                )
                return

        if active_motion == MOTION_LIFT:
            self.get_logger().info('Lift complete; moving to the drop_pose.')
            self._start_cartesian_motion(
                self._drop_pose,
                motion_kind=MOTION_DROP,
                unavailable_message=(
                    'Cannot continue from lift because the current tool pose is '
                    'unavailable for the Cartesian drop motion.'
                ),
            )
            return

        if active_motion == MOTION_PRE_GRASP:
            if self._pending_grasp_pose is None:
                self._fail_task(
                    'Reached the pre-grasp pose, but no grasp target is pending.'
                )
                return

            self.get_logger().info(
                'Reached the pre-grasp pose; starting Cartesian descend to the grasp pose.'
            )
            self._set_state(STATE_MOVING_TO_GRASP)
            self._start_cartesian_motion(
                self._pending_grasp_pose,
                motion_kind=MOTION_GRASP_APPROACH,
                unavailable_message=(
                    'Cannot continue from pre-grasp because the current tool pose '
                    'is unavailable for the Cartesian grasp approach.'
                ),
            )
            return

        if active_motion == MOTION_GRASP_APPROACH:
            target_pose = self._pending_grasp_pose or self._active_target_pose
            if target_pose is None:
                self._fail_task(
                    'Reached the grasp approach result, but there is no grasp target to verify.'
                )
                return
            if not self._require_target_reached(target_pose, 'the final grasp pose'):
                return

            self.get_logger().info(
                'Reached the final grasp pose; waiting for gripper commands.'
            )
            self._clear_active_motion()
            self._pending_grasp_pose = None
            self._pending_pre_grasp_pose = None
            self._set_state(STATE_WAITING_AT_GRASP)
            return

        if active_motion == MOTION_DROP:
            target_pose = self._active_target_pose or self._drop_pose
            if not self._require_target_reached(target_pose, 'drop_pose'):
                return

            self.get_logger().info(
                'Reached the drop_pose; opening the gripper before continuing home.'
            )
            self._clear_active_motion()
            self._set_state(STATE_OPENING_AT_DROP)
            self._start_auto_open('drop_pose')
            return

        if active_motion == MOTION_HOME_VIA:
            target_pose = self._active_target_pose
            if target_pose is None:
                self._fail_task(
                    'Reached the home-via result, but there is no waypoint target to verify.'
                )
                return
            if not self._require_target_reached(
                target_pose,
                'the object 2 pre-grasp home waypoint',
            ):
                return

            self.get_logger().info(
                'Reached the object 2 pre-grasp waypoint; continuing to home_pose.'
            )
            self._clear_active_motion()
            self._set_state(STATE_MOVING_TO_HOME)
            self._start_final_home_leg()
            return

        if active_motion == MOTION_HOME:
            target_pose = self._active_target_pose or self._home_pose
            if not self._require_target_reached(target_pose, 'home_pose'):
                return

            self.get_logger().info(
                'Reached the home_pose; opening the gripper.'
            )
            self._clear_active_motion()
            self._set_state(STATE_OPENING_AT_HOME)
            self._start_auto_open('home_pose')
            return

        self._reset_task(state=STATE_IDLE)

    def _web_command_cb(self, msg: String) -> None:
        try:
            command = parse_web_command(msg.data)
        except ValueError as exc:
            self.get_logger().warning(
                f'Ignoring invalid /web_command payload {msg.data!r}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        if is_start_grasp(command):
            self._start_grasp()
            return

        if is_stop_grasp(command):
            self._stop_grasp()
            return

        if is_go_home(command):
            self._go_home()
            return


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
