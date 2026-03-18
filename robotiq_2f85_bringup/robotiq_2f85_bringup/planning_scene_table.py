#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def _compute_table_center_z(
    table_height_m: float,
    robot_base_height_above_floor_m: float,
) -> float:
    return (0.5 * float(table_height_m)) - float(robot_base_height_above_floor_m)


def _build_table_collision_object(
    *,
    frame_id: str,
    object_id: str,
    size_x_m: float,
    size_y_m: float,
    size_z_m: float,
    center_x_m: float,
    center_y_m: float,
    center_z_m: float,
) -> CollisionObject:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [
        float(size_x_m),
        float(size_y_m),
        float(size_z_m),
    ]

    pose = Pose()
    pose.position.x = float(center_x_m)
    pose.position.y = float(center_y_m)
    pose.position.z = float(center_z_m)
    pose.orientation.w = 1.0

    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = object_id
    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [pose]
    collision_object.operation = CollisionObject.ADD
    return collision_object


def _build_planning_scene(table_object: CollisionObject) -> PlanningScene:
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [table_object]
    return scene


def _validate_positive(value: float, parameter_name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(
            f'{parameter_name} must be a finite positive value, got {value!r}'
        )
    return value


class PlanningSceneTable(Node):
    def __init__(self):
        super().__init__('planning_scene_table')

        self.declare_parameter('apply_planning_scene_service', '/apply_planning_scene')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('object_id', 'experiment_table')
        self.declare_parameter('size_x_m', 0.7)
        self.declare_parameter('size_y_m', 0.7)
        self.declare_parameter('size_z_m', 0.8)
        self.declare_parameter('center_x_m', 0.0)
        self.declare_parameter('center_y_m', 0.2)
        self.declare_parameter('robot_base_height_above_floor_m', 0.77)
        self.declare_parameter('retry_period_s', 1.0)

        self._apply_service = str(
            self.get_parameter('apply_planning_scene_service').value
        )
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._object_id = str(self.get_parameter('object_id').value)
        self._size_x_m = _validate_positive(
            self.get_parameter('size_x_m').value,
            'size_x_m',
        )
        self._size_y_m = _validate_positive(
            self.get_parameter('size_y_m').value,
            'size_y_m',
        )
        self._size_z_m = _validate_positive(
            self.get_parameter('size_z_m').value,
            'size_z_m',
        )
        self._center_x_m = float(self.get_parameter('center_x_m').value)
        self._center_y_m = float(self.get_parameter('center_y_m').value)
        self._robot_base_height_above_floor_m = float(
            self.get_parameter('robot_base_height_above_floor_m').value
        )
        self._retry_period_s = _validate_positive(
            self.get_parameter('retry_period_s').value,
            'retry_period_s',
        )

        self._request_in_flight = False
        self._scene_applied = False
        self._waiting_for_service_logged = False

        self._apply_client = self.create_client(
            ApplyPlanningScene,
            self._apply_service,
        )
        self._retry_timer = self.create_timer(
            self._retry_period_s,
            self._maybe_apply_table,
        )

        self.get_logger().info(
            f'Planning-scene table publisher ready: size='
            f'({self._size_x_m:.3f}, {self._size_y_m:.3f}, {self._size_z_m:.3f}) m, '
            f'center=({self._center_x_m:.3f}, {self._center_y_m:.3f}) m in '
            f'{self._frame_id}, base height={self._robot_base_height_above_floor_m:.3f} m.'
        )

    def _build_request(self) -> ApplyPlanningScene.Request:
        center_z_m = _compute_table_center_z(
            self._size_z_m,
            self._robot_base_height_above_floor_m,
        )
        table_object = _build_table_collision_object(
            frame_id=self._frame_id,
            object_id=self._object_id,
            size_x_m=self._size_x_m,
            size_y_m=self._size_y_m,
            size_z_m=self._size_z_m,
            center_x_m=self._center_x_m,
            center_y_m=self._center_y_m,
            center_z_m=center_z_m,
        )

        request = ApplyPlanningScene.Request()
        request.scene = _build_planning_scene(table_object)
        return request

    def _maybe_apply_table(self) -> None:
        if self._scene_applied or self._request_in_flight:
            return

        if not self._apply_client.wait_for_service(timeout_sec=0.0):
            if not self._waiting_for_service_logged:
                self.get_logger().info(
                    f'Waiting for MoveIt planning scene service {self._apply_service}...'
                )
                self._waiting_for_service_logged = True
            return

        if self._waiting_for_service_logged:
            self.get_logger().info(
                f'MoveIt planning scene service {self._apply_service} is available.'
            )
            self._waiting_for_service_logged = False

        request = self._build_request()
        self._request_in_flight = True
        future = self._apply_client.call_async(request)
        future.add_done_callback(self._apply_response_cb)

    def _apply_response_cb(self, future) -> None:
        self._request_in_flight = False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to apply the planning-scene table: {exc}'
            )
            return

        if not response.success:
            self.get_logger().warning(
                'MoveIt rejected the planning-scene table update; retrying.'
            )
            return

        self._scene_applied = True
        if self._retry_timer is not None:
            self._retry_timer.cancel()

        top_z_m = (
            _compute_table_center_z(
                self._size_z_m,
                self._robot_base_height_above_floor_m,
            )
            + (0.5 * self._size_z_m)
        )
        self.get_logger().info(
            f'Applied planning-scene table {self._object_id!r} in {self._frame_id}: '
            f'center=({self._center_x_m:.3f}, {self._center_y_m:.3f}, '
            f'{_compute_table_center_z(self._size_z_m, self._robot_base_height_above_floor_m):.3f}) m, '
            f'top_z={top_z_m:.3f} m.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneTable()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
