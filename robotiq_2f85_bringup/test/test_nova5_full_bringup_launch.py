import importlib.util
import os
from pathlib import Path

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros.actions')


def _load_launch_module(filename: str):
    os.environ.setdefault('ROS_LOG_DIR', '/tmp')

    path = Path(__file__).resolve().parents[1] / 'launch' / filename
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _node_parameter_map(node) -> dict[str, str]:
    def _stringify(value) -> str:
        if isinstance(value, (list, tuple)):
            return ''.join(_stringify(item) for item in value)
        if hasattr(value, 'variable_name'):
            return ''.join(part.text for part in value.variable_name)
        if hasattr(value, 'text'):
            return value.text
        return str(value)

    extracted = {}
    for param_map in node.__dict__['_Node__parameters']:
        if not isinstance(param_map, dict):
            continue
        for key_parts, value_parts in param_map.items():
            key = ''.join(part.text for part in key_parts)
            extracted[key] = _stringify(value_parts)
    return extracted


def _node_arguments(node) -> str:
    return ''.join(str(argument) for argument in node.__dict__.get('_Node__arguments', []))


class _FakeMoveItConfig:
    robot_description = {'robot_description': 'fake'}
    robot_description_semantic = {'robot_description_semantic': 'fake'}
    robot_description_kinematics = {'robot_description_kinematics': 'fake'}
    planning_pipelines = {'planning_pipelines': 'fake'}

    def to_dict(self):
        return {'robot_description': 'fake'}


class _FakeMoveItConfigsBuilder:
    def __init__(self, *args, **kwargs):
        pass

    def robot_description(self, *args, **kwargs):
        return self

    def robot_description_semantic(self, *args, **kwargs):
        return self

    def planning_pipelines(self, *args, **kwargs):
        return self

    def to_moveit_configs(self):
        return _FakeMoveItConfig()


def test_full_bringup_launch_includes_gripper_and_grasp_web_command_nodes(monkeypatch):
    module = _load_launch_module('nova5_full_bringup.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package_name: '/tmp')
    monkeypatch.setattr(module, 'MoveItConfigsBuilder', _FakeMoveItConfigsBuilder)

    launch_description = module.generate_launch_description()

    gripper_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'gripper_cmd_sender'
    ]
    assert len(gripper_nodes) == 1
    assert '/web_command' in _node_parameter_map(gripper_nodes[0])['command_topic']

    grasp_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'grasp_task_planner'
    ]
    assert len(grasp_nodes) == 1
    grasp_params = _node_parameter_map(grasp_nodes[0])
    assert '/web_command' in grasp_params['command_topic']
    assert '/grasp_object_id' in grasp_params['object_id_topic']

    apriltag_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'apriltag_node'
    ]
    assert len(apriltag_nodes) == 1

    apriltag_localizer_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'apriltag_arm_planner'
    ]
    assert len(apriltag_localizer_nodes) == 0

    apriltag_printer_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'apriltag_pose_printer'
    ]
    assert len(apriltag_printer_nodes) == 1
    printer_params = _node_parameter_map(apriltag_printer_nodes[0])
    assert '/detections' in printer_params['detection_topic']
    assert printer_params['anchor_tag_id'] == '11'
    assert 'base_link' in printer_params['anchor_parent_frame']
    assert 'latched_tag_11' in printer_params['anchor_child_frame']
    assert printer_params['tag_ids'] == '01234'
    assert 'latched_tag_11' in printer_params['frames_to_log']
    assert 'base_link' in printer_params['frames_to_log']

    fixed_tag_tf_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_name', None) == 'fixed_map_tag_11_tf'
    ]
    assert len(fixed_tag_tf_nodes) == 1
    fixed_tag_tf_arguments = _node_arguments(fixed_tag_tf_nodes[0])
    assert 'map_tag_11' in fixed_tag_tf_arguments
    assert '0.222' in fixed_tag_tf_arguments
    assert '0.223' in fixed_tag_tf_arguments
    assert '0.794' in fixed_tag_tf_arguments

    move_group_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'move_group'
    ]
    assert len(move_group_nodes) == 1
    move_group_params = _node_parameter_map(move_group_nodes[0])
    assert move_group_params['trajectory_execution.allowed_execution_duration_scaling'] == '4.0'
    assert move_group_params['trajectory_execution.allowed_goal_duration_margin'] == '2.0'
    assert move_group_params['trajectory_execution.allowed_start_tolerance'] == '0.05'

    table_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'planning_scene_table'
    ]
    assert len(table_nodes) == 1
    table_params = _node_parameter_map(table_nodes[0])
    assert 'base_link' in table_params['frame_id']
    assert '0.7' in table_params['size_x_m']
    assert '0.7' in table_params['size_y_m']
    assert '0.79' in table_params['size_z_m']
    assert '0.57' in table_params['center_y_m']
