import importlib.util
import os
from pathlib import Path

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros.actions')


def _load_launch_description(filename: str):
    os.environ.setdefault('ROS_LOG_DIR', '/tmp')

    path = Path(__file__).with_name(filename)
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _node_parameter_map(node) -> dict[str, str]:
    param_map = node.__dict__['_Node__parameters'][0]
    extracted = {}
    for key_parts, value_parts in param_map.items():
        key = ''.join(part.text for part in key_parts)
        value = value_parts[0]
        extracted[key] = ''.join(part.text for part in value.variable_name)
    return extracted


def test_action_server_launch_forwards_modbus_settings_only_to_manager():
    launch_description = _load_launch_description('robotiq_2f85_action_server.launch.py')

    argument_names = [
        entity.name
        for entity in launch_description.entities
        if entity.__class__.__name__ == 'DeclareLaunchArgument'
    ]
    assert 'modbus_index' in argument_names
    assert 'modbus_probe_indices' not in argument_names

    manager_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'gripper_modbus_manager'
    ]
    assert len(manager_nodes) == 1
    manager_params = _node_parameter_map(manager_nodes[0])
    assert manager_params['modbus_index'] == 'modbus_index'
    assert 'modbus_probe_indices' not in manager_params

    proxy_nodes = [
        entity
        for entity in launch_description.entities
        if getattr(entity, '_Node__node_executable', None) == 'robotiq_2f85_action_server'
    ]
    assert len(proxy_nodes) == 1
    assert proxy_nodes[0].__dict__['_Node__parameters'] == []
