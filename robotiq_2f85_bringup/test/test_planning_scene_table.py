from pathlib import Path
import sys

import pytest

pytest.importorskip('moveit_msgs.msg')
pytest.importorskip('moveit_msgs.srv')
pytest.importorskip('shape_msgs.msg')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotiq_2f85_bringup import planning_scene_table


def test_compute_table_center_z_uses_base_height_reference():
    center_z = planning_scene_table._compute_table_center_z(
        table_height_m=0.8,
        robot_base_height_above_floor_m=0.77,
    )

    assert center_z == pytest.approx(-0.37)


def test_build_table_collision_object_uses_requested_geometry():
    collision_object = planning_scene_table._build_table_collision_object(
        frame_id='base_link',
        object_id='experiment_table',
        size_x_m=0.7,
        size_y_m=0.7,
        size_z_m=0.8,
        center_x_m=0.0,
        center_y_m=0.2,
        center_z_m=-0.37,
    )

    assert collision_object.header.frame_id == 'base_link'
    assert collision_object.id == 'experiment_table'
    assert collision_object.primitives[0].type == collision_object.primitives[0].BOX
    assert collision_object.primitives[0].dimensions == pytest.approx([0.7, 0.7, 0.8])
    assert collision_object.primitive_poses[0].position.x == pytest.approx(0.0)
    assert collision_object.primitive_poses[0].position.y == pytest.approx(0.2)
    assert collision_object.primitive_poses[0].position.z == pytest.approx(-0.37)
    assert collision_object.primitive_poses[0].orientation.w == pytest.approx(1.0)


def test_build_planning_scene_wraps_table_as_diff_world_object():
    table_object = planning_scene_table._build_table_collision_object(
        frame_id='base_link',
        object_id='experiment_table',
        size_x_m=0.7,
        size_y_m=0.7,
        size_z_m=0.8,
        center_x_m=0.0,
        center_y_m=0.2,
        center_z_m=-0.37,
    )

    scene = planning_scene_table._build_planning_scene(table_object)

    assert scene.is_diff is True
    assert len(scene.world.collision_objects) == 1
    assert scene.world.collision_objects[0].id == 'experiment_table'


@pytest.mark.parametrize('value', [0.0, -1.0])
def test_validate_positive_rejects_non_positive_values(value):
    with pytest.raises(ValueError):
        planning_scene_table._validate_positive(value, 'size_x_m')
