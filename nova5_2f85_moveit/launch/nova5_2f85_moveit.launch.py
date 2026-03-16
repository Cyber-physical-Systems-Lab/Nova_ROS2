import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ── package share directories ──────────────────────────────────────
    moveit_pkg = get_package_share_directory('nova5_2f85_moveit')
    desc_pkg   = get_package_share_directory('nova5_2f85_description')

    # ── MoveIt config ──────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("nova5_robot_2f85", package_name="nova5_2f85_moveit")
        .robot_description(
            file_path=os.path.join(desc_pkg, "urdf", "nova5_2f85.urdf"),
            mappings={
                "prefix": "",
                "sim_ignition": "false",
                "sim_isaac": "false",
                "mock_sensor_commands": "false",
            },
        )
        .robot_description_semantic(
            file_path="config/nova5_robot_2f85.srdf"
        )
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl'],
            load_all=False,
        )
        .to_moveit_configs()
    )

    # ── launch arguments ───────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz with MoveIt MotionPlanning panel')

    use_rviz = LaunchConfiguration('use_rviz')

    # ── 1. robot_state_publisher ───────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': False},
        ],
    )

    # ── 2. Joint-state bridge ──────────────────────────────────────────
    # Reads real arm positions from /joint_states_robot (dobot_bringup_v3)
    # and publishes /joint_states with arm + gripper mimic joints so that
    # robot_state_publisher and move_group see the actual robot pose.
    # (Replaces ros2_control mock hardware + joint_state_broadcaster which
    #  only reflected commanded positions, not the real robot.)
    joint_states_node = Node(
        package='robotiq_2f85_bringup',
        executable='nova5_2f85_joint_states',
        output='screen',
    )

    # ── 3. FollowJointTrajectory action server ─────────────────────────
    # Receives planned trajectories from move_group and forwards each
    # waypoint to the real Dobot arm via the ServoJ TCP API service.
    # DOBOT_TYPE=nova5 makes the action server advertise the topic that
    # moveit_controllers.yaml points to:
    #   /nova5_group_controller/follow_joint_trajectory
    action_server_node = Node(
        package='dobot_moveit',
        executable='action_move_server',
        output='screen',
        additional_env={'DOBOT_TYPE': 'nova5'},
    )

    # ── 4. move_group ──────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            moveit_config.planning_pipelines,
        ],
    )

    # ── 5. RViz ────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(moveit_pkg, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        rsp_node,
        joint_states_node,
        action_server_node,
        move_group_node,
        rviz_node,
    ])
