"""
Single-launch bringup for Nova5 + Robotiq 2F-85 + wrist camera,
fixed-map grasp tasks, and a planning-scene table.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# Robot base height above the floor (map frame) in metres.
ROBOT_BASE_HEIGHT_M = 0.77

# Conservative planning-scene table box, expressed in base_link.
PLANNING_TABLE_SIZE_X_M = 0.70
PLANNING_TABLE_SIZE_Y_M = 0.70
PLANNING_TABLE_SIZE_Z_M = 0.79
PLANNING_TABLE_CENTER_X_M = 0.00
PLANNING_TABLE_CENTER_Y_M = 0.57
TRAJECTORY_EXECUTION_ALLOWED_EXECUTION_DURATION_SCALING = 4.0
TRAJECTORY_EXECUTION_ALLOWED_GOAL_DURATION_MARGIN_S = 2.0
TRAJECTORY_EXECUTION_ALLOWED_START_TOLERANCE_RAD = 0.05
MAP_TAG_11_X_M = 0.222
MAP_TAG_11_Y_M = 0.223
MAP_TAG_11_Z_M = 0.794


def generate_launch_description():

    bringup_share = FindPackageShare('robotiq_2f85_bringup')
    moveit_pkg    = get_package_share_directory('nova5_2f85_moveit')
    desc_pkg      = get_package_share_directory('nova5_2f85_description')

    # ── MoveIt configuration ─────────────────────────────────────────────────
    # Mirrors nova5_2f85_moveit.launch.py exactly.
    moveit_config = (
        MoveItConfigsBuilder('nova5_robot_2f85', package_name='nova5_2f85_moveit')
        .robot_description(
            file_path=os.path.join(desc_pkg, 'urdf', 'nova5_2f85.urdf'),
            mappings={
                'prefix':               '',
                'sim_ignition':         'false',
                'sim_isaac':            'false',
                'mock_sensor_commands': 'false',
            },
        )
        .robot_description_semantic(file_path='config/nova5_robot_2f85.srdf')
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl'],
            load_all=False,
        )
        .to_moveit_configs()
    )

    # ── Launch arguments ─────────────────────────────────────────────────────

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz with the MoveIt MotionPlanning panel')

    use_table_map_arg = DeclareLaunchArgument(
        'use_table_map', default_value='true',
        description=(
            f'Publish a static map→base_link TF placing the robot '
            f'{ROBOT_BASE_HEIGHT_M} m above the map frame'))
    use_planning_table_arg = DeclareLaunchArgument(
        'use_planning_table', default_value='true',
        description=(
            'Apply a conservative 0.7x0.7x0.8 m table collision object in MoveIt '
            'centered at x=0.0 m, y=0.57 m in base_link'))

    # Camera mounting transform: robotiq_85_base_link → camera_link.
    # IMPORTANT: replace the default values with measured offsets.
    # Optical-frame convention: +X right, +Y down, +Z forward (into scene).
    # static_transform_publisher order: x y z  yaw pitch roll  parent child
    cam_x_arg     = DeclareLaunchArgument(
        'cam_x',     default_value='-0.0429',
        description='Camera X offset from robotiq_85_base_link [m] – measure and update')
    cam_y_arg     = DeclareLaunchArgument(
        'cam_y',     default_value='-0.032',
        description='Camera Y offset from robotiq_85_base_link [m]')
    cam_z_arg     = DeclareLaunchArgument(
        'cam_z',     default_value='0.06',
        description='Camera Z offset from robotiq_85_base_link [m]')
    cam_roll_arg  = DeclareLaunchArgument(
        'cam_roll',  default_value='0.0',
        description='Camera roll from robotiq_85_base_link [rad]')
    cam_pitch_arg = DeclareLaunchArgument(
        'cam_pitch', default_value='0.0',
        description='Camera pitch from robotiq_85_base_link [rad]')
    cam_yaw_arg   = DeclareLaunchArgument(
        'cam_yaw',   default_value='0.0',
        description='Camera yaw from robotiq_85_base_link [rad]')

    # Gripper Modbus parameters.
    modbus_host_arg = DeclareLaunchArgument(
        'modbus_host', default_value='127.0.0.1',
        description='Modbus TCP host for the Robotiq 2F-85 gripper')
    modbus_port_arg = DeclareLaunchArgument(
        'modbus_port', default_value='60000',
        description='Modbus TCP port')
    slave_id_arg = DeclareLaunchArgument(
        'slave_id', default_value='9',
        description='Modbus slave ID')
    modbus_index_arg = DeclareLaunchArgument(
        'modbus_index', default_value='-1',
        description='Known Dobot Modbus index to adopt; -1 creates a new gripper channel automatically')
    feedback_period_arg = DeclareLaunchArgument(
        'feedback_period_s', default_value='0.1',
        description='Gripper feedback polling period in seconds')
    # ── 1 & 2. Dobot arm hardware ─────────────────────────────────────────────
    dobot_bringup_node = Node(
        package='dobot_bringup_v3',
        executable='dobot_bringup',
    )
    dobot_feedback_node = Node(
        package='dobot_bringup_v3',
        executable='feedback',
    )

    # ── 3. Robot state publisher (full arm + gripper TF tree) ─────────────────
    # Publishes: base_link → Link1…Link6 → tool_link
    #            + all gripper link transforms.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': False},
        ],
    )

    # ── 4. Joint-state bridge ─────────────────────────────────────────────────
    # Merges /joint_states_robot (arm, 6 DOF) with mimic gripper states and
    # republishes as /joint_states so robot_state_publisher and move_group
    # see the full 12-DOF state.
    joint_states_node = Node(
        package='robotiq_2f85_bringup',
        executable='nova5_2f85_joint_states',
    )

    # ── 5. MoveIt → Dobot trajectory bridge ───────────────────────────────────
    # Receives FollowJointTrajectory goals from move_group and forwards them
    # to the real arm via the ServoJ TCP API.
    action_server_node = Node(
        package='dobot_moveit',
        executable='action_move_server',
        additional_env={'DOBOT_TYPE': 'nova5'},
    )

    # ── 6. Shared Robotiq 2F-85 Modbus manager ───────────────────────────────
    # Single supported Modbus owner. It uses the configured Modbus index when
    # provided; otherwise it creates a new gripper channel and publishes
    # /web_feedback for web-facing gripper readback.
    gripper_manager_node = Node(
        package='robotiq_2f85_bringup',
        executable='gripper_modbus_manager',
        name='gripper_modbus_manager',
        output='screen',
        parameters=[{
            'modbus_host': LaunchConfiguration('modbus_host'),
            'modbus_port': LaunchConfiguration('modbus_port'),
            'slave_id': LaunchConfiguration('slave_id'),
            'modbus_index': LaunchConfiguration('modbus_index'),
            'feedback_period_s': LaunchConfiguration('feedback_period_s'),
        }],
    )

    # ── 7. Public Robotiq 2F-85 action proxy ─────────────────────────────────
    # Pure proxy: forwards public gripper goals to gripper_modbus_manager and
    # does not own or configure a separate Modbus channel.
    gripper_node = Node(
        package='robotiq_2f85_bringup',
        executable='robotiq_2f85_action_server',
        name='robotiq_2f85_action_server',
        output='screen',
    )

    web_command_node = Node(
        package='robotiq_2f85_bringup',
        executable='gripper_cmd_sender',
        name='gripper_cmd_sender',
        output='screen',
        parameters=[{
            'command_topic': '/web_command',
        }],
    )

    grasp_task_planner_node = Node(
        package='robotiq_2f85_bringup',
        executable='grasp_task_planner',
        name='grasp_task_planner',
        output='screen',
        parameters=[
            PathJoinSubstitution([bringup_share, 'config', 'grasp_task_planner.yaml']),
            {
                'command_topic': '/web_command',
                'object_id_topic': '/grasp_object_id',
            },
        ],
    )

    # ── 10. MoveIt move_group ─────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            moveit_config.planning_pipelines,
            {
                'trajectory_execution.allowed_execution_duration_scaling':
                    TRAJECTORY_EXECUTION_ALLOWED_EXECUTION_DURATION_SCALING,
                'trajectory_execution.allowed_goal_duration_margin':
                    TRAJECTORY_EXECUTION_ALLOWED_GOAL_DURATION_MARGIN_S,
                'trajectory_execution.allowed_start_tolerance':
                    TRAJECTORY_EXECUTION_ALLOWED_START_TOLERANCE_RAD,
            },
        ],
    )

    planning_table_node = Node(
        package='robotiq_2f85_bringup',
        executable='planning_scene_table',
        name='planning_scene_table',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'object_id': 'experiment_table',
            'size_x_m': PLANNING_TABLE_SIZE_X_M,
            'size_y_m': PLANNING_TABLE_SIZE_Y_M,
            'size_z_m': PLANNING_TABLE_SIZE_Z_M,
            'center_x_m': PLANNING_TABLE_CENTER_X_M,
            'center_y_m': PLANNING_TABLE_CENTER_Y_M,
            'robot_base_height_above_floor_m': ROBOT_BASE_HEIGHT_M,
        }],
        condition=IfCondition(LaunchConfiguration('use_planning_table')),
    )

    # camera_node = Node(
    #     package='camera_ros',
    #     executable='camera_node',
    #     name='camera',
    #     output='screen',
    #     parameters=[{
    #         'format': 'MJPEG',
    #         'width':  1920,
    #         'height': 1080,
    #         'camera_info_url': [
    #             'file://',
    #             PathJoinSubstitution([bringup_share, 'config', 'camera_calibration.yaml']),
    #         ],
    #     }],
    # )

    # apriltag_node = Node(
    #     package='apriltag_ros',
    #     executable='apriltag_node',
    #     name='apriltag_node',
    #     output='screen',
    #     remappings=[
    #         ('image_rect', '/camera/image_raw'),
    #         ('camera_info', '/camera/camera_info'),
    #     ],
    #     parameters=[
    #         PathJoinSubstitution([bringup_share, 'config', 'tags_36h11.yaml']),
    #     ],
    # )

    # apriltag_pose_printer_node = Node(
    #     package='robotiq_2f85_bringup',
    #     executable='apriltag_pose_printer',
    #     name='apriltag_pose_printer',
    #     output='screen',
    #     parameters=[{
    #         'detection_topic': '/detections',
    #         'anchor_tag_id': 11,
    #         'anchor_parent_frame': 'base_link',
    #         'anchor_child_frame': 'latched_tag_11',
    #         'tag_ids': [0, 1, 2, 3, 4],
    #         'frames_to_log': ['latched_tag_11', 'base_link'],
    #         'log_period_s': 0.5,
    #         'anchor_publish_period_s': 0.2,
    #     }],
    # )

    static_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_tool_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--frame-id', 'robotiq_85_base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    camera_link_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera',
        ],
    )

    table_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='table_map_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', str(ROBOT_BASE_HEIGHT_M),
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'base_link',
        ],
        condition=IfCondition(LaunchConfiguration('use_table_map')),
    )

    fixed_map_tag_11_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fixed_map_tag_11_tf',
        output='screen',
        arguments=[
            '--x', str(MAP_TAG_11_X_M),
            '--y', str(MAP_TAG_11_Y_M),
            '--z', str(MAP_TAG_11_Z_M),
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'map_tag_11',
        ],
    )

    # ── 18. [Optional] RViz ──────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(moveit_pkg, 'config', 'full_bringup.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        # ── Declare arguments ──────────────────────────────────────────────
        use_rviz_arg,
        use_table_map_arg,
        use_planning_table_arg,
        cam_x_arg, cam_y_arg, cam_z_arg,
        cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
        modbus_host_arg, modbus_port_arg, slave_id_arg, modbus_index_arg,
        feedback_period_arg,

        # ── Arm hardware ───────────────────────────────────────────────────
        dobot_bringup_node,    # TCP driver
        dobot_feedback_node,   # joint-state publisher

        # ── Robot model / TF ───────────────────────────────────────────────
        rsp_node,              # base_link → … → tool_link
        joint_states_node,     # /joint_states_robot → /joint_states
        static_camera_tf,      # robotiq_85_base_link → camera_link
        camera_link_to_camera_tf,   # camera_link → camera
        table_map_tf,          # map → base_link  [conditional]
        # fixed_map_tag_11_tf,   # map → map_tag_11
        planning_table_node,   # base_link-relative table collision object

        # ── Controllers / action servers ───────────────────────────────────
        action_server_node,    # MoveIt → Dobot ServoJ
        gripper_manager_node,  # Single supported Modbus owner + real /gripper/joint_states
        gripper_node,          # Public gripper action proxy
        web_command_node,      # /web_command gripper_pos routing
        grasp_task_planner_node,  # /web_command grasp-task routing

        # ── MoveIt ─────────────────────────────────────────────────────────
        move_group_node,
    
        # ── Perception ─────────────────────────────────────────────────────
        # camera_node,           # /camera/image_raw + /camera/camera_info
        # apriltag_node,         # AprilTag detections + tag_<id> TF
        # apriltag_pose_printer_node,  # tag_0..tag_4 position logger

        # ── Visualisation ──────────────────────────────────────────────────
        rviz_node,             # [conditional]
    ])
