"""
nova5_full_bringup.launch.py
============================
Single-launch bringup for Nova5 + Robotiq 2F-85 + wrist camera + AprilTag
arm planner.

TF chain (no world anchor required for the planner):

  map → base_link → Link1 … Link6 → tool_link   [with use_table_map:=true]
                                  ↓  (static TF)
                           camera_link
                                  ↓  (apriltag_ros)
                               tag_0 … tag_5

The apriltag_arm_planner transforms detected tags from camera_link
→ base_link entirely through the robot's own TF tree; no external world/map
frame is required.

With use_table_map:=true (default) a static TF is published:

  map → base_link   (xyz = 0 0 TABLE_HEIGHT_M)

This places the robot base directly TABLE_HEIGHT_M metres above the map/floor
frame and removes the extra world/dummy root links from the TF tree.

Nodes started
-------------
  1.  dobot_bringup_v3/dobot_bringup     – arm TCP driver
  2.  dobot_bringup_v3/feedback          – arm joint-state publisher
  3.  robot_state_publisher              – nova5_2f85 URDF TF tree
  4.  nova5_2f85_joint_states            – bridges /joint_states_robot → /joint_states
  5.  dobot_moveit/action_move_server    – MoveIt FollowJointTrajectory → ServoJ
  6.  robotiq_2f85_action_server         – gripper Modbus action server
  7.  moveit_ros_move_group/move_group   – MoveIt planning & execution
  8.  camera_ros/camera_node             – USB camera (1920*1080 MJPG)
  9.  apriltag_ros/apriltag_node         – 36h11 tag detector, IDs 0-5
  10. tf2_ros/static_transform_publisher – robotiq_85_base_link → camera_link
  11. tf2_ros/static_transform_publisher – map → base_link  [conditional]
  12. rviz2                              – MoveIt Motion Planning [conditional]

Launch arguments
----------------
  cam_x / cam_y / cam_z           Camera translation from robotiq_85_base_link (m).
                                    PLACEHOLDER – measure the physical offset.
  cam_roll / cam_pitch / cam_yaw  Camera rotation from robotiq_85_base_link (rad).
  modbus_host / modbus_port / slave_id  Gripper Modbus connection.
  use_table_map   (true/false)    Publish map→base_link table TF.
  use_rviz        (true/false)    Open RViz with MoveIt panel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# Height of the table surface above the floor (map frame) in metres.
TABLE_HEIGHT_M = 1.0


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
            f'{TABLE_HEIGHT_M} m above the map frame'))

    # Camera mounting transform: robotiq_85_base_link → camera_link.
    # IMPORTANT: replace the default values with measured offsets.
    # Optical-frame convention: +X right, +Y down, +Z forward (into scene).
    # static_transform_publisher order: x y z  yaw pitch roll  parent child
    cam_x_arg     = DeclareLaunchArgument(
        'cam_x',     default_value='0.04',
        description='Camera X offset from robotiq_85_base_link [m] – measure and update')
    cam_y_arg     = DeclareLaunchArgument(
        'cam_y',     default_value='0.0',
        description='Camera Y offset from robotiq_85_base_link [m]')
    cam_z_arg     = DeclareLaunchArgument(
        'cam_z',     default_value='0.05',
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

    # Gripper Modbus parameters
    modbus_host_arg = DeclareLaunchArgument(
        'modbus_host', default_value='127.0.0.1',
        description='Modbus TCP host for the Robotiq 2F-85 gripper')
    modbus_port_arg = DeclareLaunchArgument(
        'modbus_port', default_value='60000',
        description='Modbus TCP port')
    slave_id_arg = DeclareLaunchArgument(
        'slave_id', default_value='9',
        description='Modbus slave ID')

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

    # ── 6. Robotiq 2F-85 gripper Modbus action server ─────────────────────────
    gripper_node = Node(
        package='robotiq_2f85_bringup',
        executable='robotiq_2f85_action_server',
        name='robotiq_2f85_action_server',
        output='screen',
        parameters=[{
            'modbus_host': LaunchConfiguration('modbus_host'),
            'modbus_port': LaunchConfiguration('modbus_port'),
            'slave_id':    LaunchConfiguration('slave_id'),
        }],
    )

    # ── 7. MoveIt move_group ──────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            moveit_config.planning_pipelines,
        ],
    )

    # ── 8. USB camera (camera_ros / libcamera) ────────────────────────────────
    # Publishes /camera/image_raw and /camera/camera_info.
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'format': 'MJPEG',
            'width':  1920,
            'height': 1080,
            'camera_info_url': [
                'file://',
                PathJoinSubstitution([bringup_share, 'config', 'camera_calibration.yaml']),
            ],
        }],
    )

    # ── 9. AprilTag detector (36h11, IDs 0-5) ────────────────────────────────
    # Publishes /detections (apriltag_msgs/AprilTagDetectionArray) and
    # per-tag TF transforms relative to camera_link.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect',  '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[PathJoinSubstitution([bringup_share, 'config', 'tags_36h11.yaml'])],
    )

    # ── 10. Static TF: tool_link → camera_link ───────────────────────────────
    # Connects the camera into the robot TF tree so that tag poses published
    # in the camera frame can be looked up relative to base_link by the
    # apriltag_arm_planner – no world/map frame is needed for this chain.
    #
    # Full resolved chain:
    #   base_link → … → Link6 → robotiq_85_base_link → camera_link → tag_N
    #
    # static_transform_publisher CLI order: x y z  yaw pitch roll  parent child
    static_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_tool_tf',
        output='screen',
        arguments=[
            LaunchConfiguration('cam_x'),
            LaunchConfiguration('cam_y'),
            LaunchConfiguration('cam_z'),
            LaunchConfiguration('cam_yaw'),    # yaw comes before pitch/roll
            LaunchConfiguration('cam_pitch'),
            LaunchConfiguration('cam_roll'),
            'robotiq_85_base_link',            # parent
            'camera_link', # child
        ],
    )

    # Bridge common camera frame names so camera_ros/apriltag_ros stay in the
    # same TF tree even if they use different defaults.
    camera_link_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_tf',
        output='screen',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'camera_link',
            'camera',
        ],
    )

    # ── 11. [Optional] Static TF: map → base_link (table scene) ──────────────
    # Publishes the robot base directly at TABLE_HEIGHT_M above the map
    # (floor) frame. All AprilTags detected on the table surface remain
    # reachable from base_link through the robot + camera TF chain.
    #
    # Enabled by default (use_table_map:=true).
    # Disable with use_table_map:=false if you have a different map setup.
    table_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='table_map_tf',
        output='screen',
        arguments=[
            '0', '0', str(TABLE_HEIGHT_M),  # robot base_link is TABLE_HEIGHT_M m above floor
            '0', '0', '0',                  # no rotation: table is level
            'map',   # parent (floor / absolute reference)
            'base_link', # child
        ],
        condition=IfCondition(LaunchConfiguration('use_table_map')),
    )

    # ── 12. [Optional] RViz ───────────────────────────────────────────────────
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

    # ── Task planner ──────────────────────────────────────────────────────────
    # Subscribes to /detections, transforms each tag to base_link, computes a
    # goal pose 1 m along the tag's Z axis, and sends it to move_group.
    planner_node = Node(
        package='robotiq_2f85_bringup',
        executable='apriltag_arm_planner',
        output='screen',
    )

    return LaunchDescription([
        # ── Declare arguments ──────────────────────────────────────────────
        use_rviz_arg,
        use_table_map_arg,
        cam_x_arg, cam_y_arg, cam_z_arg,
        cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
        modbus_host_arg, modbus_port_arg, slave_id_arg,

        # ── Arm hardware ───────────────────────────────────────────────────
        dobot_bringup_node,    # TCP driver
        dobot_feedback_node,   # joint-state publisher

        # ── Robot model / TF ───────────────────────────────────────────────
        rsp_node,              # base_link → … → tool_link
        joint_states_node,     # /joint_states_robot → /joint_states
        static_camera_tf,      # robotiq_85_base_link → camera_link
        camera_link_to_camera_tf,   # camera_link → camera
        table_map_tf,          # map → base_link  [conditional]

        # ── Controllers / action servers ───────────────────────────────────
        action_server_node,    # MoveIt → Dobot ServoJ
        gripper_node,          # Robotiq 2F-85 Modbus

        # ── MoveIt ─────────────────────────────────────────────────────────
        move_group_node,

        # ── Perception ─────────────────────────────────────────────────────
        camera_node,           # /camera/image_raw + /camera/camera_info
        apriltag_node,         # /detections + per-tag TF

        # ── Task planner ───────────────────────────────────────────────────
        planner_node,          # 1 m approach along tag Z axis

        # ── Visualisation ──────────────────────────────────────────────────
        rviz_node,             # [conditional]
    ])
