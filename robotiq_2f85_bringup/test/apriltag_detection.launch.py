from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── camera_ros parameters ──────────────────────────────────────────
        # Do not force a camera name. camera_ros will auto-select the default
        # detected camera, which is safer across reboots and USB re-enumeration.
        DeclareLaunchArgument('format', default_value='YUYV',
                              description='Pixel format: YUYV or MJPEG'),
        DeclareLaunchArgument('width',  default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=['file://', PathJoinSubstitution([
                FindPackageShare('robotiq_2f85_bringup'),
                'config', 'camera_calibration.yaml',
            ])],
            description='Calibration URL (file:///absolute/path/to/calibration.yaml)',
        ),

        # ── AprilTag parameters ────────────────────────────────────────────
        DeclareLaunchArgument(
            'tags_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('robotiq_2f85_bringup'),
                'config', 'tags_36h11.yaml',
            ]),
            description='Path to apriltag tags YAML (edit config/tags_36h11.yaml in this package)',
        ),

        # ── Camera node (camera_ros / libcamera) ───────────────────────────
        # Publishes /camera/image_raw  and  /camera/camera_info
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'format':          ParameterValue(LaunchConfiguration('format'), value_type=str),
                'width':           LaunchConfiguration('width'),
                'height':          LaunchConfiguration('height'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }],
        ),

        # ── AprilTag detector ──────────────────────────────────────────────
        # image_rect  → /camera/image_raw   (no rectification node needed)
        # camera_info → /camera/camera_info
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect',  '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
            parameters=[LaunchConfiguration('tags_config')],
        ),
    ])
