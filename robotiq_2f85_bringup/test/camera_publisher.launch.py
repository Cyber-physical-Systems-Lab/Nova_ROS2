from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='\_SB_.PC00.XHCI.RHUB.HS13-13:1.0-0bda:5a63',
            description='libcamera camera ID (from camera_ros startup log)',
        ),
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
            description='Calibration URL  (file:///absolute/path/to/calibration.yaml)',
        ),
         # ── Camera node (camera_ros / libcamera) ───────────────────────────
        # Publishes /camera/image_raw  and  /camera/camera_info
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'camera':          ParameterValue(LaunchConfiguration('camera_name'), value_type=str),
                'format':          ParameterValue(LaunchConfiguration('format'), value_type=str),
                'width':           LaunchConfiguration('width'),
                'height':          LaunchConfiguration('height'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }],
        ),
    ])