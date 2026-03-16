from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('modbus_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('modbus_port', default_value='60000'),
        DeclareLaunchArgument('slave_id', default_value='9'),
        Node(
            package='robotiq_2f85_bringup',
            executable='robotiq_2f85_action_server',
            name='robotiq_2f85_action_server',
            output='screen',
            parameters=[{
                'modbus_host': LaunchConfiguration('modbus_host'),
                'modbus_port': LaunchConfiguration('modbus_port'),
                'slave_id': LaunchConfiguration('slave_id'),
            }],
        ),
    ])
