from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('modbus_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('modbus_port', default_value='60000'),
        DeclareLaunchArgument('slave_id', default_value='9'),
        DeclareLaunchArgument('modbus_index', default_value='-1'),
        DeclareLaunchArgument('feedback_period_s', default_value='0.1'),
        # Single supported Modbus owner for the gripper stack under test.
        Node(
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
        ),
        # Public proxy only; all direct Modbus ownership stays in the manager.
        Node(
            package='robotiq_2f85_bringup',
            executable='robotiq_2f85_action_server',
            name='robotiq_2f85_action_server',
            output='screen',
        ),
    ])
