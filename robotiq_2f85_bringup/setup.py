from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotiq_2f85_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpslab',
    maintainer_email='cpslab@todo.todo',
    description='Dobot Nova5 gripper and camera bringup nodes.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robotiq_2f85_action_server = robotiq_2f85_bringup.action_server:main',
            'gripper_cmd_sender = robotiq_2f85_bringup.gripper_cmd_sender:main',
            'camera_publisher = robotiq_2f85_bringup.camera_publisher:main',
            'nova5_2f85_joint_states = robotiq_2f85_bringup.nova5_2f85_joint_states:main',
            'apriltag_arm_planner = robotiq_2f85_bringup.apriltag_arm_planner:main',
        ],
    },
)
