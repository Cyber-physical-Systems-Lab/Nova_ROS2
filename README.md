# Nova_ROS2

ROS 2 Humble packages for a Dobot Nova5 with a Robotiq 2F-85 gripper, MoveIt, and a wrist-mounted camera / AprilTag workflow.

This repository is the custom layer that sits on top of the upstream Dobot ROS 2 driver. It contains:

- A combined Nova5 + 2F-85 robot description
- A MoveIt configuration for the combined robot
- Bringup code that bridges Dobot joint feedback into `/joint_states`
- A full-system launch file that wires MoveIt, camera, TF, and AprilTag detection together
- An AprilTag relay node that republishes detected tags into the `map` frame

Platform target: Ubuntu 22.04 + ROS 2 Humble

## Repository layout

```text
src/Nova_ROS2/
├── nova5_2f85_description/
│   ├── urdf/nova5_2f85.urdf.xacro
│   └── meshes/
├── nova5_2f85_moveit/
│   ├── config/
│   └── launch/nova5_2f85_moveit.launch.py
└── robotiq_2f85_bringup/
    ├── config/
    │   ├── camera_calibration.yaml
    │   └── tags_36h11.yaml
    ├── launch/nova5_full_bringup.launch.py
    └── robotiq_2f85_bringup/
        ├── nova5_2f85_joint_states.py
        └── apriltag_arm_planner.py
```

## What each package does

### `nova5_2f85_description`

Provides the combined robot model for the Nova5 arm and Robotiq 2F-85 gripper, including URDF/Xacro and meshes.

### `nova5_2f85_moveit`

Provides the MoveIt configuration for the combined robot:

- SRDF
- kinematics
- joint limits
- controller config
- RViz config
- a MoveIt launch file: `nova5_2f85_moveit.launch.py`

### `robotiq_2f85_bringup`

Contains runtime launch/config/python code for the real robot setup:

- `nova5_full_bringup.launch.py`
  Starts Dobot bringup, MoveIt, TF publishers, camera, AprilTag detection, and the local helper nodes.
- `nova5_2f85_joint_states.py`
  Bridges `/joint_states_robot` from the Dobot driver into a combined `/joint_states` topic that also includes gripper mimic joints.
- `apriltag_arm_planner.py`
  Subscribes to `/detections`, resolves tag frames through TF, and republishes `map_tag_<id>` frames in the `map` frame.

## External dependencies

This repo expects the upstream Dobot ROS 2 stack to be present in the same workspace, especially:

- `dobot_bringup_v3`
- `dobot_moveit`
- `dobot_msgs_v3`

The launch files also expect common Humble packages such as:

- `moveit_ros_move_group`
- `moveit_configs_utils`
- `robot_state_publisher`
- `rviz2`
- `camera_ros`
- `apriltag_ros`
- `tf2_ros`

## Suggested workspace setup

```bash
mkdir -p ~/dobot_ws/src
cd ~/dobot_ws/src

git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3.git
git clone <this-repo-url> Nova_ROS2
```

Build from the workspace root:

```bash
cd ~/dobot_ws
colcon build
source install/setup.bash
```

## Launch files

### Full bringup

```bash
ros2 launch robotiq_2f85_bringup nova5_full_bringup.launch.py
```

This launch file is intended to start:

- Dobot TCP bringup and feedback
- `robot_state_publisher`
- the `nova5_2f85_joint_states` bridge
- MoveIt `move_group`
- the Dobot MoveIt action bridge
- wrist camera input through `camera_ros`
- AprilTag detection through `apriltag_ros`
- static TF for camera mounting
- optional `map -> base_link` table transform
- RViz

Useful launch arguments:

- `use_rviz:=false` to run without RViz
- `use_table_map:=false` to disable the static `map -> base_link` transform
- `cam_x`, `cam_y`, `cam_z`, `cam_roll`, `cam_pitch`, `cam_yaw` to define the camera mounting transform
- `modbus_host`, `modbus_port`, `slave_id` for gripper communication settings

Example:

```bash
ros2 launch robotiq_2f85_bringup nova5_full_bringup.launch.py \
  use_rviz:=false \
  cam_x:=0.04 cam_y:=0.0 cam_z:=0.05 \
  cam_roll:=0.0 cam_pitch:=0.0 cam_yaw:=0.0
```

### MoveIt-only launch

```bash
ros2 launch nova5_2f85_moveit nova5_2f85_moveit.launch.py
```

This starts:

- `robot_state_publisher`
- `nova5_2f85_joint_states`
- `dobot_moveit/action_move_server`
- `move_group`
- optional RViz

## TF and perception notes

The full bringup connects the camera into the robot TF tree with:

```text
base_link -> ... -> robotiq_85_base_link -> camera_link
```

It also publishes a zero-offset bridge:

```text
camera_link -> camera
```

AprilTag detections from `apriltag_ros` can then be transformed into `map`, and `apriltag_arm_planner.py` republishes them as:

```text
map_tag_0, map_tag_1, ...
```

The default table setup also publishes:

```text
map -> base_link
```

with a fixed table height defined in `nova5_full_bringup.launch.py`.

## Configuration files

- `robotiq_2f85_bringup/config/camera_calibration.yaml`
  Camera intrinsics loaded by `camera_ros`
- `robotiq_2f85_bringup/config/tags_36h11.yaml`
  AprilTag family and configured tag IDs

## Current status

This README reflects the files currently present in this repository. If you change package names, add back removed runtime nodes, or split the full bringup into smaller launches, update this document to keep it aligned with the code.
