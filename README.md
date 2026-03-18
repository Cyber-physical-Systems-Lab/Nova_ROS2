# Nova_ROS2

ROS 2 Humble packages for a Dobot Nova5 with a Robotiq 2F-85 gripper, MoveIt, and a wrist-mounted camera / AprilTag workflow.

This repository is the custom layer that sits on top of the upstream Dobot ROS 2 driver. It contains:

- A combined Nova5 + 2F-85 robot description
- A MoveIt configuration for the combined robot
- Bringup code that bridges Dobot joint feedback into `/joint_states`
- A full-system launch file that wires MoveIt, camera, TF, and AprilTag detection together
- An AprilTag relay node that republishes detected tags into the `map` frame
- A web-commanded grasp task planner for human-in-the-loop grasp experiments
- A planning-scene table object so MoveIt avoids the experiment table during planning

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
        ├── grasp_task_planner.py
        ├── nova5_2f85_joint_states.py
        ├── planning_scene_table.py
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
- `gripper_modbus_manager.py`
  Single supported Robotiq Modbus owner. It publishes `/gripper/joint_states` and `/web_feedback`, serves the internal gripper action, and can reuse a configured `modbus_index` or create a new gripper channel when needed.
- `action_server.py`
  Public gripper action proxy. It forwards user-facing gripper goals to `gripper_modbus_manager` and does not open its own Modbus channel.
- `nova5_2f85_joint_states.py`
  Bridges `/joint_states_robot` from the Dobot driver into a combined `/joint_states` topic that also includes gripper mimic joints.
- `apriltag_arm_planner.py`
  Subscribes to `/detections`, resolves tag frames through TF, and republishes `map_tag_<id>` frames in the `map` frame.
- `grasp_task_planner.py`
  Subscribes to `/web_command`, accepts `start_grasp` and `stop_grasp`, reads `/grasp_object_id`, moves to fixed grasp poses in `base_link`, lifts with a Cartesian motion, and then moves to a fixed handover pose.
- `planning_scene_table.py`
  Applies a conservative box-shaped table collision object to MoveIt's planning scene. The default full bringup places a `0.7 x 0.7 x 0.8 m` table at `x=0.0 m`, `y=0.2 m` in `base_link`, with the robot base assumed to sit `0.77 m` above the floor.

`gripper_feedback_publisher.py` remains in the repository as a legacy standalone poller, but it is not part of the current bringup and is not the supported runtime path.

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
- `gripper_modbus_manager` as the single supported gripper Modbus owner
- `robotiq_2f85_action_server` as the public gripper proxy
- `gripper_cmd_sender` as the `/web_command` gripper router
- `grasp_task_planner` as the `/web_command` arm-task router
- `planning_scene_table` as the MoveIt collision-table publisher
- `/web_feedback` as string gripper readback for the web server (`0=closed`, `100=open`)
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
- `use_planning_table:=false` to disable the MoveIt table collision object
- `cam_x`, `cam_y`, `cam_z`, `cam_roll`, `cam_pitch`, `cam_yaw` to define the camera mounting transform
- `modbus_host`, `modbus_port`, `slave_id` for gripper communication settings
- `modbus_index:=N` to reuse a known Dobot Modbus channel directly
- `/web_command` accepts `gripper_pos:<percent>`, `start_grasp`, `stop_grasp`, and legacy bare gripper percentages like `83`
- `/grasp_object_id` selects object IDs `1..5` for the grasp task planner
- `/web_feedback` publishes measured gripper state with `0=closed`, `100=open`

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

For human-in-the-loop grasp experiments, `grasp_task_planner.py` uses the same
`/web_command` topic for arm-task commands. `start_grasp` moves the arm to the
configured grasp pose for the currently selected `/grasp_object_id`. After
manual gripper adjustment via `/web_command`, `stop_grasp` lifts the tool
straight up with a Cartesian path and then moves to a fixed handover pose.

## Configuration files

- `robotiq_2f85_bringup/config/camera_calibration.yaml`
  Camera intrinsics loaded by `camera_ros`
- `robotiq_2f85_bringup/config/tags_36h11.yaml`
  AprilTag family and configured tag IDs, including `tag_11`

## Current status

This README reflects the files currently present in this repository. If you change package names, add back removed runtime nodes, or split the full bringup into smaller launches, update this document to keep it aligned with the code.
