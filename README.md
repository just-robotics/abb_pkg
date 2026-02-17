# abb_pkg

ROS 2 workspace for controlling an **ABB** industrial robot via **Externally Guided Motion (EGM)** and for integration with **LeRobot** (robot learning and dataset recording).

## Repository description

This repository contains:

- **abb_pkg** — Main EGM control node: communicates with the ABB controller over UDP, runs inverse kinematics (Trac-IK), publishes joint state and end-effector pose, and provides a `move_to_pose` action. Includes RViz visualization (robot model and markers).
- **abb_libegm** — C++ library for the EGM protocol (ABB option 689-1); used by `abb_pkg` to send/receive motion commands.
- **abb_lerobot** — LeRobot adapter and utilities: bridge from pose topics to the `move_to_pose` action, dataset recorder (JSONL), and a CLI to send pose goals.
- **abb_grip** — Gripper node: subscribes to `gripper/command` and forwards string commands to the gripper over UDP.
- **camera_wrapper** — Wrapper for a Pylon-based camera (e.g. Basler); optional for vision.
- **trac_ik** / **trac_ik_lib** — Inverse kinematics library (drop-in replacement for KDL’s IK); used by `abb_pkg` to convert Cartesian targets to joint positions.

All launch and run instructions below assume you are in the **workspace** directory and have run `source install/setup.bash` after building.

## Dependencies

### System (Debian/Ubuntu)

- **ROS 2** (Humble or compatible distro)
- **NLopt C++** (for `trac_ik_lib`):

  ```bash
  sudo apt-get update
  sudo apt-get install -y libnlopt-cxx-dev
  ```

- **Other ROS 2 packages** (install if not present): `ros-humble-kdl-parser`, `ros-humble-urdf`, `ros-humble-robot-state-publisher`, `ros-humble-tf2-ros`, `ros-humble-rviz2`, `ros-humble-visualization-msgs`, `ros-humble-geometry-msgs`, `ros-humble-sensor-msgs`, `ros-humble-std-msgs`, `ros-humble-action-msgs`, `ros-humble-launch`, `ros-humble-launch-ros`, and standard build tools (`build-essential`, `cmake`, `python3-colcon-common-extensions`).

### Workspace packages

- **abb_libegm**: Boost, Protobuf (`protobuf-dev`, `protobuf`).
- **abb_pkg**: `abb_libegm`, `trac_ik_lib`, `kdl_parser`, `tf2`, `tf2_ros`, `visualization_msgs`, `geometry_msgs`, `sensor_msgs`, `action_msgs`, `launch`, `launch_ros`; custom action interface `abb_pkg/action/MoveToPose`.
- **abb_lerobot**: Python 3, `rclpy`, `abb_pkg` (for the action); optional LeRobot Python package for the full robot API.
- **abb_grip**: `rclcpp`, `std_msgs`.
- **camera_wrapper**: Python; Pylon SDK if using a Basler camera.
- **trac_ik_lib**: `libnlopt-cxx-dev`, `eigen`, `kdl_parser`, `urdf`, `rclcpp`.

## Build

From the repository root:

```bash
cd workspace
sudo apt-get install -y libnlopt-cxx-dev
colcon build --symlink-install
source install/setup.bash
```

If `nlopt.hpp` is missing, CMake will report: install `libnlopt-cxx-dev` and rebuild.

## Run instructions

All commands are run from the **workspace** directory (`abb_pkg/workspace`) after:

```bash
source install/setup.bash
```

### 1. EGM control and RViz

Start the EGM node, robot state publisher, fixed frame, and RViz:

```bash
ros2 launch abb_pkg egm_rviz.launch.py
```

Ensure the ABB controller is in EGM mode and reachable on the configured UDP port (default 6515).

### 2. Send a pose goal (CLI)

Send a single Cartesian goal `[x, y, z]` in metres and orientation `[roll, pitch, yaw]` in degrees:

```bash
./src/abb_lerobot/send_pose.sh 1.0 -0.5 0.8 180 0 -90
```

Example: `x=1.0`, `y=-0.5`, `z=0.8`, roll=180°, pitch=0°, yaw=-90°. The script calls the `move_to_pose` action server provided by `abb_pkg`. The EGM + RViz launch must be running.

### 3. LeRobot bridge

Run the bridge that forwards `/target_pose` (PoseStamped) to the `move_to_pose` action:

```bash
ros2 launch abb_lerobot lerobot_node.launch.py
```

Use this when driving the robot from LeRobot or any node that publishes target poses.

### 4. Dataset recording

Record joint states, end-effector pose, target pose, and action result to a JSONL file:

```bash
ros2 launch abb_lerobot record_dataset.launch.py
```

Parameters (e.g. `output_dir`, `file_name`, `rate_hz`, topic names) can be overridden via the launch file or a config.

### 5. Gripper

Run the gripper node (forwards `gripper/command` to the gripper over UDP):

```bash
ros2 run abb_grip grip_node
```

Configure `gripper_ip` and `gripper_port` via parameters if needed (defaults in the node).

### 6. Camera (optional)

If using the Pylon camera wrapper:

```bash
ros2 launch camera_wrapper camera_node.launch.py
```

Requires the camera driver and `camera_wrapper` config (e.g. `config/params.yaml`) to be set up.

## Typical workflow

1. Build and source the workspace (see **Build**).
2. Start EGM and RViz: `ros2 launch abb_pkg egm_rviz.launch.py`.
3. Optionally start the LeRobot bridge: `ros2 launch abb_lerobot lerobot_node.launch.py`.
4. Send goals via `./src/abb_lerobot/send_pose.sh ...` or by publishing to `/target_pose` (with the bridge running).
5. To collect data, run `ros2 launch abb_lerobot record_dataset.launch.py` and perform episodes; check the configured `output_dir` for the JSONL dataset.

## Topics and actions (main)

| Topic / Action        | Type                 | Description                          |
|-----------------------|----------------------|--------------------------------------|
| `joint_states`        | JointState           | Current joint positions (from EGM).  |
| `ee_pose`            | PoseStamped          | Current end-effector pose.           |
| `move_to_pose`       | Action (MoveToPose)  | Cartesian pose goal for the robot.   |
| `target_pose`        | PoseStamped          | Target pose (consumed by bridge).    |
| `gripper/command`     | String               | Gripper commands (UDP forward).      |

Config and URDF for the ABB model (e.g. IRB 1600) live under `workspace/src/abb_pkg/config` and `workspace/src/abb_pkg/urdf`.
