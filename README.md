# abb_pkg

**Maintainers:** Ksenia Vasilieva, Artem Kondratiev, Roman Eidelman

ROS 2 workspace for controlling an **ABB** industrial robot via **Externally Guided Motion (EGM)**.

## Repository description

This repository contains:

- **abb_pkg** — Main EGM control node: communicates with the ABB controller over UDP, runs inverse kinematics (Trac-IK), publishes joint state and end-effector pose, and provides a `move_to_pose` action. Includes RViz visualization (robot model and markers).
- **abb_libegm** — C++ library for the EGM protocol (ABB option 689-1); used by `abb_pkg` to send/receive motion commands.
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

## Docker

The project provides a Docker image based on `osrf/ros:humble-desktop-full` with the workspace pre-built. Use it from the **repository root**.

### Build the image

From the repository root:

```bash
cd docker
./build.sh
```

Or:

```bash
cd docker
docker compose build
```

### Run the container

Start the container in the background:

```bash
cd docker
./run.sh
```

Or:

```bash
cd docker
docker compose up -d
```

The container uses `network_mode: host` so the robot and other hosts are reachable. For RViz/GUI, ensure X11 forwarding (e.g. `-e DISPLAY`, `/tmp/.X11-unix` volume) is available; the compose file sets these by default.

### Enter the container

Open a shell inside the running container:

```bash
cd docker
./exec.sh
```

Or:

```bash
cd docker
docker compose exec abb bash
```

Inside the container, the workspace is at `/workspace` and `install/setup.bash` is already sourced in `~/.bashrc`. Run launch files as usual, e.g.:

```bash
ros2 launch abb_pkg egm_rviz.launch.py
```

### Stop the container

```bash
cd docker
./stop.sh
```

Or:

```bash
cd docker
docker compose down
```

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

For the IRB 1600 10/1.45 model specifically:

```bash
ros2 launch abb_pkg irb1600_10_145_egm_rviz.launch.py
```

EGM only (no RViz):

```bash
ros2 launch abb_pkg egm.launch.py
```

Ensure the ABB controller is in EGM mode and reachable on the configured UDP port (default 6515).

### 2. Send a pose goal

Send Cartesian goals to the robot using the `move_to_pose` action. Use an action client (e.g. `ros2 action send_goal`) or any node that calls the `abb_pkg/action/MoveToPose` action. Goal fields: `position` (Point) and `orientation` (Quaternion).

### 3. Gripper

Run the gripper node (forwards `gripper/command` to the gripper over UDP):

```bash
ros2 run abb_grip grip_node
```

Configure `gripper_ip` and `gripper_port` via parameters if needed (defaults in the node).

### 4. Camera (optional)

If using the Pylon camera wrapper:

```bash
ros2 launch camera_wrapper camera_node.launch.py
```

Requires the camera driver and `camera_wrapper` config (e.g. `config/params.yaml`) to be set up.

### 5. MoveIt 2 + ros2_control stack (simulation, no RAPID)

Three containers, see `docker/moveit_stack/`:

| Container    | Package                      | Runs                                                                                   |
|--------------|------------------------------|----------------------------------------------------------------------------------------|
| `hardware`   | `abb_irb1600_bringup`        | `robot_state_publisher`, `ros2_control_node` (hardware plugin), `joint_state_broadcaster`, `joint_trajectory_controller` |
| `moveit`     | `abb_irb1600_moveit_config`  | `move_group` (OMPL, KDL), RViz with the MotionPlanning panel                           |
| `trajectory` | `abb_trajectory_generator`   | `work_cell` (tool change, milling / laser work) and the circle / line generator, both sent to `move_group` |

```bash
cd docker/moveit_stack
docker compose up --build                      # RViz needs X11; RVIZ=false for headless
AUTOSTART_FIGURE=circle docker compose up      # run a figure right after startup
```

**Work cell.** Three tables are added to the MoveIt planning scene: a tool table with a mill and a
laser in holders, a milling table with a workpiece and a laser table with a sheet. The **WorkCell**
panel in RViz has the buttons:

| Button              | Service (`std_srvs/Trigger`) | What happens                                                    |
|---------------------|------------------------------|-----------------------------------------------------------------|
| Взять фрезу / лазер | `/work_cell/take_mill`, `/work_cell/take_laser` | puts the current tool back if needed, takes the new one |
| Снять инструмент    | `/work_cell/release_tool`    | returns the tool to its holder                                  |
| Начать работу       | `/work_cell/start_work`      | mill: raster over the workpiece, laser: circle and star on the sheet; repeats until stopped |
| Прекратить работу   | `/work_cell/stop_work`       | stops the motion and lifts the tool                             |

A held tool is attached to `tool0` in the planning scene, so MoveIt plans with it. The status is in
`/work_cell/status`, visuals (cell, tools, machining trace, laser beam) in `/work_cell/markers`.
Cell geometry is in `abb_trajectory_generator/cell_layout.py`, speeds in `config/params.yaml`.

Run a figure on a running stack:

```bash
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: circle}"   # or line
```

The generator moves to the first waypoint (`/move_action`), interpolates the path
(`/compute_cartesian_path`) and executes it (`/execute_trajectory`). Figure geometry, tool
orientation and speed scaling are in `abb_trajectory_generator/config/params.yaml`.

Without Docker: `ros2 launch abb_irb1600_bringup hardware.launch.xml`,
`ros2 launch abb_irb1600_moveit_config move_group.launch.xml`,
`ros2 launch abb_trajectory_generator trajectory_generator.launch.xml`.
The robot model alone: `ros2 launch abb_irb1600_description view_robot.launch.xml`.

**Switching to the real robot.** Only the `hardware` container changes. The robot driver must be a
`hardware_interface::SystemInterface` plugin with joints `joint_1` … `joint_6` (rad), command
interface `position` and state interface `position`:

1. add the driver package to `docker/moveit_stack/hardware.Dockerfile` (see the comment there);
2. put driver-specific `<param>` entries into the `<hardware>` block of
   `abb_irb1600_description/urdf/irb1600.ros2_control.xacro`;
3. start with `HARDWARE_PLUGIN=<driver_pkg>/<DriverSystem> docker compose up`.

Before the first run on hardware check `update_rate` and trajectory tolerances in
`abb_irb1600_bringup/config/ros2_controllers.yaml` and the acceleration limits in
`abb_irb1600_moveit_config/config/joint_limits.yaml` (velocity/acceleration scaling defaults to 0.1).

## Typical workflow

1. Build and source the workspace (see **Build**).
2. Start EGM and RViz: `ros2 launch abb_pkg egm_rviz.launch.py`.
3. Send goals to the robot via the `move_to_pose` action from your application or `ros2 action send_goal`.

## Topics and actions (main)

| Topic / Action        | Type                 | Description                          |
|-----------------------|----------------------|--------------------------------------|
| `joint_states`        | JointState           | Current joint positions (from EGM).  |
| `ee_pose`            | PoseStamped          | Current end-effector pose.           |
| `move_to_pose`       | Action (MoveToPose)  | Cartesian pose goal for the robot.   |
| `gripper/command`     | String               | Gripper commands (UDP forward).      |

Config and URDF for the ABB model (e.g. IRB 1600) live under `workspace/src/abb_pkg/config` and `workspace/src/abb_pkg/urdf`.
