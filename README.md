# abb_pkg

**Maintainers:** Ksenia Vasilieva, Artem Kondratiev, Roman Eidelman

ROS 2 (Humble) workspace for an **ABB IRB 1600-10/1.45** industrial robot. It has two independent
ways to drive the robot:

| Stack | What it is | Where |
|-------|------------|-------|
| **MoveIt 2 + ros2_control** | Simulation of a mill / laser work cell: MoveIt plans, `joint_trajectory_controller` executes on a mock (or real) hardware plugin. No RAPID needed. | [MoveIt 2 + ros2_control stack](#moveit-2--ros2_control-stack-simulation) |
| **EGM** | Direct control of a real controller over UDP via **Externally Guided Motion** (RAPID program on the robot side). | [EGM stack](#egm-stack-real-robot) |

Notes for coding agents and a detailed description of the pipeline: [AGENTS.md](AGENTS.md).

## Repository layout

```
abb_pkg/
├── AGENTS.md                      # architecture and pipeline for developers / agents
├── RAPID/                         # RAPID programs for the EGM stack
├── docker/
│   ├── Dockerfile, *.sh           # EGM container (whole workspace)
│   └── moveit_stack/              # MoveIt simulation: three containers
└── workspace/src/
    ├── abb/                       # ros-industrial ABB support: IRB 1600 meshes and URDF macro
    ├── abb_irb1600_description/   # URDF + ros2_control xacro, start pose
    ├── abb_irb1600_bringup/       # ros2_control_node, controllers, robot_state_publisher
    ├── abb_irb1600_moveit_config/ # SRDF, kinematics, OMPL, limits, move_group + RViz launch
    ├── abb_trajectory_generator/  # work_cell node, figure generator, MoveIt client (Python)
    ├── abb_work_cell_panel/       # RViz panel with the work cell buttons (C++)
    ├── abb_pkg/, abb_libegm/, trac_ik/   # EGM stack
    └── ...                        # gripper, camera, teleop, RRT, LeRobot tools (see below)
```

| Package | Stack | Description |
|---------|-------|-------------|
| `abb_irb1600_description` | MoveIt | URDF of the IRB 1600-10/1.45 with the `ros2_control` block; the hardware plugin is a xacro argument |
| `abb_irb1600_bringup` | MoveIt | `hardware.launch.xml`: `robot_state_publisher`, `ros2_control_node`, `joint_state_broadcaster`, `joint_trajectory_controller` |
| `abb_irb1600_moveit_config` | MoveIt | `move_group.launch.xml`: `move_group` (OMPL, KDL) and RViz with the MotionPlanning and WorkCell panels |
| `abb_trajectory_generator` | MoveIt | `work_cell` (tool change, milling, laser cutting) and `trajectory_generator` (circle / line), both talk to `move_group` over ROS interfaces only |
| `abb_work_cell_panel` | MoveIt | RViz 2 panel plugin: buttons calling the `/work_cell/*` services, status line |
| `abb_pkg` | EGM | EGM control node: UDP to the controller, Trac-IK, `joint_states`, `ee_pose`, `move_to_pose` action, RViz |
| `abb_libegm` | EGM | C++ library for the EGM protocol (ABB option 689-1) |
| `trac_ik` | EGM | Inverse kinematics library (drop-in replacement for KDL's IK) |
| `vel_control` | EGM | Velocity control over EGM (`cmd_vel` → robot), see its `AGENTS.md` |
| `abb_grip` | EGM | Forwards `gripper/command` strings to the gripper over UDP |
| `camera_wrapper` | — | Pylon (Basler) camera wrapper, optional |
| `abb_teleop`, `abb_lerobot` | EGM | Gamepad teleoperation, LeRobot dataset recorder |
| `rrt_3d_planner`, `rrt_path_executor`, `rrt_rviz_panel`, `point_cloud_emulator` | EGM | 3D RRT planning on a point cloud and its execution |

## MoveIt 2 + ros2_control stack (simulation)

### Architecture

Three containers on the host network (CycloneDDS, `ROS_DOMAIN_ID` from the environment, default 0):

```
┌──────────────── trajectory ────────────────┐   ┌─────────────── moveit ───────────────┐   ┌────────────── hardware ──────────────┐
│ work_cell            trajectory_generator  │   │ move_group                           │   │ ros2_control_node                    │
│  /work_cell/* srv     /figure_type topic   │──▶│  /move_action  /execute_trajectory   │──▶│  joint_trajectory_controller         │
│  /work_cell/status    autostart_figure     │   │  /compute_cartesian_path /compute_ik │   │   (FollowJointTrajectory action)     │
│  /work_cell/markers                        │   │  /apply_planning_scene               │   │  joint_state_broadcaster → /joint_states
└────────────────────────────────────────────┘   │ RViz: MotionPlanning + WorkCell panel│   │  hardware plugin: mock_components    │
                                                 └──────────────────────────────────────┘   │ robot_state_publisher → /tf          │
                                                                                            └──────────────────────────────────────┘
```

| Container    | Image            | Package                      | Runs |
|--------------|------------------|------------------------------|------|
| `hardware`   | `abb_hardware`   | `abb_irb1600_bringup`        | `robot_state_publisher`, `ros2_control_node` (hardware plugin), `joint_state_broadcaster`, `joint_trajectory_controller` |
| `moveit`     | `abb_moveit`     | `abb_irb1600_moveit_config`  | `move_group` (OMPL, KDL), RViz with the MotionPlanning and WorkCell panels |
| `trajectory` | `abb_trajectory` | `abb_trajectory_generator`   | `work_cell` and `trajectory_generator` |

Only the `hardware` container knows whether the robot is simulated or real: MoveIt and the work
cell see the same controllers and joints in both cases.

### Run with Docker

From the repository root:

```bash
xhost +local:docker                 # let RViz in the container open a window on the host display
cd docker/moveit_stack
docker compose up --build           # builds the three images on the first run, then shows all logs
```

When the stack is up, the logs contain:

- `hardware`: `Configured and activated joint_state_broadcaster` / `joint_trajectory_controller`;
- `moveit`: `You can start planning now!`;
- `trajectory`: `Ready, publish "circle" or "line" to figure_type` and `Work cell added to the planning scene`.

After that the nodes are silent until a command arrives (a panel button, a service call, a figure).

Options (environment variables of `docker compose`):

| Variable           | Default                          | Meaning |
|--------------------|----------------------------------|---------|
| `RVIZ`             | `true`                           | `false` runs `move_group` headless |
| `AUTOSTART_FIGURE` | empty                            | `circle` or `line`: run this figure right after start |
| `HARDWARE_PLUGIN`  | `mock_components/GenericSystem`  | ros2_control plugin of the `hardware` container |
| `ROS_DOMAIN_ID`    | `0`                              | DDS domain of all three containers |

```bash
RVIZ=false docker compose up                  # headless
AUTOSTART_FIGURE=circle docker compose up     # draw a circle after startup
docker compose up -d                          # in the background; logs are not printed then:
docker compose logs -f                        #   all containers
docker compose logs -f trajectory             #   only work_cell / trajectory_generator
docker compose exec trajectory bash           # shell in a container (then: . install/setup.sh)
docker compose down                           # stop and remove the containers
```

The sources are **copied** into the images at build time (no volume), so after changing code run
`docker compose up --build` (or `docker compose build <service>`).

### Run without Docker

Dependencies (Humble):

```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-rmw-cyclonedds-cpp
```

Build:

```bash
cd workspace
colcon build --symlink-install --packages-up-to \
  abb_irb1600_bringup abb_irb1600_moveit_config abb_trajectory_generator
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # same RMW as the containers
```

Run in three terminals (source the workspace in each one), in this order:

```bash
ros2 launch abb_irb1600_bringup hardware.launch.xml                   # [hardware_plugin:=<pkg>/<System>]
ros2 launch abb_irb1600_moveit_config move_group.launch.xml           # [rviz:=false]
ros2 launch abb_trajectory_generator trajectory_generator.launch.xml  # [autostart_figure:=circle]
```

The robot model alone, with joint sliders: `ros2 launch abb_irb1600_description view_robot.launch.xml`.

### Work cell

`work_cell` adds three tables to the MoveIt planning scene:

- **tool table** (`y < 0`): a mill and a laser standing in holders;
- **milling table** (`x > 0`): a workpiece;
- **laser table** (`y > 0`): a sheet.

The **WorkCell** panel in RViz has the buttons:

| Button              | Service (`std_srvs/Trigger`) | What happens |
|---------------------|------------------------------|--------------|
| Взять фрезу / лазер | `/work_cell/take_mill`, `/work_cell/take_laser` | puts the current tool back if needed, takes the new one |
| Снять инструмент    | `/work_cell/release_tool`    | returns the tool to its holder |
| Начать работу       | `/work_cell/start_work`      | mill: raster over the workpiece, laser: circle and star on the sheet; runs once, then the robot goes home |
| Прекратить работу   | `/work_cell/stop_work`       | stops the motion, lifts the tool clear of the part and stays there |
| Домой               | `/work_cell/go_home`         | moves the robot to the `home` pose of the SRDF |

The same from a shell:

```bash
ros2 service call /work_cell/take_mill std_srvs/srv/Trigger
ros2 service call /work_cell/start_work std_srvs/srv/Trigger
ros2 topic echo /work_cell/status
```

A service answers at once (`принято`, or the reason it is refused, e.g. `занят: ...`) and the job runs
in the background; one job at a time. The progress is in `/work_cell/status` (latched) and in the
panel. Visuals (cell, tools, machining trace, laser beam) are in `/work_cell/markers`.

Home is `[0, 0, 0, 0, 90, -90]` deg, the pose the simulated robot starts in
(`abb_irb1600_description/config/initial_positions.yaml` and `home` in the SRDF; keep them equal).
A held tool is attached to `tool0` in the planning scene, so MoveIt plans with it.

### Figures

```bash
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: circle}"   # or line
```

`trajectory_generator` moves to the first waypoint (`/move_action`), interpolates the path
(`/compute_cartesian_path`) and executes it (`/execute_trajectory`). Figures are planned with
collision checking against the cell and a held tool, so with a tool in hand a figure that passes
too low over a table is refused (`only N% of the path is feasible`). Figure geometry, tool
orientation and speed scaling are in `abb_trajectory_generator/config/params.yaml`.

### Planning by hand in RViz

Turn on **Displays → MotionPlanning → Planning Request → Query Goal State**: the goal appears as a
translucent green robot with a marker to drag, then use **Plan & Execute**. It is off by default.

### Configuration

| What | File |
|------|------|
| Start pose of the simulated robot | `abb_irb1600_description/config/initial_positions.yaml` |
| ros2_control joints / interfaces, hardware plugin parameters | `abb_irb1600_description/urdf/irb1600.ros2_control.xacro` |
| Controllers, `update_rate`, trajectory tolerances | `abb_irb1600_bringup/config/ros2_controllers.yaml` |
| Planning group, `home` pose, disabled collisions | `abb_irb1600_moveit_config/config/irb1600_10_145.srdf` |
| Velocity / acceleration limits and default scaling (0.1) | `abb_irb1600_moveit_config/config/joint_limits.yaml` |
| Planner (OMPL RRTConnect), IK (KDL), MoveIt → controller mapping | `abb_irb1600_moveit_config/config/{planning_pipelines,kinematics,moveit_controllers}.yaml` |
| RViz layout | `abb_irb1600_moveit_config/config/moveit.rviz` |
| Cell geometry, tools, milling / laser programs geometry | `abb_trajectory_generator/abb_trajectory_generator/cell_layout.py` |
| Work cell and figure speeds, figure geometry | `abb_trajectory_generator/config/params.yaml` |

### Switching to the real robot

Only the `hardware` container changes. The robot driver must be a
`hardware_interface::SystemInterface` plugin with joints `joint_1` … `joint_6` (rad), command
interface `position` and state interface `position`:

1. add the driver package to `docker/moveit_stack/hardware.Dockerfile` (see the comment there);
2. put driver-specific `<param>` entries into the `<hardware>` block of
   `abb_irb1600_description/urdf/irb1600.ros2_control.xacro`;
3. start with `HARDWARE_PLUGIN=<driver_pkg>/<DriverSystem> docker compose up`.

Before the first run on hardware check `update_rate` and trajectory tolerances in
`abb_irb1600_bringup/config/ros2_controllers.yaml`, the acceleration limits in
`abb_irb1600_moveit_config/config/joint_limits.yaml` (velocity/acceleration scaling defaults to 0.1)
and the speeds in `abb_trajectory_generator/config/params.yaml`.

### Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| Nothing is printed after `docker compose up -d` | `-d` detaches; use `docker compose logs -f` |
| No new log lines after startup | Expected: the nodes log only on commands |
| RViz does not open, `could not connect to display` | Run `xhost +local:docker`; `DISPLAY` must be set in the shell running `docker compose` |
| `libGL error: failed to load driver: nouveau` | Harmless, RViz falls back to another GL driver |
| Panel: `Нет связи с work_cell` / `Сервис недоступен` | `work_cell` is not running or in another DDS domain / RMW; check `docker compose logs trajectory` |
| Status `ошибка: move_group недоступен` | `move_group` did not come up within 30 s of `work_cell` start; restart it: `docker compose restart trajectory` |
| Status `занят: ...` | A job is running; wait or press **Прекратить работу** |
| Nodes outside Docker do not see the containers | Use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and the same `ROS_DOMAIN_ID` |

## EGM stack (real robot)

### Dependencies

- **ROS 2** Humble
- **NLopt C++** (for `trac_ik_lib`): `sudo apt-get install -y libnlopt-cxx-dev`
- ROS 2 packages: `ros-humble-kdl-parser`, `ros-humble-urdf`, `ros-humble-robot-state-publisher`,
  `ros-humble-tf2-ros`, `ros-humble-rviz2`, `ros-humble-visualization-msgs`,
  `ros-humble-geometry-msgs`, `ros-humble-sensor-msgs`, `ros-humble-std-msgs`,
  `ros-humble-action-msgs`, `ros-humble-launch`, `ros-humble-launch-ros`, and the build tools
  (`build-essential`, `cmake`, `python3-colcon-common-extensions`).
- `abb_libegm`: Boost, Protobuf (`protobuf-dev`, `protobuf`).
- `camera_wrapper`: Pylon SDK if a Basler camera is used.

### Build

```bash
cd workspace
sudo apt-get install -y libnlopt-cxx-dev
colcon build --symlink-install
source install/setup.bash
```

If `nlopt.hpp` is missing, CMake reports it: install `libnlopt-cxx-dev` and rebuild.

### Docker

An image based on `osrf/ros:humble-desktop-full` with the whole workspace built; `workspace/src` is
mounted into the container. From the repository root:

```bash
cd docker
./build.sh        # or: docker compose build
./run.sh          # or: docker compose up -d   (host network, X11 forwarding)
./exec.sh         # or: docker compose exec abb bash
./stop.sh         # or: docker compose down
```

Inside the container the workspace is at `/workspace` and `install/setup.bash` is sourced in
`~/.bashrc`.

### Run

All commands are run from `workspace` after `source install/setup.bash`.

1. **EGM control and RViz:**

   ```bash
   ros2 launch abb_pkg egm_rviz.launch.py                 # EGM node, robot_state_publisher, RViz
   ros2 launch abb_pkg irb1600_10_145_egm_rviz.launch.py  # the IRB 1600 10/1.45 model
   ros2 launch abb_pkg egm.launch.py                      # EGM only, no RViz
   ```

   The ABB controller must be in EGM mode and reachable on the configured UDP port (default 6515).

2. **Pose goal:** the `move_to_pose` action (`abb_pkg/action/MoveToPose`, fields `position` and
   `orientation`), e.g. via `ros2 action send_goal`.

3. **Gripper:** `ros2 run abb_grip grip_node` (parameters `gripper_ip`, `gripper_port`).

4. **Camera (optional):** `ros2 launch camera_wrapper camera_node.launch.py` (needs the camera
   driver and `camera_wrapper/config/params.yaml`).

### Topics and actions

| Topic / Action    | Type                | Description                         |
|-------------------|---------------------|-------------------------------------|
| `joint_states`    | JointState          | Current joint positions (from EGM). |
| `ee_pose`         | PoseStamped         | Current end-effector pose.          |
| `move_to_pose`    | Action (MoveToPose) | Cartesian pose goal for the robot.  |
| `gripper/command` | String              | Gripper commands (UDP forward).     |

Config and URDF of the EGM stack live in `workspace/src/abb_pkg/config` and
`workspace/src/abb_pkg/urdf`.
