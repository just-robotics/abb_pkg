# Everything that talks to the robot: ros2_control_node with the hardware plugin,
# controllers and robot_state_publisher. In simulation the plugin is mock_components;
# for the real robot add the driver package below and set HARDWARE_PLUGIN.
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -y && \
    apt install -y \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    # standard interfaces of the BOXes API (manipulator_manual.md, section 9)
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-action-msgs \
    ros-jazzy-tf2-msgs \
    ros-jazzy-moveit-msgs \
    ros-jazzy-control-msgs \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY workspace/src/abb src/abb
COPY workspace/src/abb_irb1600_description src/abb_irb1600_description
COPY workspace/src/abb_irb1600_bringup src/abb_irb1600_bringup
# Real robot driver (hardware_interface::SystemInterface plugin):
# COPY workspace/src/<driver_pkg> src/<driver_pkg>

# BOXes interfaces (manipulator_manual.md)
COPY workspace/src/boxes_interfaces src/boxes_interfaces

RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-up-to abb_irb1600_bringup \
    move_to_pose_interface manipulator_msgs box_msgs

ENV HARDWARE_PLUGIN=mock_components/GenericSystem

CMD ["bash", "-c", ". install/setup.sh && exec ros2 launch abb_irb1600_bringup hardware.launch.xml hardware_plugin:=${HARDWARE_PLUGIN}"]
