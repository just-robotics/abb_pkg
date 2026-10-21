# Everything that talks to the robot: ros2_control_node with the hardware plugin,
# controllers and robot_state_publisher. In simulation the plugin is mock_components;
# for the real robot add the driver package below and set HARDWARE_PLUGIN.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -y && \
    apt install -y \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY workspace/src/abb src/abb
COPY workspace/src/abb_irb1600_description src/abb_irb1600_description
COPY workspace/src/abb_irb1600_bringup src/abb_irb1600_bringup
# Real robot driver (hardware_interface::SystemInterface plugin):
# COPY workspace/src/<driver_pkg> src/<driver_pkg>

RUN . /opt/ros/humble/setup.sh && colcon build --packages-up-to abb_irb1600_bringup

ENV HARDWARE_PLUGIN=mock_components/GenericSystem

CMD ["bash", "-c", ". install/setup.sh && exec ros2 launch abb_irb1600_bringup hardware.launch.xml hardware_plugin:=${HARDWARE_PLUGIN}"]
