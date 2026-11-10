# Trajectory generation: figures and the mill / laser work cell, sent to move_group.
# Uses only ROS interfaces of MoveIt, no MoveIt libraries.
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
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY workspace/src/abb_trajectory_generator src/abb_trajectory_generator

# BOXes interfaces (manipulator_manual.md)
COPY workspace/src/boxes_interfaces src/boxes_interfaces

RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select abb_trajectory_generator \
    move_to_pose_interface manipulator_msgs box_msgs

ENV AUTOSTART_FIGURE=""

CMD ["bash", "-c", ". install/setup.sh && exec ros2 launch abb_trajectory_generator trajectory_generator.launch.xml ${AUTOSTART_FIGURE:+autostart_figure:=$AUTOSTART_FIGURE}"]
