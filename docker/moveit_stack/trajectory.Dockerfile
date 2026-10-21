# Trajectory generation: figures and the mill / laser work cell, sent to move_group.
# Uses only ROS interfaces of MoveIt, no MoveIt libraries.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -y && \
    apt install -y \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-moveit-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY workspace/src/abb_trajectory_generator src/abb_trajectory_generator

RUN . /opt/ros/humble/setup.sh && colcon build --packages-select abb_trajectory_generator

ENV AUTOSTART_FIGURE=""

CMD ["bash", "-c", ". install/setup.sh && exec ros2 launch abb_trajectory_generator trajectory_generator.launch.xml ${AUTOSTART_FIGURE:+autostart_figure:=$AUTOSTART_FIGURE}"]
