# MoveIt: move_group (planning, execution through joint_trajectory_controller) and RViz.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -y && \
    apt install -y \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY workspace/src/abb src/abb
COPY workspace/src/abb_irb1600_description src/abb_irb1600_description
COPY workspace/src/abb_irb1600_moveit_config src/abb_irb1600_moveit_config

RUN . /opt/ros/humble/setup.sh && colcon build --packages-up-to abb_irb1600_moveit_config

ENV RVIZ=true

CMD ["bash", "-c", ". install/setup.sh && exec ros2 launch abb_irb1600_moveit_config move_group.launch.xml rviz:=${RVIZ}"]
