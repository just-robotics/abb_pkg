# MoveIt: move_group (planning, execution through joint_trajectory_controller) and RViz
# with the work cell panel.
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
    ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY workspace/src/abb src/abb
COPY workspace/src/abb_irb1600_description src/abb_irb1600_description
COPY workspace/src/abb_irb1600_moveit_config src/abb_irb1600_moveit_config
COPY workspace/src/abb_work_cell_panel src/abb_work_cell_panel

# BOXes interfaces (manipulator_manual.md)
COPY workspace/src/boxes_interfaces src/boxes_interfaces

RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-up-to abb_irb1600_moveit_config \
    move_to_pose_interface manipulator_msgs box_msgs

ENV RVIZ=true

CMD ["bash", "-c", ". install/setup.sh && exec ros2 launch abb_irb1600_moveit_config move_group.launch.xml rviz:=${RVIZ}"]
