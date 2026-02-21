from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_path = get_package_share_directory("abb_pkg")

    params = os.path.join(pkg_path, "config", "params.yaml")
    urdf = os.path.join(pkg_path, "urdf", "irb1600.urdf")
    rviz_config = os.path.join(pkg_path, "config", "abb.rviz")

    with open(urdf, "r", encoding="utf-8") as f:
        robot_description = f.read()

    egm_node = Node(
        package="abb_pkg",
        executable="egm_node",
        name="egm_node",
        output="screen",
        parameters=[
            params,
            {"robot_description": robot_description},
        ],
    )

    # Publishes TF from /joint_states using robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Ensure RViz fixed frame has a valid transform chain
    static_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            egm_node,
            robot_state_publisher,
            static_world_to_base,
            rviz2,
        ]
    )

