from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    abb_pkg_share = FindPackageShare("abb_pkg")
    irb1600_share = FindPackageShare("abb_irb1600_support")

    params_yaml = PathJoinSubstitution([abb_pkg_share, "config", "params.yaml"])
    rviz_config = PathJoinSubstitution([abb_pkg_share, "config", "abb.rviz"])

    start_egm = LaunchConfiguration("start_egm")
    start_rviz = LaunchConfiguration("start_rviz")

    # Build robot_description from xacro (IRB1600-10/1.45)
    xacro_file = PathJoinSubstitution([irb1600_share, "urdf", "irb1600_10_145.xacro"])
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str,
        )
    }

    egm_node = Node(
        package="abb_pkg",
        executable="egm_node",
        name="egm_node",
        output="screen",
        condition=IfCondition(start_egm),
        parameters=[
            params_yaml,
            robot_description,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

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
        condition=IfCondition(start_rviz),
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_egm",
                default_value="true",
                description="Start abb_pkg/egm_node (needs real EGM connection).",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz2 (requires GUI / X server).",
            ),
            egm_node,
            robot_state_publisher,
            static_world_to_base,
            rviz2,
        ]
    )
