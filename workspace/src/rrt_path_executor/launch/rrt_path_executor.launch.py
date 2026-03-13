from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("rrt_path_executor")
    config_path = PathJoinSubstitution([pkg_share, "config", "params.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=config_path,
                description="Path to rrt_path_executor params file",
            ),
            Node(
                package="rrt_path_executor",
                executable="path_executor_node",
                name="rrt_path_executor_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
