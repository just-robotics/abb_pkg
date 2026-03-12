from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = FindPackageShare("point_cloud_emulator")
    config_path = PathJoinSubstitution([pkg_share, "config", "params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=config_path,
            description="Path to the point_cloud_emulator config YAML",
        ),
        Node(
            package="point_cloud_emulator",
            executable="point_cloud_emulator_node",
            name="point_cloud_emulator",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
