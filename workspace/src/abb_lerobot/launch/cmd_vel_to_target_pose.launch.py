import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get path to config file
    config_file = os.path.join(
        get_package_share_directory("abb_lerobot"),
        "config",
        "cmd_vel_to_target_pose.yaml"
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=config_file,
            description="Path to cmd_vel_to_target_pose config YAML file"
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Topic for cmd_vel messages (overrides config)"
        ),
        DeclareLaunchArgument(
            "target_pose_topic",
            default_value="/target_pose",
            description="Topic to publish target poses (overrides config)"
        ),
        
        # CmdVel to TargetPose node
        Node(
            package="abb_lerobot",
            executable="cmd_vel_to_target_pose",
            name="cmd_vel_to_target_pose",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                }
            ]
        ),
    ])
