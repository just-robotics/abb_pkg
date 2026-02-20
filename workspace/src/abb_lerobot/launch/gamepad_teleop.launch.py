import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get path to config file
    config_file = os.path.join(
        get_package_share_directory("abb_lerobot"),
        "config",
        "gamepad_teleop.yaml"
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=config_file,
            description="Path to gamepad teleop config YAML file"
        ),
        DeclareLaunchArgument(
            "joy_topic",
            default_value="/joy",
            description="Topic for joy messages (overrides config)"
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Topic to publish cmd_vel (overrides config)"
        ),
        DeclareLaunchArgument(
            "joy_device",
            default_value="",
            description="Joystick device path (e.g., /dev/input/js0). Empty string = auto-detect"
        ),
        
        # Joy node (auto-detects device if device_name is empty)
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "device_name": LaunchConfiguration("joy_device"),
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }]
        ),
        
        # Gamepad teleop node
        Node(
            package="abb_lerobot",
            executable="gamepad_teleop",
            name="gamepad_teleop",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "joy_topic": LaunchConfiguration("joy_topic"),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                }
            ]
        ),
    ])
