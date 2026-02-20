import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get paths to config files
    gamepad_config_file = os.path.join(
        get_package_share_directory("abb_lerobot"),
        "config",
        "gamepad_teleop.yaml"
    )
    cmd_vel_config_file = os.path.join(
        get_package_share_directory("abb_lerobot"),
        "config",
        "cmd_vel_to_target_pose.yaml"
    )
    
    return LaunchDescription([
        # ========== Gamepad Teleop Arguments ==========
        DeclareLaunchArgument(
            "gamepad_config_file",
            default_value=gamepad_config_file,
            description="Path to gamepad teleop config YAML file"
        ),
        DeclareLaunchArgument(
            "joy_topic",
            default_value="/joy",
            description="Topic for joy messages"
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Topic to publish cmd_vel"
        ),
        DeclareLaunchArgument(
            "joy_device",
            default_value="",
            description="Joystick device path (e.g., /dev/input/js0). Empty string = auto-detect"
        ),
        
        # ========== CmdVel to TargetPose Arguments ==========
        DeclareLaunchArgument(
            "cmd_vel_config_file",
            default_value=cmd_vel_config_file,
            description="Path to cmd_vel_to_target_pose config YAML file"
        ),
        DeclareLaunchArgument(
            "target_pose_topic",
            default_value="/target_pose",
            description="Topic to publish target poses"
        ),
        DeclareLaunchArgument(
            "ee_pose_topic",
            default_value="/ee_pose",
            description="Topic for robot end effector pose"
        ),
        
        # ========== LeRobot Bridge Arguments ==========
        DeclareLaunchArgument(
            "action_name",
            default_value="/move_to_pose",
            description="Action name for move_to_pose"
        ),
        DeclareLaunchArgument(
            "preempt",
            default_value="true",
            description="Preempt previous goals when new target_pose arrives"
        ),
        
        # ========== Nodes ==========
        
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
                LaunchConfiguration("gamepad_config_file"),
                {
                    "joy_topic": LaunchConfiguration("joy_topic"),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                }
            ]
        ),
        
        # CmdVel to TargetPose node
        Node(
            package="abb_lerobot",
            executable="cmd_vel_to_target_pose",
            name="cmd_vel_to_target_pose",
            output="screen",
            parameters=[
                LaunchConfiguration("cmd_vel_config_file"),
                {
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                    "ee_pose_topic": LaunchConfiguration("ee_pose_topic"),
                }
            ]
        ),
        
        # LeRobot bridge node
        Node(
            package="abb_lerobot",
            executable="lerobot_bridge",
            name="lerobot_bridge",
            output="screen",
            parameters=[
                {
                    "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                    "action_name": LaunchConfiguration("action_name"),
                    "preempt": LaunchConfiguration("preempt"),
                }
            ]
        ),
    ])
