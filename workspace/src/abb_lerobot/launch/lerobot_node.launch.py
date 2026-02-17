from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Bridge /target_pose (PoseStamped) -> /move_to_pose action
    lerobot_bridge = Node(
        package="abb_lerobot",
        executable="lerobot_bridge",
        name="lerobot_bridge",
        output="screen",
        parameters=[
            {"target_pose_topic": "/target_pose"},
            {"action_name": "/move_to_pose"},
            {"preempt": True},
        ],
    )

    return LaunchDescription([lerobot_bridge])

