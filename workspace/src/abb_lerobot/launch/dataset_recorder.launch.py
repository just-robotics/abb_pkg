from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="abb_lerobot",
                executable="dataset_recorder",
                name="dataset_recorder",
                output="screen",
                parameters=[
                    {
                        "dataset_root": "dataset",
                        "image_topic": "/image_raw",
                        "joint_states_topic": "/joint_states",
                        "ee_pose_topic": "/ee_pose",
                        "target_pose_topic": "/target_pose",
                        "gripper_topic": "/gripper/command",
                    }
                ],
            ),
        ]
    )

