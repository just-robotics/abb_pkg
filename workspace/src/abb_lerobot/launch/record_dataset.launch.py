from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    recorder = Node(
        package="abb_lerobot",
        executable="dataset_recorder",
        name="dataset_recorder",
        output="screen",
        parameters=[
            {"output_dir": "./dataset"},
            {"file_name": "dataset.jsonl"},
            {"rate_hz": 10.0},
            {"joint_states_topic": "/joint_states"},
            {"ee_pose_topic": "/ee_pose"},
            {"target_pose_topic": "/target_pose"},
            {"last_success_topic": "/move_to_pose/last_success"},
            {"last_message_topic": "/move_to_pose/last_message"},
        ],
    )

    return LaunchDescription([recorder])

