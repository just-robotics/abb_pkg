from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("camera_wrapper"),
        "config",
        "params.yaml"
    )

    return LaunchDescription([
        Node(
            package="camera_wrapper",
            executable="pylon_camera_node",
            name="pylon_camera",
            parameters=[config],
            output="screen",
        )
    ])
