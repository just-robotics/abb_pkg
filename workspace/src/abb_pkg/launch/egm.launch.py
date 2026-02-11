from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_path = get_package_share_directory('abb_pkg')
    
    params = os.path.join(pkg_path, 'config', 'params.yaml')
    urdf = os.path.join(pkg_path, "urdf", "irb1600.urdf")
    
    with open(urdf, "r", encoding="utf-8") as f:
        robot_description = f.read()

    egm_node = Node(
        package='abb_pkg',
        executable='egm_node',
        name='egm_node',
        output='screen',
        parameters=[
            params,
            {"robot_description": robot_description},
        ],
    )

    return LaunchDescription([
        egm_node
    ])
