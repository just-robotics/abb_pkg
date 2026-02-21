from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
  pkg_path = get_package_share_directory('vel_control')
  params = os.path.join(pkg_path, 'config', 'params.yaml')

  vel_control_node = Node(
      package='vel_control',
      executable='vel_control_node',
      name='vel_control_node',
      output='screen',
      parameters=[params],
  )

  return LaunchDescription([vel_control_node])
