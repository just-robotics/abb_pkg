"""Generate a Cartesian trajectory and execute it through MoveIt.

Talks to move_group only through its ROS interfaces, so it needs no MoveIt
libraries and does not care whether the robot is simulated or real:
  1. /move_action           - plan + execute a free motion to the first waypoint
  2. /compute_cartesian_path - straight-line interpolation through the waypoints
  3. /execute_trajectory    - execute it on joint_trajectory_controller

Trigger: std_msgs/String on figure_type ("circle" or "line"), or the
autostart_figure parameter.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from abb_trajectory_generator.moveit_client import MoveItClient, make_pose, rpy_to_quaternion


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')

        group_name = self.declare_parameter('group_name', 'manipulator').value
        frame_id = self.declare_parameter('frame_id', 'base_link').value
        ee_link = self.declare_parameter('ee_link', 'tool0').value
        self.orientation_rpy_deg = list(
            self.declare_parameter('orientation_rpy_deg', [180.0, 0.0, -90.0]).value)
        self.circle_center = list(self.declare_parameter('circle_center', [0.75, 0.0, 0.7]).value)
        self.circle_radius = self.declare_parameter('circle_radius', 0.2).value
        self.circle_points = self.declare_parameter('circle_points', 72).value
        self.line_start = list(self.declare_parameter('line_start', [0.75, -0.4, 0.7]).value)
        self.line_end = list(self.declare_parameter('line_end', [0.75, 0.4, 0.7]).value)
        self.line_points = self.declare_parameter('line_points', 10).value
        self.max_step = self.declare_parameter('max_step', 0.005).value
        self.min_fraction = self.declare_parameter('min_fraction', 0.99).value
        self.velocity_scaling = self.declare_parameter('velocity_scaling', 0.1).value
        self.acceleration_scaling = self.declare_parameter('acceleration_scaling', 0.1).value
        autostart_figure = self.declare_parameter('autostart_figure', '').value

        self.moveit = MoveItClient(self, group_name, frame_id, ee_link)

        self.busy = threading.Lock()
        self.create_subscription(String, 'figure_type', self.figure_callback, 10)

        if autostart_figure:
            self.start(autostart_figure)

        self.get_logger().info('Ready, publish "circle" or "line" to figure_type')

    def figure_callback(self, msg):
        self.start(msg.data.strip())

    def start(self, figure):
        if figure not in ('circle', 'line'):
            self.get_logger().error(f'Unknown figure "{figure}", expected circle or line')
            return
        if not self.busy.acquire(blocking=False):
            self.get_logger().warn(f'Busy, ignoring "{figure}"')
            return
        # Waiting on MoveIt must not block the executor that delivers its responses.
        threading.Thread(target=self.run, args=(figure,), daemon=True).start()

    def run(self, figure):
        try:
            ok, message = self.execute_figure(figure)
            if ok:
                self.get_logger().info(f'{figure}: done')
            else:
                self.get_logger().error(f'{figure}: {message}')
        finally:
            self.busy.release()

    def execute_figure(self, figure):
        waypoints = self.circle() if figure == 'circle' else self.line()

        if not self.moveit.wait_for_servers():
            return False, 'move_group is not available'

        self.get_logger().info(f'{figure}: moving to the start pose')
        ok, message = self.moveit.move_to_pose(
            waypoints[0], self.velocity_scaling, self.acceleration_scaling)
        if not ok:
            return False, message

        self.get_logger().info(f'{figure}: executing {len(waypoints) - 1} waypoints')
        return self.moveit.follow_path(
            waypoints[1:], self.velocity_scaling, self.acceleration_scaling,
            max_step=self.max_step, min_fraction=self.min_fraction)

    def circle(self):
        cx, cy, cz = self.circle_center
        n = max(3, self.circle_points)
        # closed loop: the last point equals the first one
        return [self.pose(cx + self.circle_radius * math.cos(2 * math.pi * i / n),
                          cy + self.circle_radius * math.sin(2 * math.pi * i / n),
                          cz)
                for i in range(n + 1)]

    def line(self):
        n = max(1, self.line_points)
        return [self.pose(*(s + (e - s) * i / n for s, e in zip(self.line_start, self.line_end)))
                for i in range(n + 1)]

    def pose(self, x, y, z):
        q = rpy_to_quaternion(*(math.radians(a) for a in self.orientation_rpy_deg))
        return make_pose(x, y, z, q)


def main():
    rclpy.init()
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
