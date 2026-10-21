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
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, MoveItErrorCodes
from moveit_msgs.msg import OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

SERVER_TIMEOUT_S = 30.0
MOTION_TIMEOUT_S = 300.0


def rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')

        self.group_name = self.declare_parameter('group_name', 'manipulator').value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.ee_link = self.declare_parameter('ee_link', 'tool0').value
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

        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')

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

        if not (self.move_client.wait_for_server(timeout_sec=SERVER_TIMEOUT_S)
                and self.execute_client.wait_for_server(timeout_sec=SERVER_TIMEOUT_S)
                and self.cartesian_client.wait_for_service(timeout_sec=SERVER_TIMEOUT_S)):
            return False, 'move_group is not available'

        self.get_logger().info(f'{figure}: moving to the start pose')
        result = self.wait(self.move_client.send_goal_async(self.pose_goal(waypoints[0])))
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'failed to reach the start pose ({self.error(result)})'

        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame_id
        request.start_state.is_diff = True
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.waypoints = waypoints[1:]
        request.max_step = self.max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        response = self.wait_future(self.cartesian_client.call_async(request))
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'Cartesian planning failed ({self.error(response)})'
        if response.fraction < self.min_fraction:
            return False, f'only {response.fraction:.0%} of the path is feasible'

        self.get_logger().info(
            f'{figure}: executing {len(response.solution.joint_trajectory.points)} points')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        result = self.wait(self.execute_client.send_goal_async(goal))
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'execution failed ({self.error(result)})'
        return True, ''

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
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        q = rpy_to_quaternion(*(math.radians(a) for a in self.orientation_rpy_deg))
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        return pose

    def pose_goal(self, pose):
        position = PositionConstraint()
        position.header.frame_id = self.frame_id
        position.link_name = self.ee_link
        region = BoundingVolume()
        region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001]))
        center = Pose()
        center.position = pose.position
        center.orientation.w = 1.0
        region.primitive_poses.append(center)
        position.constraint_region = region
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.frame_id
        orientation.link_name = self.ee_link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        return goal

    def wait(self, send_goal_future):
        goal_handle = self.wait_future(send_goal_future)
        if goal_handle is None or not goal_handle.accepted:
            return None
        response = self.wait_future(goal_handle.get_result_async(), MOTION_TIMEOUT_S)
        return None if response is None else response.result

    @staticmethod
    def wait_future(future, timeout=SERVER_TIMEOUT_S):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.01)
        return future.result()

    @staticmethod
    def error(result):
        return 'no response' if result is None else f'MoveIt error {result.error_code.val}'


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
