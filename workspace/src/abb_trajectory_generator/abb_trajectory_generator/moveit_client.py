"""Thin client for move_group's ROS interfaces (no MoveIt libraries needed).

All calls block the calling thread, so use them from a worker thread while
the node is spun elsewhere.
"""

import math
import threading
import time

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, MoveItErrorCodes, PlanningSceneComponents
from moveit_msgs.msg import OrientationConstraint, PositionConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

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


def make_pose(x, y, z, q):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def wait_future(future, timeout=SERVER_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.01)
    return future.result()


def error_text(result):
    return 'no response' if result is None else f'MoveIt error {result.error_code.val}'


class MoveItClient:

    def __init__(self, node, group_name, frame_id, ee_link):
        self.group_name = group_name
        self.frame_id = frame_id
        self.ee_link = ee_link
        self.move_client = ActionClient(node, MoveGroup, 'move_action')
        self.execute_client = ActionClient(node, ExecuteTrajectory, 'execute_trajectory')
        self.cartesian_client = node.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.scene_client = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.get_scene_client = node.create_client(GetPlanningScene, 'get_planning_scene')
        self._goal_lock = threading.Lock()
        self._goal_handle = None

    def wait_for_servers(self, timeout=SERVER_TIMEOUT_S):
        return (self.move_client.wait_for_server(timeout_sec=timeout)
                and self.execute_client.wait_for_server(timeout_sec=timeout)
                and self.cartesian_client.wait_for_service(timeout_sec=timeout)
                and self.scene_client.wait_for_service(timeout_sec=timeout)
                and self.get_scene_client.wait_for_service(timeout_sec=timeout))

    def move_to_pose(self, pose, velocity_scaling, acceleration_scaling):
        """Collision-free motion of ee_link to pose. Returns (ok, message)."""
        result = self._run_action(
            self.move_client, self._pose_goal(pose, velocity_scaling, acceleration_scaling))
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'planning to pose failed ({error_text(result)})'
        return True, ''

    def follow_path(self, waypoints, velocity_scaling, acceleration_scaling,
                    avoid_collisions=True, max_step=0.005, min_fraction=0.99):
        """Straight-line motion of ee_link through waypoints. Returns (ok, message)."""
        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame_id
        request.start_state.is_diff = True
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.waypoints = list(waypoints)
        request.max_step = max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = avoid_collisions
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        response = wait_future(self.cartesian_client.call_async(request))
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'Cartesian planning failed ({error_text(response)})'
        if response.fraction < min_fraction:
            return False, f'only {response.fraction:.0%} of the path is feasible'

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        result = self._run_action(self.execute_client, goal)
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'execution failed ({error_text(result)})'
        return True, ''

    def apply_scene(self, scene):
        response = wait_future(
            self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene)))
        return response is not None and response.success

    def scene_object_ids(self):
        """Ids of (world, attached) collision objects, or None if move_group did not answer."""
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                          | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        response = wait_future(self.get_scene_client.call_async(request))
        if response is None:
            return None
        scene = response.scene
        return ([o.id for o in scene.world.collision_objects],
                [a.object.id for a in scene.robot_state.attached_collision_objects])

    def cancel(self):
        """Stop the running motion, if any."""
        with self._goal_lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()

    def _run_action(self, client, goal):
        goal_handle = wait_future(client.send_goal_async(goal))
        if goal_handle is None or not goal_handle.accepted:
            return None
        with self._goal_lock:
            self._goal_handle = goal_handle
        try:
            response = wait_future(goal_handle.get_result_async(), MOTION_TIMEOUT_S)
        finally:
            with self._goal_lock:
                self._goal_handle = None
        return None if response is None else response.result

    def _pose_goal(self, pose, velocity_scaling, acceleration_scaling):
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
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = acceleration_scaling
        goal.request.goal_constraints.append(constraints)
        # empty diffs = "use the current state and scene", including the attached tool
        goal.request.start_state.is_diff = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        return goal
