#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from abb_pkg.action import MoveToPose


class TargetPoseBridgeNode(Node):
    """
    Simple bridge node:

      /target_pose (PoseStamped) -> /move_to_pose (abb_pkg/action/MoveToPose)

    Publishes:
      /move_to_pose/last_success (Bool)
      /move_to_pose/last_message (String)
    """

    def __init__(self):
        super().__init__("lerobot_bridge")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("action_name", "/move_to_pose")
        self.declare_parameter("preempt", True)

        self._target_pose_topic = self.get_parameter("target_pose_topic").value
        self._action_name = self.get_parameter("action_name").value
        self._preempt = bool(self.get_parameter("preempt").value)

        self._client = ActionClient(self, MoveToPose, self._action_name)
        self._goal_handle = None
        self._busy = False

        self._last_success_pub = self.create_publisher(
            Bool, f"{self._action_name}/last_success", 10
        )
        self._last_message_pub = self.create_publisher(
            String, f"{self._action_name}/last_message", 10
        )

        self._sub = self.create_subscription(
            PoseStamped, self._target_pose_topic, self._on_target_pose, 10
        )

        self.get_logger().info(
            f"Bridge: {self._target_pose_topic} -> {self._action_name} (preempt={self._preempt})"
        )

    def _publish_result(self, success: bool, message: str):
        b = Bool()
        b.data = bool(success)
        self._last_success_pub.publish(b)

        s = String()
        s.data = str(message)
        self._last_message_pub.publish(s)

    def _on_target_pose(self, msg: PoseStamped):
        if not self._client.wait_for_server(timeout_sec=0.0):
            if not hasattr(self, '_server_warn_logged'):
                self.get_logger().warn("move_to_pose server not available yet")
                self._server_warn_logged = True
            return

        if self._busy and self._preempt and self._goal_handle is not None:
            self.get_logger().info("Preempting previous goal")
            self._goal_handle.cancel_goal_async()

        goal = MoveToPose.Goal()
        goal.position.x = msg.pose.position.x
        goal.position.y = msg.pose.position.y
        goal.position.z = msg.pose.position.z
        goal.orientation = msg.pose.orientation

        self.get_logger().info(
            f"Received target_pose: x={goal.position.x:.3f}, y={goal.position.y:.3f}, z={goal.position.z:.3f}, "
            f"quat=({goal.orientation.x:.3f}, {goal.orientation.y:.3f}, {goal.orientation.z:.3f}, {goal.orientation.w:.3f})"
        )

        self._busy = True
        fut = self._client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by action server")
            self._publish_result(False, "Goal rejected")
            self._busy = False
            self._goal_handle = None
            return

        self.get_logger().info("Goal accepted by action server")
        self._goal_handle = goal_handle
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            res = future.result().result
            if res.success:
                self.get_logger().info(f"Goal completed successfully: {res.message}")
            else:
                self.get_logger().warn(f"Goal failed: {res.message}")
            self._publish_result(res.success, res.message)
        finally:
            self._busy = False
            self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

