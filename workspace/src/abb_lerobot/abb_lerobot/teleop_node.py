#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from abb_pkg.action import MoveToPose


class AbbTeleopNode(Node):

    def __init__(self):
        super().__init__('abb_teleop')
        
        # Action client for sending poses
        self.action_client_ = ActionClient(self, MoveToPose, 'move_to_pose')
        
        # Subscriptions for robot state
        self.ee_pose_sub_ = self.create_subscription(
            PoseStamped, 'ee_pose', self.ee_pose_callback, 10)
        self.joint_state_sub_ = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.ping_sub_ = self.create_subscription(
            Bool, 'abb_ping', self.ping_callback, 1)
        
        # Current robot state
        self.current_ee_pose_ = None
        self.current_joint_state_ = None
        self.connected_ = False
        self.ping_time_ = None
        
        # Action execution state
        self.current_goal_handle_ = None
        self.action_result_ = None
        
        # Timer for connection check
        self.timer_ = self.create_timer(0.1, self.ping_timer_callback)
        
        self.get_logger().info('ABB Teleop node started')
        self.get_logger().info('Waiting for action server...')
        
    def now(self):
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns * 1e-9
        
    def ping_callback(self, msg: Bool):
        if not msg.data:
            return
        self.ping_time_ = self.now()
            
    def ping_timer_callback(self):
        if self.ping_time_ is None:
            self.connected_ = False
            return
        
        delta = self.now() - self.ping_time_
        was_connected = self.connected_
        self.connected_ = False if delta > 0.2 else True
        
        if was_connected != self.connected_:
            self.get_logger().info(f'Robot connection: {self.connected_}')

    def ee_pose_callback(self, msg: PoseStamped):
        """Callback for end effector pose updates"""
        self.current_ee_pose_ = msg
        self.get_logger().debug(
            f'EE Pose: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}')
    
    def joint_state_callback(self, msg: JointState):
        """Callback for joint state updates"""
        self.current_joint_state_ = msg
        if msg.position:
            self.get_logger().debug(
                f'Joint positions: {[f"{p:.3f}" for p in msg.position[:3]]}...')
    
    def send_pose_goal_async(self, position: Point, orientation: Quaternion):
        """
        Send a pose goal to the robot asynchronously
        
        Args:
            position: Target position (in meters)
            orientation: Target orientation (quaternion)
            
        Returns:
            Future for goal handle, or None if server not available
        """
        # Wait for action server
        if not self.action_client_.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return None
        
        # Create goal
        goal_msg = MoveToPose.Goal()
        goal_msg.position = position
        goal_msg.orientation = orientation
        
        self.get_logger().info(
            f'Sending pose goal: x={position.x:.3f}, y={position.y:.3f}, '
            f'z={position.z:.3f}')
        
        # Send goal
        send_goal_future = self.action_client_.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        return send_goal_future
    
    def send_pose_goal(self, position: Point, orientation: Quaternion, 
                       timeout_sec: float = 30.0) -> bool:
        """
        Send a pose goal to the robot and wait for result
        
        Args:
            position: Target position (in meters)
            orientation: Target orientation (quaternion)
            timeout_sec: Timeout for action execution
            
        Returns:
            True if goal succeeded, False otherwise
        """
        send_goal_future = self.send_pose_goal_async(position, orientation)
        
        if send_goal_future is None:
            return False
        
        # Wait for goal to be accepted (non-blocking check)
        # Note: In a real application, you should handle this in a separate thread
        # or use async callbacks. For simplicity, we use a timeout loop.
        import time
        start_time = time.time()
        while not send_goal_future.done() and (time.time() - start_time) < 5.0:
            time.sleep(0.01)
        
        if not send_goal_future.done():
            self.get_logger().error('Failed to send goal (timeout)')
            return False
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return False
        
        self.get_logger().info('Goal accepted, executing...')
        self.current_goal_handle_ = goal_handle
        
        # Get result future
        result_future = goal_handle.get_result_async()
        
        # Wait for result with timeout
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            if result_future.done():
                break
            time.sleep(0.1)
        
        if not result_future.done():
            self.get_logger().warn('Action execution timeout')
            # Cancel goal
            goal_handle.cancel_goal_async()
            self.action_result_ = None
            self.current_goal_handle_ = None
            return False
        
        result = result_future.result().result
        self.action_result_ = result
        
        if result.success:
            self.get_logger().info(f'Goal succeeded: {result.message}')
        else:
            self.get_logger().warn(f'Goal failed: {result.message}')
        
        self.current_goal_handle_ = None
        return result.success
    
    def feedback_callback(self, feedback_msg):
        """Callback for action feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Feedback: distance={feedback.distance_to_target:.3f}m, '
            f'current_pos=({feedback.current_position.x:.3f}, '
            f'{feedback.current_position.y:.3f}, '
            f'{feedback.current_position.z:.3f})')
    
    def cancel_current_goal(self):
        """Cancel the current goal if any"""
        if self.current_goal_handle_ is not None:
            self.get_logger().info('Cancelling current goal...')
            self.current_goal_handle_.cancel_goal_async()
            self.current_goal_handle_ = None
            self.action_result_ = None
    
    def get_current_ee_pose(self) -> PoseStamped:
        """Get current end effector pose"""
        return self.current_ee_pose_
    
    def get_current_joint_state(self) -> JointState:
        """Get current joint state"""
        return self.current_joint_state_
    
    def get_last_action_result(self):
        """Get last action result"""
        return self.action_result_
    
    def is_connected(self) -> bool:
        """Check if robot is connected"""
        return self.connected_


def main(args=None):
    rclpy.init(args=args)
    node = AbbTeleopNode()
    
    # Example usage:
    # # Create a pose goal
    # position = Point()
    # position.x = 0.75  # meters
    # position.y = 0.0
    # position.z = 1.1215
    # 
    # orientation = Quaternion()
    # orientation.w = 1.0  # Identity quaternion
    # 
    # # Send goal and wait for result
    # success = node.send_pose_goal(position, orientation, timeout_sec=30.0)
    # if success:
    #     print("Robot reached target pose!")
    # else:
    #     print("Failed to reach target pose")
    # 
    # # Or use async version
    # future = node.send_pose_goal_async(position, orientation)
    # # Handle future in your own callback
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node.cancel_current_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
