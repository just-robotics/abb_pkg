#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class CmdVelToTargetPoseNode(Node):
    """
    Node that reads cmd_vel (Twist) and publishes target_pose (PoseStamped).
    Integrates velocities with configurable scaling factors for each axis.
    """

    def __init__(self):
        super().__init__("cmd_vel_to_target_pose")
        
        # Parameters
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("ee_pose_topic", "/ee_pose")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate", 20.0)  # Hz
        
        # Scaling factors for each axis (cmd_vel -> coord_dot)
        self.declare_parameter("scale.linear_x", 0.01)  # m/s -> m
        self.declare_parameter("scale.linear_y", 0.01)  # m/s -> m
        self.declare_parameter("scale.linear_z", 0.01)  # m/s -> m
        self.declare_parameter("scale.angular_x", 1.0)   # rad/s -> rad (roll)
        self.declare_parameter("scale.angular_y", 1.0)   # rad/s -> rad (pitch)
        self.declare_parameter("scale.angular_z", 1.0)   # rad/s -> rad (yaw)
        
        # Get parameters
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        self._ee_pose_topic = self.get_parameter("ee_pose_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        
        # Scaling factors
        self._scale_linear_x = self.get_parameter("scale.linear_x").get_parameter_value().double_value
        self._scale_linear_y = self.get_parameter("scale.linear_y").get_parameter_value().double_value
        self._scale_linear_z = self.get_parameter("scale.linear_z").get_parameter_value().double_value
        self._scale_angular_x = self.get_parameter("scale.angular_x").get_parameter_value().double_value
        self._scale_angular_y = self.get_parameter("scale.angular_y").get_parameter_value().double_value
        self._scale_angular_z = self.get_parameter("scale.angular_z").get_parameter_value().double_value
        
        # Current pose (in meters and radians) - will be initialized from robot state
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_z = 0.0
        self._current_roll = 0.0
        self._current_pitch = 0.0
        self._current_yaw = 0.0
        self._pose_initialized = False
        
        # Publishers
        self._target_pose_pub = self.create_publisher(PoseStamped, self._target_pose_topic, 10)
        
        # Subscribers
        self._cmd_vel_sub = self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_callback, 10)
        self._ee_pose_sub = self.create_subscription(PoseStamped, self._ee_pose_topic, self._ee_pose_callback, 10)
        
        # Timer for periodic publishing and integration
        self._update_timer = self.create_timer(1.0 / self._publish_rate, self._update_and_publish)
        
        # Last cmd_vel received
        self._last_cmd_vel = Twist()
        self._last_cmd_vel_time = None
        
        self.get_logger().info(
            f"CmdVel to TargetPose node started:\n"
            f"  Subscribing to: {self._cmd_vel_topic}, {self._ee_pose_topic}\n"
            f"  Publishing to: {self._target_pose_topic}\n"
            f"  Frame ID: {self._frame_id}\n"
            f"  Publish rate: {self._publish_rate} Hz\n"
            f"  Scaling factors:\n"
            f"    linear:  x={self._scale_linear_x:.4f}, y={self._scale_linear_y:.4f}, z={self._scale_linear_z:.4f}\n"
            f"    angular: x={self._scale_angular_x:.4f}, y={self._scale_angular_y:.4f}, z={self._scale_angular_z:.4f}\n"
            f"  Waiting for initial pose from {self._ee_pose_topic}..."
        )

    def _ee_pose_callback(self, msg: PoseStamped):
        """Initialize current pose from robot state."""
        if not self._pose_initialized:
            self._current_x = msg.pose.position.x
            self._current_y = msg.pose.position.y
            self._current_z = msg.pose.position.z
            
            # Convert quaternion to RPY
            quat = msg.pose.orientation
            euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self._current_roll = euler[0]
            self._current_pitch = euler[1]
            self._current_yaw = euler[2]
            
            self._pose_initialized = True
            self.get_logger().info(
                f"Initialized pose from robot state: "
                f"x={self._current_x:.3f}, y={self._current_y:.3f}, z={self._current_z:.3f}, "
                f"rpy=({math.degrees(self._current_roll):.1f}°, {math.degrees(self._current_pitch):.1f}°, {math.degrees(self._current_yaw):.1f}°)"
            )

    def _cmd_vel_callback(self, msg: Twist):
        """Store the latest cmd_vel message."""
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = self.get_clock().now()

    def _update_and_publish(self):
        """Update current pose based on cmd_vel and publish target_pose."""
        # Don't publish until pose is initialized from robot state
        if not self._pose_initialized:
            return
        
        now = self.get_clock().now()
        
        # Calculate time delta
        if self._last_cmd_vel_time is not None:
            dt = (now - self._last_cmd_vel_time).nanoseconds / 1e9
        else:
            dt = 1.0 / self._publish_rate  # Default to one timestep
        
        # Limit dt to prevent large jumps
        dt = min(dt, 0.1)  # Max 100ms
        
        # Update position (integrate linear velocities)
        self._current_x += self._last_cmd_vel.linear.x * self._scale_linear_x * dt
        self._current_y += self._last_cmd_vel.linear.y * self._scale_linear_y * dt
        self._current_z += self._last_cmd_vel.linear.z * self._scale_linear_z * dt
        
        # Update orientation (integrate angular velocities)
        self._current_roll += self._last_cmd_vel.angular.x * self._scale_angular_x * dt
        self._current_pitch += self._last_cmd_vel.angular.y * self._scale_angular_y * dt
        self._current_yaw += self._last_cmd_vel.angular.z * self._scale_angular_z * dt
        
        # Normalize yaw to [-pi, pi]
        self._current_yaw = math.atan2(math.sin(self._current_yaw), math.cos(self._current_yaw))
        
        # Publish target pose
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        
        msg.pose.position.x = self._current_x
        msg.pose.position.y = self._current_y
        msg.pose.position.z = self._current_z
        
        # Convert RPY to quaternion
        quat = quaternion_from_euler(
            self._current_roll,
            self._current_pitch,
            self._current_yaw
        )
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self._target_pose_pub.publish(msg)
        
        # Log published target pose (throttled to once per second)
        if not hasattr(self, '_publish_counter'):
            self._publish_counter = 0
        self._publish_counter += 1
        if self._publish_counter % 20 == 0:  # Every ~1 second at 20Hz
            self.get_logger().info(
                f"Published target_pose: x={self._current_x:.3f}, y={self._current_y:.3f}, z={self._current_z:.3f}, "
                f"rpy=({math.degrees(self._current_roll):.1f}°, {math.degrees(self._current_pitch):.1f}°, {math.degrees(self._current_yaw):.1f}°)"
            )
        
        # Update last time
        self._last_cmd_vel_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToTargetPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
