#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class GamepadTeleopNode(Node):
    """
    Gamepad teleop node for ABB robot.
    Maps Xbox 360 controller to /cmd_vel (Twist).
    
    Controls:
    - axis 1 -: +x, axis 1 +: -x  -> linear.x = -axis[1]
    - axis 0 -: +y, axis 0 +: -y  -> linear.y = -axis[0]
    - axis 2: +z        -> linear.z = axis[2]
    - axis 5: -z        -> linear.z = -axis[5] (combined with axis 2)
    - axis 3 -: -roll, axis 3 +: +roll  -> angular.x = axis[3]
    - axis 4 -: -pitch, axis 4 +: +pitch  -> angular.y = -axis[4]
    - button 4 && axis 3 -: +yaw, button 4 && axis 3 +: -yaw  -> angular.z = axis[3] if button[4]
    """

    def __init__(self):
        super().__init__("gamepad_teleop")
        
        # Basic parameters
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("dead_zone", 0.1)
        self.declare_parameter("publish_rate", 20.0)  # Hz
        
        # Axis sign multipliers
        self.declare_parameter("axis_signs.linear_x", -1.0)
        self.declare_parameter("axis_signs.linear_y", -1.0)
        self.declare_parameter("axis_signs.linear_z_axis2", 1.0)
        self.declare_parameter("axis_signs.linear_z_axis5", -1.0)
        self.declare_parameter("axis_signs.angular_x", 1.0)
        self.declare_parameter("axis_signs.angular_y", -1.0)
        self.declare_parameter("axis_signs.angular_z", 1.0)
        
        # Axis indices
        self.declare_parameter("axis_indices.linear_x", 1)
        self.declare_parameter("axis_indices.linear_y", 0)
        self.declare_parameter("axis_indices.linear_z_axis2", 2)
        self.declare_parameter("axis_indices.linear_z_axis5", 5)
        self.declare_parameter("axis_indices.angular_x", 3)
        self.declare_parameter("axis_indices.angular_y", 4)
        self.declare_parameter("axis_indices.angular_z", 3)
        
        # Button index for yaw
        self.declare_parameter("yaw_button_index", 4)
        
        # Lock other axes when moving along Z
        self.declare_parameter("lock_other_axes_on_z_movement", False)
        
        # Get parameters
        self._joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._dead_zone = self.get_parameter("dead_zone").get_parameter_value().double_value
        self._publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        
        # Axis signs
        self._sign_linear_x = self.get_parameter("axis_signs.linear_x").get_parameter_value().double_value
        self._sign_linear_y = self.get_parameter("axis_signs.linear_y").get_parameter_value().double_value
        self._sign_linear_z_axis2 = self.get_parameter("axis_signs.linear_z_axis2").get_parameter_value().double_value
        self._sign_linear_z_axis5 = self.get_parameter("axis_signs.linear_z_axis5").get_parameter_value().double_value
        self._sign_angular_x = self.get_parameter("axis_signs.angular_x").get_parameter_value().double_value
        self._sign_angular_y = self.get_parameter("axis_signs.angular_y").get_parameter_value().double_value
        self._sign_angular_z = self.get_parameter("axis_signs.angular_z").get_parameter_value().double_value
        
        # Axis indices
        self._idx_linear_x = self.get_parameter("axis_indices.linear_x").get_parameter_value().integer_value
        self._idx_linear_y = self.get_parameter("axis_indices.linear_y").get_parameter_value().integer_value
        self._idx_linear_z_axis2 = self.get_parameter("axis_indices.linear_z_axis2").get_parameter_value().integer_value
        self._idx_linear_z_axis5 = self.get_parameter("axis_indices.linear_z_axis5").get_parameter_value().integer_value
        self._idx_angular_x = self.get_parameter("axis_indices.angular_x").get_parameter_value().integer_value
        self._idx_angular_y = self.get_parameter("axis_indices.angular_y").get_parameter_value().integer_value
        self._idx_angular_z = self.get_parameter("axis_indices.angular_z").get_parameter_value().integer_value
        
        # Yaw button
        self._yaw_button_idx = self.get_parameter("yaw_button_index").get_parameter_value().integer_value
        
        # Lock other axes on Z movement
        self._lock_other_axes_on_z = self.get_parameter("lock_other_axes_on_z_movement").get_parameter_value().bool_value
        
        # Lock other angular axes on angular movement
        self.declare_parameter("lock_other_angular_on_angular_movement", False)
        self._lock_other_angular_on_angular = self.get_parameter("lock_other_angular_on_angular_movement").get_parameter_value().bool_value
        
        # Publisher
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        
        # Subscriber
        self._joy_sub = self.create_subscription(Joy, self._joy_topic, self._joy_callback, 10)
        
        # Track if we've received any joy messages
        self._joy_received = False
        
        # Timer for periodic publishing
        self._publish_timer = self.create_timer(1.0 / self._publish_rate, self._publish_cmd_vel)
        
        # Current cmd_vel values
        self._current_cmd_vel = Twist()
        self._has_input = False
        
        self.get_logger().info(
            f"Gamepad teleop started:\n"
            f"  Subscribing to: {self._joy_topic}\n"
            f"  Publishing to: {self._cmd_vel_topic}\n"
            f"  Publish rate: {self._publish_rate} Hz\n"
            f"  Dead zone: {self._dead_zone}\n"
            f"  Axis signs: linear_x={self._sign_linear_x}, linear_y={self._sign_linear_y}, "
            f"linear_z_axis2={self._sign_linear_z_axis2}, linear_z_axis5={self._sign_linear_z_axis5}, "
            f"angular_x={self._sign_angular_x}, angular_y={self._sign_angular_y}, angular_z={self._sign_angular_z}\n"
            f"  Axis indices: linear_x={self._idx_linear_x}, linear_y={self._idx_linear_y}, "
            f"linear_z_axis2={self._idx_linear_z_axis2}, linear_z_axis5={self._idx_linear_z_axis5}, "
            f"angular_x={self._idx_angular_x}, angular_y={self._idx_angular_y}, angular_z={self._idx_angular_z}\n"
            f"  Yaw button index: {self._yaw_button_idx}\n"
            f"  Lock other axes on Z movement: {self._lock_other_axes_on_z}\n"
            f"  Lock other angular on angular movement: {self._lock_other_angular_on_angular}"
        )

    def _apply_dead_zone(self, value):
        """Apply dead zone to stick input."""
        if abs(value) < self._dead_zone:
            return 0.0
        return value

    def _joy_callback(self, msg: Joy):
        """Process gamepad input and update cmd_vel."""
        if not self._joy_received:
            self._joy_received = True
            self.get_logger().info(f"Received first joy message: axes={len(msg.axes)}, buttons={len(msg.buttons)}")
        
        # Check if we have enough axes/buttons
        max_axis_idx = max(
            self._idx_linear_x, self._idx_linear_y,
            self._idx_linear_z_axis2, self._idx_linear_z_axis5,
            self._idx_angular_x, self._idx_angular_y, self._idx_angular_z
        )
        if len(msg.axes) <= max_axis_idx or len(msg.buttons) <= self._yaw_button_idx:
            self.get_logger().warn_once(
                f"Gamepad message has insufficient axes/buttons: "
                f"axes={len(msg.axes)}, buttons={len(msg.buttons)}, "
                f"required axes>={max_axis_idx+1}, buttons>={self._yaw_button_idx+1}"
            )
            return
        
        # Initialize cmd_vel
        cmd_vel = Twist()
        self._has_input = False
        
        # Linear X
        raw_axis_val = msg.axes[self._idx_linear_x]
        axis_val = self._apply_dead_zone(raw_axis_val)
        cmd_vel.linear.x = self._sign_linear_x * axis_val
        if abs(raw_axis_val) >= self._dead_zone:
            self._has_input = True
        
        # Linear Y
        raw_axis_val = msg.axes[self._idx_linear_y]
        axis_val = self._apply_dead_zone(raw_axis_val)
        cmd_vel.linear.y = self._sign_linear_y * axis_val
        if abs(raw_axis_val) >= self._dead_zone:
            self._has_input = True
        
        # Linear Z: combine axis 2 and axis 5
        raw_axis2_val = msg.axes[self._idx_linear_z_axis2] if len(msg.axes) > self._idx_linear_z_axis2 else 0.0
        raw_axis5_val = msg.axes[self._idx_linear_z_axis5] if len(msg.axes) > self._idx_linear_z_axis5 else 0.0
        axis2_val = self._apply_dead_zone(raw_axis2_val)
        axis5_val = self._apply_dead_zone(raw_axis5_val)
        cmd_vel.linear.z = self._sign_linear_z_axis2 * axis2_val + self._sign_linear_z_axis5 * axis5_val
        if abs(raw_axis2_val) >= self._dead_zone or abs(raw_axis5_val) >= self._dead_zone:
            self._has_input = True
        
        # Check yaw button first
        yaw_button = msg.buttons[self._yaw_button_idx] if len(msg.buttons) > self._yaw_button_idx else 0
        
        # Angular X (Roll): only when yaw button is NOT pressed
        if yaw_button == 0:
            raw_axis_val = msg.axes[self._idx_angular_x] if len(msg.axes) > self._idx_angular_x else 0.0
            axis_val = self._apply_dead_zone(raw_axis_val)
            cmd_vel.angular.x = self._sign_angular_x * axis_val
            if abs(raw_axis_val) >= self._dead_zone:
                self._has_input = True
        else:
            cmd_vel.angular.x = 0.0
        
        # Angular Y (Pitch)
        raw_axis_val = msg.axes[self._idx_angular_y] if len(msg.axes) > self._idx_angular_y else 0.0
        axis_val = self._apply_dead_zone(raw_axis_val)
        cmd_vel.angular.y = self._sign_angular_y * axis_val
        if abs(raw_axis_val) >= self._dead_zone:
            self._has_input = True
        
        # Angular Z (Yaw): only when button is pressed
        if yaw_button == 1:
            raw_axis_val = msg.axes[self._idx_angular_z] if len(msg.axes) > self._idx_angular_z else 0.0
            axis_val = self._apply_dead_zone(raw_axis_val)
            cmd_vel.angular.z = self._sign_angular_z * axis_val
            if abs(raw_axis_val) >= self._dead_zone:
                self._has_input = True
        else:
            cmd_vel.angular.z = 0.0
        
        # Lock other axes when moving along Z (if enabled)
        if self._lock_other_axes_on_z and abs(cmd_vel.linear.z) > 1e-6:
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            # Keep has_input true if Z movement is active
            if abs(cmd_vel.linear.z) > 1e-6:
                self._has_input = True
        
        # Lock other angular axes when moving along one angular axis (if enabled)
        if self._lock_other_angular_on_angular:
            abs_angular_x = abs(cmd_vel.angular.x)
            abs_angular_y = abs(cmd_vel.angular.y)
            abs_angular_z = abs(cmd_vel.angular.z)
            
            has_angular_x = abs_angular_x > 1e-6
            has_angular_y = abs_angular_y > 1e-6
            has_angular_z = abs_angular_z > 1e-6
            
            # Count how many angular axes are active
            active_angular_count = sum([has_angular_x, has_angular_y, has_angular_z])
            
            # If at least one angular axis is active, keep only the one with maximum absolute value
            if active_angular_count >= 1:
                # Find which axis has the maximum absolute value
                max_val = max(abs_angular_x, abs_angular_y, abs_angular_z)
                
                if abs_angular_x == max_val:
                    # Keep angular.x, block others
                    cmd_vel.angular.y = 0.0
                    cmd_vel.angular.z = 0.0
                elif abs_angular_y == max_val:
                    # Keep angular.y, block others
                    cmd_vel.angular.x = 0.0
                    cmd_vel.angular.z = 0.0
                elif abs_angular_z == max_val:
                    # Keep angular.z, block others
                    cmd_vel.angular.x = 0.0
                    cmd_vel.angular.y = 0.0
        
        # Update current cmd_vel
        self._current_cmd_vel = cmd_vel
        
        # Debug logging (only when there's input)
        if self._has_input:
            self.get_logger().debug(
                f"Joy input: axes={[f'{v:.2f}' for v in msg.axes[:6]]}, "
                f"buttons={msg.buttons[:5]}, "
                f"cmd_vel=({cmd_vel.linear.x:.3f}, {cmd_vel.linear.y:.3f}, {cmd_vel.linear.z:.3f}, "
                f"{cmd_vel.angular.x:.3f}, {cmd_vel.angular.y:.3f}, {cmd_vel.angular.z:.3f})"
            )

    def _publish_cmd_vel(self):
        """Publish current cmd_vel."""
        # Always publish, even if zero
        self._cmd_vel_pub.publish(self._current_cmd_vel)
        
        if self._has_input:
            self.get_logger().info(
                f"Published cmd_vel: "
                f"linear=({self._current_cmd_vel.linear.x:.3f}, {self._current_cmd_vel.linear.y:.3f}, {self._current_cmd_vel.linear.z:.3f}), "
                f"angular=({self._current_cmd_vel.angular.x:.3f}, {self._current_cmd_vel.angular.y:.3f}, {self._current_cmd_vel.angular.z:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
