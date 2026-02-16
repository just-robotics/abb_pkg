#!/usr/bin/env python3
"""
Simple script to send pose goal to ABB robot
Usage:
  python3 send_pose.py <x> <y> <z> <roll_deg> <pitch_deg> <yaw_deg>

Legacy (still supported):
  python3 send_pose.py <x> <y> <z> [qx] [qy] [qz] [qw]
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Quaternion
from abb_pkg.action import MoveToPose


def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    """Convert RPY (degrees) to quaternion (x,y,z,w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    if len(sys.argv) < 4:
        print("Usage:")
        print("  python3 send_pose.py <x> <y> <z> <roll_deg> <pitch_deg> <yaw_deg>")
        print("Example:")
        print("  python3 send_pose.py 0.8 0.0 0.6 180 0 -90")
        print("")
        print("Legacy (quat):")
        print("  python3 send_pose.py <x> <y> <z> [qx] [qy] [qz] [qw]")
        print("  python3 send_pose.py 0.8 0.0 0.6 0 0 0 1")
        sys.exit(1)
    
    rclpy.init()
    
    node = Node('send_pose_client')
    action_client = ActionClient(node, MoveToPose, 'move_to_pose')
    
    # Wait for action server
    print("Waiting for action server...")
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Action server not available!")
        rclpy.shutdown()
        sys.exit(1)
    
    # Parse arguments
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])

    # Preferred: RPY degrees (6 args after script name => total 7 argv)
    # Example: send_pose.py x y z roll pitch yaw
    if len(sys.argv) >= 7:
        roll = float(sys.argv[4])
        pitch = float(sys.argv[5])
        yaw = float(sys.argv[6])
        quat = rpy_deg_to_quat(roll, pitch, yaw)
        mode = f"rpy_deg=({roll:.1f},{pitch:.1f},{yaw:.1f})"
    else:
        # Legacy quaternion mode: send_pose.py x y z [qx qy qz qw]
        qx = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        qy = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        qz = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
        qw = float(sys.argv[7]) if len(sys.argv) > 7 else 1.0
        quat = Quaternion(x=qx, y=qy, z=qz, w=qw)
        mode = f"quat=({quat.x:.3f},{quat.y:.3f},{quat.z:.3f},{quat.w:.3f})"
    
    # Create goal
    goal_msg = MoveToPose.Goal()
    goal_msg.position = Point(x=x, y=y, z=z)
    goal_msg.orientation = quat
    
    print(f"Sending pose goal: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    print(f"Orientation: {mode} -> quat=({quat.x:.3f},{quat.y:.3f},{quat.z:.3f},{quat.w:.3f})")
    
    # Send goal
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)
    
    if not send_goal_future.done():
        node.get_logger().error("Failed to send goal")
        rclpy.shutdown()
        sys.exit(1)
    
    goal_handle = send_goal_future.result()
    
    if not goal_handle.accepted:
        node.get_logger().error("Goal was rejected")
        rclpy.shutdown()
        sys.exit(1)
    
    print("Goal accepted! Waiting for result...")
    
    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
    
    if not result_future.done():
        node.get_logger().warn("Action execution timeout")
        goal_handle.cancel_goal_async()
        rclpy.shutdown()
        sys.exit(1)
    
    result = result_future.result().result
    
    if result.success:
        print(f"✓ Success! {result.message}")
        rclpy.shutdown()
        sys.exit(0)
    else:
        print(f"✗ Failed: {result.message}")
        rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
