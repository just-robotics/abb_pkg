import math
import subprocess
import threading
import time
from typing import List

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool


class PathExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("rrt_path_executor_node")

        self.path_topic = self.declare_parameter("path_topic", "/planned_path").value
        self.send_path_topic = self.declare_parameter("send_path_topic", "/send_path").value
        self.send_pose_script = self.declare_parameter(
            "send_pose_script", "/workspace/src/send_pose.sh"
        ).value

        self.default_roll_deg = float(self.declare_parameter("default_roll_deg", 180.0).value)
        self.default_pitch_deg = float(self.declare_parameter("default_pitch_deg", 0.0).value)
        self.default_yaw_deg = float(self.declare_parameter("default_yaw_deg", -90.0).value)
        self.use_pose_orientation = bool(self.declare_parameter("use_pose_orientation", False).value)

        self.call_timeout_sec = float(self.declare_parameter("call_timeout_sec", 30.0).value)
        self.pause_between_points_sec = float(
            self.declare_parameter("pause_between_points_sec", 0.1).value
        )
        self.stop_on_failure = bool(self.declare_parameter("stop_on_failure", True).value)

        self._path_lock = threading.Lock()
        self._latest_path: List = []
        self._is_executing = False

        self.create_subscription(Path, self.path_topic, self._path_callback, 10)
        self.create_subscription(Bool, self.send_path_topic, self._send_path_callback, 10)

        self.get_logger().info(
            f"rrt_path_executor_node started. path_topic={self.path_topic}, "
            f"send_path_topic={self.send_path_topic}, script={self.send_pose_script}"
        )

    def _path_callback(self, msg: Path) -> None:
        with self._path_lock:
            self._latest_path = list(msg.poses)
        self.get_logger().info(f"Stored new path with {len(msg.poses)} points")

    def _send_path_callback(self, msg: Bool) -> None:
        if not msg.data:
            return

        with self._path_lock:
            path_copy = list(self._latest_path)
            if self._is_executing:
                self.get_logger().warn("Path execution already in progress, trigger ignored")
                return
            if not path_copy:
                self.get_logger().warn("No path available to execute")
                return
            self._is_executing = True

        worker = threading.Thread(
            target=self._execute_path_worker,
            args=(path_copy,),
            daemon=True,
        )
        worker.start()

    def _execute_path_worker(self, path_poses: List) -> None:
        self.get_logger().info(f"Starting path execution for {len(path_poses)} points")
        try:
            for idx, pose_stamped in enumerate(path_poses):
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                z = pose_stamped.pose.position.z

                roll_deg, pitch_deg, yaw_deg = self._orientation_for_pose(pose_stamped)

                cmd = [
                    "/bin/bash",
                    self.send_pose_script,
                    str(x),
                    str(y),
                    str(z),
                    str(roll_deg),
                    str(pitch_deg),
                    str(yaw_deg),
                ]

                self.get_logger().info(
                    f"[{idx + 1}/{len(path_poses)}] send_pose: "
                    f"x={x:.3f}, y={y:.3f}, z={z:.3f}, rpy=({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f})"
                )

                try:
                    result = subprocess.run(
                        cmd,
                        capture_output=True,
                        text=True,
                        timeout=self.call_timeout_sec,
                        check=False,
                    )
                except subprocess.TimeoutExpired:
                    self.get_logger().error(
                        f"[{idx + 1}/{len(path_poses)}] send_pose timeout after {self.call_timeout_sec:.1f}s"
                    )
                    if self.stop_on_failure:
                        break
                    continue
                except Exception as exc:  # pylint: disable=broad-except
                    self.get_logger().error(f"[{idx + 1}/{len(path_poses)}] send_pose failed: {exc}")
                    if self.stop_on_failure:
                        break
                    continue

                if result.returncode != 0:
                    stderr = (result.stderr or "").strip()
                    self.get_logger().error(
                        f"[{idx + 1}/{len(path_poses)}] send_pose returned {result.returncode}. {stderr}"
                    )
                    if self.stop_on_failure:
                        break
                elif result.stdout:
                    self.get_logger().debug(result.stdout.strip())

                if self.pause_between_points_sec > 0.0:
                    time.sleep(self.pause_between_points_sec)
        finally:
            with self._path_lock:
                self._is_executing = False
            self.get_logger().info("Path execution finished")

    def _orientation_for_pose(self, pose_stamped) -> tuple:
        if not self.use_pose_orientation:
            return self.default_roll_deg, self.default_pitch_deg, self.default_yaw_deg

        q = pose_stamped.pose.orientation
        roll, pitch, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    @staticmethod
    def _quat_to_euler(x: float, y: float, z: float, w: float) -> tuple:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
