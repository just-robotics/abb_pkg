#!/usr/bin/env python3

import json
import os
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


def _to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class Snapshot:
    joint: Optional[JointState] = None
    ee_pose: Optional[PoseStamped] = None
    target_pose: Optional[PoseStamped] = None
    last_success: Optional[Bool] = None
    last_message: Optional[String] = None


class DatasetRecorder(Node):
    """
    Minimal dataset recorder for abb_pkg + lerobot bridge.

    Records time-series samples to JSONL:
      obs: joint_states + ee_pose
      action: latest target_pose
      result: last_success/last_message (from move_to_pose bridge)
    """

    def __init__(self):
        super().__init__("abb_dataset_recorder")

        self.declare_parameter("output_dir", "./dataset")
        self.declare_parameter("file_name", "dataset.jsonl")
        self.declare_parameter("rate_hz", 10.0)

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("ee_pose_topic", "/ee_pose")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("last_success_topic", "/move_to_pose/last_success")
        self.declare_parameter("last_message_topic", "/move_to_pose/last_message")

        out_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        file_name = self.get_parameter("file_name").get_parameter_value().string_value
        self._rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value

        self._joint_topic = self.get_parameter("joint_states_topic").value
        self._ee_pose_topic = self.get_parameter("ee_pose_topic").value
        self._target_pose_topic = self.get_parameter("target_pose_topic").value
        self._last_success_topic = self.get_parameter("last_success_topic").value
        self._last_message_topic = self.get_parameter("last_message_topic").value

        os.makedirs(out_dir, exist_ok=True)
        self._path = os.path.join(out_dir, file_name)
        self._fh = open(self._path, "a", encoding="utf-8")

        self._snap = Snapshot()

        self.create_subscription(JointState, self._joint_topic, self._on_joint, 50)
        self.create_subscription(PoseStamped, self._ee_pose_topic, self._on_pose, 50)
        self.create_subscription(PoseStamped, self._target_pose_topic, self._on_target, 50)
        self.create_subscription(Bool, self._last_success_topic, self._on_success, 10)
        self.create_subscription(String, self._last_message_topic, self._on_message, 10)

        period = 1.0 / self._rate_hz if self._rate_hz > 0 else 0.1
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"Recording to: {self._path} @ {self._rate_hz} Hz")

    def destroy_node(self):
        try:
            if self._fh:
                self._fh.flush()
                self._fh.close()
        finally:
            super().destroy_node()

    def _on_joint(self, msg: JointState):
        self._snap.joint = msg

    def _on_pose(self, msg: PoseStamped):
        self._snap.ee_pose = msg

    def _on_target(self, msg: PoseStamped):
        self._snap.target_pose = msg

    def _on_success(self, msg: Bool):
        self._snap.last_success = msg

    def _on_message(self, msg: String):
        self._snap.last_message = msg

    def _tick(self):
        js = self._snap.joint
        pose = self._snap.ee_pose
        target = self._snap.target_pose

        if js is None or pose is None:
            return  # no obs yet

        # Build sample
        sample = {
            "t_wall": time.time(),
            "t_joint": _to_sec(js.header.stamp) if js.header.stamp else None,
            "t_pose": _to_sec(pose.header.stamp) if pose.header.stamp else None,
            "obs": {
                "joint": {
                    "name": list(js.name),
                    "position": list(js.position),
                    "velocity": list(js.velocity) if js.velocity else [],
                    "effort": list(js.effort) if js.effort else [],
                },
                "ee_pose": {
                    "frame_id": pose.header.frame_id,
                    "position": {
                        "x": pose.pose.position.x,
                        "y": pose.pose.position.y,
                        "z": pose.pose.position.z,
                    },
                    "orientation": {
                        "x": pose.pose.orientation.x,
                        "y": pose.pose.orientation.y,
                        "z": pose.pose.orientation.z,
                        "w": pose.pose.orientation.w,
                    },
                },
            },
            "action": None,
            "result": None,
        }

        if target is not None:
            sample["action"] = {
                "frame_id": target.header.frame_id,
                "position": {
                    "x": target.pose.position.x,
                    "y": target.pose.position.y,
                    "z": target.pose.position.z,
                },
                "orientation": {
                    "x": target.pose.orientation.x,
                    "y": target.pose.orientation.y,
                    "z": target.pose.orientation.z,
                    "w": target.pose.orientation.w,
                },
            }

        if self._snap.last_success is not None or self._snap.last_message is not None:
            sample["result"] = {
                "success": bool(self._snap.last_success.data)
                if self._snap.last_success is not None
                else None,
                "message": self._snap.last_message.data
                if self._snap.last_message is not None
                else None,
            }

        self._fh.write(json.dumps(sample, ensure_ascii=False) + "\n")
        # Flush lightly to reduce data loss on crash
        self._fh.flush()


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

