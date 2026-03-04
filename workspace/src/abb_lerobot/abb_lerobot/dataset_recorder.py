#!/usr/bin/env python3

import os
import json
import cv2
import rclpy
import time

from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__("dataset_recorder")

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter("dataset_root", "dataset")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("ee_pose_topic", "/ee_pose")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("gripper_topic", "/gripper/command")
        self.declare_parameter("sync_slop", 0.1)
        self.declare_parameter("sync_queue_size", 20)

        self.dataset_root = (
            self.get_parameter("dataset_root").get_parameter_value().string_value
        )
        os.makedirs(self.dataset_root, exist_ok=True)

        self.episode_idx = 0
        self.frame_idx = 0

        self.start_new_episode()

        # Subscribers
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        joint_topic = (
            self.get_parameter("joint_states_topic")
            .get_parameter_value()
            .string_value
        )
        ee_topic = (
            self.get_parameter("ee_pose_topic").get_parameter_value().string_value
        )
        target_topic = (
            self.get_parameter("target_pose_topic")
            .get_parameter_value()
            .string_value
        )
        gripper_topic = (
            self.get_parameter("gripper_topic").get_parameter_value().string_value
        )

        self.image_sub = Subscriber(self, Image, image_topic)
        self.joint_sub = Subscriber(self, JointState, joint_topic)
        self.ee_sub = Subscriber(self, PoseStamped, ee_topic)
        self.target_sub = Subscriber(self, PoseStamped, target_topic)
        self.gripper_sub = Subscriber(self, String, gripper_topic)

        sync_slop = (
            self.get_parameter("sync_slop").get_parameter_value().double_value
        )
        sync_queue_size = (
            self.get_parameter("sync_queue_size")
            .get_parameter_value()
            .integer_value
        )

        self.sync = ApproximateTimeSynchronizer(
            [
                self.image_sub,
                self.joint_sub,
                self.ee_sub,
                self.target_sub,
                self.gripper_sub,
            ],
            queue_size=sync_queue_size,
            slop=sync_slop,
        )

        self.sync.registerCallback(self.callback)

        self.get_logger().info("Dataset recorder started")
        self.get_logger().info(f"Dataset root: {self.dataset_root}")
        self.get_logger().info(
            f"Subscribing to: {image_topic}, {joint_topic}, {ee_topic}, {target_topic}, {gripper_topic}"
        )

    def start_new_episode(self):

        self.episode_idx += 1
        self.frame_idx = 0

        self.episode_dir = os.path.join(
            self.dataset_root, f"episode_{self.episode_idx:06d}"
        )

        os.makedirs(self.episode_dir, exist_ok=True)

        self.json_file = open(
            os.path.join(self.episode_dir, "data.jsonl"), "w", encoding="utf-8"
        )

        self.get_logger().info(f"Started episode {self.episode_idx}")

    def callback(self, img, joints, ee_pose, target_pose, gripper):

        timestamp = time.time()

        # Convert image
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        image_name = f"frame_{self.frame_idx:06d}.png"
        image_path = os.path.join(self.episode_dir, image_name)

        cv2.imwrite(image_path, cv_img)

        obs = {
            "image": image_name,
            "joint_positions": list(joints.position) if joints.position else [],
            "joint_velocities": list(joints.velocity) if joints.velocity else [],
            "ee_pose": [
                ee_pose.pose.position.x,
                ee_pose.pose.position.y,
                ee_pose.pose.position.z,
                ee_pose.pose.orientation.x,
                ee_pose.pose.orientation.y,
                ee_pose.pose.orientation.z,
                ee_pose.pose.orientation.w,
            ],
        }

        action = {
            "target_pose": [
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ],
            "gripper": gripper.data if gripper.data else "0",
        }

        record = {
            "timestamp": timestamp,
            "obs": obs,
            "action": action,
        }

        self.json_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.json_file.flush()

        self.frame_idx += 1

    def destroy_node(self):
        if hasattr(self, "json_file") and self.json_file:
            self.json_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

