"""
Minimal LeRobot Robot adapter for abb_pkg/egm_node.cpp.

NO cameras, NO gripper.

Obs (from ROS topics):
  - /joint_states
  - /ee_pose

Action (via action server):
  - /move_to_pose  (abb_pkg/action/MoveToPose)
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np
import torch

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from abb_pkg.action import MoveToPose
from lerobot.robots import Robot, RobotConfig


@RobotConfig.register_subclass("abb_egm_robot")
@dataclass
class AbbEgmRobotConfig(RobotConfig):
    """Minimal LeRobot config for abb_pkg/egm_node.cpp setup."""

    # Topics produced by abb_pkg
    joint_states_topic: str = "/joint_states"
    ee_pose_topic: str = "/ee_pose"

    # Action name exposed by abb_pkg
    action_name: str = "/move_to_pose"

    # Joint ordering for a stable state vector
    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
    )

    # Observation wait
    max_wait_s: float = 0.5

    # Action behavior
    wait_result: bool = False
    action_timeout_s: float = 30.0


class _AbbEgmRosNode(Node):
    """ROS2 helper node: subscribes to state and sends MoveToPose goals."""

    def __init__(self, cfg: AbbEgmRobotConfig):
        super().__init__("abb_egm_lerobot")
        self.cfg = cfg

        self._lock = threading.Lock()
        self._joint_state: Optional[JointState] = None
        self._ee_pose: Optional[PoseStamped] = None

        self._sub_joint = self.create_subscription(
            JointState, cfg.joint_states_topic, self._on_joint, 50
        )
        self._sub_pose = self.create_subscription(
            PoseStamped, cfg.ee_pose_topic, self._on_pose, 50
        )

        self._client = ActionClient(self, MoveToPose, cfg.action_name)

    def _on_joint(self, msg: JointState):
        with self._lock:
            self._joint_state = msg

    def _on_pose(self, msg: PoseStamped):
        with self._lock:
            self._ee_pose = msg

    def snapshot(self) -> tuple[Optional[JointState], Optional[PoseStamped]]:
        with self._lock:
            return self._joint_state, self._ee_pose

    def send_goal(self, xyzquat: np.ndarray, wait_result: bool, timeout_s: float):
        if not self._client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError("move_to_pose action server not available")

        goal = MoveToPose.Goal()
        goal.position.x = float(xyzquat[0])
        goal.position.y = float(xyzquat[1])
        goal.position.z = float(xyzquat[2])
        goal.orientation.x = float(xyzquat[3])
        goal.orientation.y = float(xyzquat[4])
        goal.orientation.z = float(xyzquat[5])
        goal.orientation.w = float(xyzquat[6])

        fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done():
            raise RuntimeError("Timeout while sending goal")

        goal_handle = fut.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal rejected")

        if not wait_result:
            return None

        res_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=float(timeout_s))
        if not res_fut.done():
            goal_handle.cancel_goal_async()
            raise RuntimeError("Action timeout")
        return res_fut.result().result


class AbbEgmRobot(Robot):
    """
    Minimal LeRobot Robot for abb_pkg/egm_node.cpp:

    - Observation: joints + ee pose
      state = [q1..qN, x,y,z, qx,qy,qz,qw]  (no gripper)
    - Action: absolute EE pose (x,y,z,qx,qy,qz,qw) via MoveToPose action
    """

    config_class = AbbEgmRobotConfig
    name = "abb_egm_robot"

    def init(self, config: AbbEgmRobotConfig):
        super().init(config)
        self._node: Optional[_AbbEgmRosNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._is_connected = False

    @property
    def observation_features(self) -> dict:
        n = len(self.config.joint_names)
        # joints (n) + ee_pos(3) + ee_quat(4)
        return {"observation.state": (n + 7,)}

    @property
    def action_features(self) -> dict:
        # [x,y,z,qx,qy,qz,qw]
        return {"action": (7,)}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self) -> None:
        if self._is_connected:
            return
        rclpy.init(args=None)

        self._node = _AbbEgmRosNode(self.config)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._is_connected = True

        def _spin():
            while rclpy.ok() and self._is_connected:
                self._executor.spin_once(timeout_sec=0.05)

        self._spin_thread = threading.Thread(target=_spin, daemon=True)
        self._spin_thread.start()

    def disconnect(self) -> None:
        if not self._is_connected:
            return
        self._is_connected = False
        try:
            if self._executor and self._node:
                self._executor.remove_node(self._node)
            if self._node:
                self._node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _ordered_joint_positions(self, js: JointState) -> np.ndarray:
        if js.name and self.config.joint_names:
            name_to_idx = {n: i for i, n in enumerate(js.name)}
            out = np.zeros((len(self.config.joint_names),), dtype=np.float32)
            for k, jn in enumerate(self.config.joint_names):
                if jn not in name_to_idx:
                    raise KeyError(f"Joint '{jn}' not found in JointState.name")
                out[k] = float(js.position[name_to_idx[jn]])
            return out
        return np.array(js.position, dtype=np.float32)

    def get_observation(self) -> dict[str, Any]:
        if not self._is_connected or self._node is None:
            raise ConnectionError("Robot is not connected")

        t0 = time.time()
        while True:
            js, pose = self._node.snapshot()
            if js is not None and pose is not None:
                break
            if (time.time() - t0) > self.config.max_wait_s:
                raise TimeoutError("Timed out waiting for /joint_states and /ee_pose")
            time.sleep(0.01)

        joint_pos = self._ordered_joint_positions(js)
        ee_pos = np.array(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            dtype=np.float32,
        )
        ee_quat = np.array(
            [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            dtype=np.float32,
        )

        state = np.concatenate([joint_pos, ee_pos, ee_quat], axis=0)
        return {"observation.state": torch.from_numpy(state)}

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        if not self._is_connected or self._node is None:
            raise ConnectionError("Robot is not connected")

        a = action.detach().cpu().numpy().astype(np.float32).reshape(-1)
        if a.shape[0] != 7:
            raise ValueError(f"Expected action shape (7,), got {a.shape}")

        self._node.send_goal(a, self.config.wait_result, self.config.action_timeout_s)
        return action