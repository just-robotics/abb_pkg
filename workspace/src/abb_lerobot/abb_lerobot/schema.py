"""
Canonical schema for abb_lerobot raw dataset (ACT / LeRobot pipeline).

Schema version 1:
- observation.state: flat vector [joint_pos_1..6, ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, ee_qw, gripper_state_numeric]
- action: target_pose (7 floats) + gripper (1 float, numeric)
"""

SCHEMA_VERSION = 1

# State vector layout (observation.state)
NUM_JOINTS = 6
NUM_EE_POSE = 7  # x, y, z, qx, qy, qz, qw
NUM_GRIPPER_STATE = 1
STATE_DIM = NUM_JOINTS + NUM_EE_POSE + NUM_GRIPPER_STATE  # 14

# Action: target_pose (7) + gripper (1)
ACTION_POSE_DIM = 7
ACTION_GRIPPER_DIM = 1
ACTION_DIM = ACTION_POSE_DIM + ACTION_GRIPPER_DIM  # 8

# Gripper: string command -> numeric [0.0, 1.0]
# 0 / close / closed -> 0.0
# 1 / open / opened -> 1.0
# Unknown -> 0.0
GRIPPER_CLOSED_VALUES = ("0", "close", "closed", "grip")
GRIPPER_OPEN_VALUES = ("1", "open", "opened", "release")


def gripper_command_to_numeric(value: str) -> float:
    """Map gripper command string to numeric state. Returns 0.0 (closed) or 1.0 (open)."""
    if not value or not value.strip():
        return 0.0
    s = value.strip().lower()
    if s in GRIPPER_OPEN_VALUES:
        return 1.0
    if s in GRIPPER_CLOSED_VALUES:
        return 0.0
    # Try parse as number
    try:
        x = float(value.strip())
        return 1.0 if x > 0.5 else 0.0
    except ValueError:
        pass
    return 0.0


def build_observation_state(
    joint_positions: list,
    ee_pose: list,
    gripper_numeric: float,
) -> list:
    """
    Build flat observation.state vector of length STATE_DIM.
    joint_positions: up to NUM_JOINTS (padded with 0 if shorter)
    ee_pose: 7 floats [x,y,z,qx,qy,qz,qw]
    gripper_numeric: 0.0 or 1.0
    """
    j = list(joint_positions)[:NUM_JOINTS]
    while len(j) < NUM_JOINTS:
        j.append(0.0)
    ee = list(ee_pose)[:NUM_EE_POSE]
    while len(ee) < NUM_EE_POSE:
        ee.append(0.0)
    return j + ee + [float(gripper_numeric)]
