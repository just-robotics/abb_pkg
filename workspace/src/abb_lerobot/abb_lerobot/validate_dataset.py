#!/usr/bin/env python3
"""
Validate abb_lerobot raw dataset (schema version, keys, types, vector lengths, image files).
"""

import argparse
import json
import os
import re
import sys

from abb_lerobot.schema import (
    ACTION_DIM,
    ACTION_POSE_DIM,
    NUM_JOINTS,
    SCHEMA_VERSION,
    STATE_DIM,
)


def _find_episode_dirs(root: str):
    """Yield (episode_index, path) for each episode_XXXXXX under root."""
    if not os.path.isdir(root):
        return
    pattern = re.compile(r"^episode_(\d+)$")
    for name in sorted(os.listdir(root)):
        m = pattern.match(name)
        if m:
            path = os.path.join(root, name)
            if os.path.isdir(path):
                yield int(m.group(1)), path


def validate_dataset(
    dataset_root: str,
    min_episode_length: int = 1,
    check_timestamps: bool = False,
):
    """
    Validate raw dataset. Returns (errors, warnings).
    errors: list of str (must fix)
    warnings: list of str (optional)
    """
    errors = []
    warnings = []

    if not os.path.isdir(dataset_root):
        errors.append(f"Not a directory: {dataset_root}")
        return errors, warnings

    episode_dirs = list(_find_episode_dirs(dataset_root))
    if not episode_dirs:
        warnings.append(f"No episode_* directories found under {dataset_root}")

    for ep_idx, ep_path in episode_dirs:
        data_path = os.path.join(ep_path, "data.jsonl")
        if not os.path.isfile(data_path):
            errors.append(f"Missing {data_path}")
            continue

        with open(data_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        if len(lines) < min_episode_length:
            warnings.append(
                f"Episode {ep_idx}: only {len(lines)} frames (min_episode_length={min_episode_length})"
            )

        prev_ts = None
        for frame_idx, line in enumerate(lines):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError as e:
                errors.append(f"Episode {ep_idx} frame {frame_idx}: invalid JSON: {e}")
                continue

            if rec.get("schema_version") != SCHEMA_VERSION:
                errors.append(
                    f"Episode {ep_idx} frame {frame_idx}: schema_version must be {SCHEMA_VERSION}, got {rec.get('schema_version')}"
                )

            obs = rec.get("obs")
            if obs is None:
                errors.append(f"Episode {ep_idx} frame {frame_idx}: missing 'obs'")
            else:
                if "image" not in obs:
                    errors.append(f"Episode {ep_idx} frame {frame_idx}: obs missing 'image'")
                else:
                    img_path = os.path.join(ep_path, obs["image"])
                    if not os.path.isfile(img_path):
                        errors.append(f"Episode {ep_idx} frame {frame_idx}: image file missing: {img_path}")

                if "state" not in obs:
                    errors.append(f"Episode {ep_idx} frame {frame_idx}: obs missing 'state'")
                else:
                    state = obs["state"]
                    if not isinstance(state, list):
                        errors.append(
                            f"Episode {ep_idx} frame {frame_idx}: obs.state must be list, got {type(state)}"
                        )
                    elif len(state) != STATE_DIM:
                        errors.append(
                            f"Episode {ep_idx} frame {frame_idx}: obs.state length must be {STATE_DIM}, got {len(state)}"
                        )

                if "joint_positions" in obs:
                    jp = obs["joint_positions"]
                    if isinstance(jp, list) and len(jp) != NUM_JOINTS:
                        warnings.append(
                            f"Episode {ep_idx} frame {frame_idx}: obs.joint_positions length {len(jp)} (expected {NUM_JOINTS})"
                        )
                if "ee_pose" in obs:
                    ee = obs["ee_pose"]
                    if isinstance(ee, list) and len(ee) != 7:
                        warnings.append(
                            f"Episode {ep_idx} frame {frame_idx}: obs.ee_pose length {len(ee)} (expected 7)"
                        )

            action = rec.get("action")
            if action is None:
                errors.append(f"Episode {ep_idx} frame {frame_idx}: missing 'action'")
            else:
                if "target_pose" not in action:
                    errors.append(f"Episode {ep_idx} frame {frame_idx}: action missing 'target_pose'")
                else:
                    tp = action["target_pose"]
                    if not isinstance(tp, list) or len(tp) != ACTION_POSE_DIM:
                        errors.append(
                            f"Episode {ep_idx} frame {frame_idx}: action.target_pose must be list of length {ACTION_POSE_DIM}, got {type(tp)} len {len(tp) if isinstance(tp, list) else 'N/A'}"
                        )
                if "gripper" not in action:
                    errors.append(f"Episode {ep_idx} frame {frame_idx}: action missing 'gripper'")
                else:
                    g = action["gripper"]
                    if not isinstance(g, (int, float)):
                        errors.append(
                            f"Episode {ep_idx} frame {frame_idx}: action.gripper must be numeric, got {type(g)}"
                        )
                    elif g not in (0, 0.0, 1, 1.0):
                        warnings.append(
                            f"Episode {ep_idx} frame {frame_idx}: action.gripper={g} (expected 0.0 or 1.0)"
                        )

            if check_timestamps and "timestamp" in rec:
                ts = rec["timestamp"]
                if prev_ts is not None and ts <= prev_ts:
                    warnings.append(
                        f"Episode {ep_idx} frame {frame_idx}: timestamp {ts} not after previous {prev_ts}"
                    )
                prev_ts = ts

    return errors, warnings


def main():
    parser = argparse.ArgumentParser(
        description="Validate abb_lerobot raw dataset"
    )
    parser.add_argument(
        "dataset_root",
        nargs="?",
        default="dataset",
        help="Raw dataset root (episode_*/data.jsonl + frame_*.png)",
    )
    parser.add_argument(
        "--min-episode-length",
        type=int,
        default=1,
        help="Minimum number of frames per episode (default: 1)",
    )
    parser.add_argument(
        "--check-timestamps",
        action="store_true",
        help="Warn if timestamps are not strictly increasing within episode",
    )
    args = parser.parse_args()

    errors, warnings = validate_dataset(
        args.dataset_root,
        min_episode_length=args.min_episode_length,
        check_timestamps=args.check_timestamps,
    )

    for w in warnings:
        print(f"WARNING: {w}", file=sys.stderr)
    for e in errors:
        print(f"ERROR: {e}", file=sys.stderr)

    if errors:
        print(f"Validation FAILED: {len(errors)} error(s), {len(warnings)} warning(s)")
        sys.exit(1)
    if warnings:
        print(f"Validation OK with {len(warnings)} warning(s)")
    else:
        print("Validation OK")
    sys.exit(0)


if __name__ == "__main__":
    main()
