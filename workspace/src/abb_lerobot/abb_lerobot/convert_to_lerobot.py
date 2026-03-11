#!/usr/bin/env python3
"""
Convert abb_lerobot raw dataset (episode_*/data.jsonl + frame_*.png) to LeRobot v3-compatible layout.

Output layout:
  output_dir/
    data/chunk-000/file-000.parquet   # tabular: index, episode_index, frame_index, timestamp, observation.state, action, task_index
    videos/<camera_key>/chunk-000/file-000.mp4
    meta/
      info.json
      stats.json
      episodes/chunk-000/file-000.parquet
      tasks.jsonl (optional)
"""

import argparse
import json
import os
import re
import sys

import cv2
import pyarrow as pa
import pyarrow.parquet as pq

from abb_lerobot.schema import (
    ACTION_DIM,
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


def _load_episode(episode_path: str):
    """Load one episode: list of dicts with keys from data.jsonl and 'image_path'."""
    data_path = os.path.join(episode_path, "data.jsonl")
    if not os.path.isfile(data_path):
        return []
    records = []
    with open(data_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("schema_version") != SCHEMA_VERSION:
                raise ValueError(
                    f"Unsupported schema_version in {data_path}: {rec.get('schema_version')}"
                )
            obs = rec.get("obs", {})
            image_name = obs.get("image", "")
            rec["image_path"] = os.path.join(episode_path, image_name)
            records.append(rec)
    return records


def _collect_frames(raw_root: str):
    """Collect all frames from raw_root. Yields (episode_index, frame_index, record)."""
    global_index = 0
    for ep_idx, ep_path in _find_episode_dirs(raw_root):
        records = _load_episode(ep_path)
        for frame_idx, rec in enumerate(records):
            rec["_global_index"] = global_index
            rec["_episode_index"] = ep_idx
            rec["_frame_index"] = frame_idx
            global_index += 1
            yield ep_idx, frame_idx, rec


def _build_episode_meta(frames_by_episode):
    """Build episode metadata: list of (episode_index, start_index, end_index)."""
    out = []
    for ep_idx, indices in sorted(frames_by_episode.items()):
        if indices:
            out.append((ep_idx, min(indices), max(indices) + 1))
    return out


def run_convert(
    raw_root: str,
    output_dir: str,
    camera_key: str = "front",
    fps: float = 30.0,
):
    """Convert raw dataset to LeRobot v3 layout under output_dir."""
    os.makedirs(output_dir, exist_ok=True)
    data_dir = os.path.join(output_dir, "data", "chunk-000")
    video_dir = os.path.join(output_dir, "videos", camera_key, "chunk-000")
    meta_dir = os.path.join(output_dir, "meta")
    episodes_meta_dir = os.path.join(meta_dir, "episodes", "chunk-000")
    for d in (data_dir, video_dir, episodes_meta_dir):
        os.makedirs(d, exist_ok=True)

    # Collect all frames and check image paths
    rows = []
    frames_by_episode = {}
    video_frames = []
    width, height = None, None

    for ep_idx, frame_idx, rec in _collect_frames(raw_root):
        obs = rec.get("obs", {})
        action = rec.get("action", {})
        state = obs.get("state")
        target_pose = action.get("target_pose", [])
        gripper = action.get("gripper", 0.0)
        if not isinstance(gripper, (int, float)):
            gripper = 0.0
        action_vec = list(target_pose)[:7]
        while len(action_vec) < 7:
            action_vec.append(0.0)
        action_vec.append(float(gripper))

        if state is None or len(state) != STATE_DIM:
            raise ValueError(
                f"Invalid observation.state in episode {ep_idx} frame {frame_idx}"
            )
        if len(action_vec) != ACTION_DIM:
            raise ValueError(
                f"Invalid action in episode {ep_idx} frame {frame_idx}"
            )

        img_path = rec.get("image_path", "")
        if not os.path.isfile(img_path):
            raise FileNotFoundError(f"Missing image: {img_path}")
        im = cv2.imread(img_path)
        if im is None:
            raise ValueError(f"Could not read image: {img_path}")
        h, w = im.shape[:2]
        if width is None:
            width, height = w, h
        elif (w, h) != (width, height):
            raise ValueError(
                f"Inconsistent image size: {img_path} has {w}x{h}, expected {width}x{height}"
            )
        video_frames.append(im)

        idx = rec["_global_index"]
        rows.append({
            "index": idx,
            "episode_index": ep_idx,
            "frame_index": frame_idx,
            "timestamp": float(rec.get("timestamp", 0.0)),
            "observation.state": state,
            "action": action_vec,
            "task_index": 0,
        })
        if ep_idx not in frames_by_episode:
            frames_by_episode[ep_idx] = []
        frames_by_episode[ep_idx].append(idx)

    if not rows:
        raise ValueError(f"No frames found under {raw_root}")

    # Write data parquet
    table = pa.table({
        "index": [r["index"] for r in rows],
        "episode_index": [r["episode_index"] for r in rows],
        "frame_index": [r["frame_index"] for r in rows],
        "timestamp": pa.array(
            [r["timestamp"] for r in rows],
            type=pa.float32(),
        ),
        "observation.state": pa.array(
            [r["observation.state"] for r in rows],
            type=pa.list_(pa.float32()),
        ),
        "action": pa.array(
            [r["action"] for r in rows],
            type=pa.list_(pa.float32()),
        ),
        "task_index": [r["task_index"] for r in rows],
    })
    pq.write_table(
        table,
        os.path.join(data_dir, "file-000.parquet"),
    )

    # Write video
    video_path = os.path.join(video_dir, "file-000.mp4")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
    for im in video_frames:
        writer.write(im)
    writer.release()

    # Episode metadata parquet
    episode_meta = _build_episode_meta(frames_by_episode)
    episodes_table = pa.table({
        "episode_index": [e[0] for e in episode_meta],
        "start_index": [e[1] for e in episode_meta],
        "end_index": [e[2] for e in episode_meta],
    })
    pq.write_table(
        episodes_table,
        os.path.join(episodes_meta_dir, "file-000.parquet"),
    )

    # meta/info.json
    total_episodes = len(episode_meta)
    total_frames = len(rows)
    info = {
        "codebase_version": "v3.0",
        "robot_type": "abb_egm",
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": 1,
        "total_chunks": 1,
        "fps": fps,
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:06d}.parquet",
        "video_path": f"videos/{camera_key}/chunk-{{chunk_index:03d}}/file-{{file_index:06d}}.mp4",
        "features": {
            "observation.state": {
                "dtype": "float32",
                "shape": [STATE_DIM],
                "names": [
                    "joint_pos_1", "joint_pos_2", "joint_pos_3",
                    "joint_pos_4", "joint_pos_5", "joint_pos_6",
                    "ee_x", "ee_y", "ee_z",
                    "ee_qx", "ee_qy", "ee_qz", "ee_qw",
                    "gripper_state",
                ],
            },
            "action": {
                "dtype": "float32",
                "shape": [ACTION_DIM],
                "names": [
                    "target_x", "target_y", "target_z",
                    "target_qx", "target_qy", "target_qz", "target_qw",
                    "gripper",
                ],
            },
            f"observation.images.{camera_key}": {
                "dtype": "video",
                "shape": [height, width, 3],
                "info": {
                    "video.height": height,
                    "video.width": width,
                    "video.codec": "mp4v",
                    "video.fps": fps,
                    "video.channels": 3,
                    "has_audio": False,
                },
            },
            "timestamp": {"dtype": "float32", "shape": [1], "names": None},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None},
            "index": {"dtype": "int64", "shape": [1], "names": None},
            "task_index": {"dtype": "int64", "shape": [1], "names": None},
        },
    }
    with open(os.path.join(meta_dir, "info.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)

    # meta/stats.json
    try:
        import numpy as np
        state_arr = np.array([r["observation.state"] for r in rows], dtype=np.float32)
        action_arr = np.array([r["action"] for r in rows], dtype=np.float32)
        stats = {
            "observation.state": {
                "min": state_arr.min(axis=0).tolist(),
                "max": state_arr.max(axis=0).tolist(),
                "mean": state_arr.mean(axis=0).tolist(),
                "std": state_arr.std(axis=0).tolist(),
            },
            "action": {
                "min": action_arr.min(axis=0).tolist(),
                "max": action_arr.max(axis=0).tolist(),
                "mean": action_arr.mean(axis=0).tolist(),
                "std": action_arr.std(axis=0).tolist(),
            },
        }
    except ImportError:
        stats = {}
    with open(os.path.join(meta_dir, "stats.json"), "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=2)

    # meta/tasks.jsonl
    with open(os.path.join(meta_dir, "tasks.jsonl"), "w", encoding="utf-8") as f:
        f.write('{"task": "manipulation"}\n')

    return total_episodes, total_frames


def main():
    parser = argparse.ArgumentParser(
        description="Convert abb_lerobot raw dataset to LeRobot v3 format"
    )
    parser.add_argument(
        "input_dir",
        nargs="?",
        default="dataset",
        help="Raw dataset root (episode_*/data.jsonl + frame_*.png)",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default="dataset_lerobot",
        help="Output directory for LeRobot v3 layout",
    )
    parser.add_argument(
        "--camera-key",
        default="front",
        help="Camera key in video path (default: front)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Frames per second for video (default: 30)",
    )
    args = parser.parse_args()

    try:
        n_ep, n_fr = run_convert(
            args.input_dir,
            args.output_dir,
            camera_key=args.camera_key,
            fps=args.fps,
        )
        print(f"Converted {n_ep} episodes, {n_fr} frames -> {args.output_dir}")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
