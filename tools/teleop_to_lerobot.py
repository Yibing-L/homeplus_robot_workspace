#!/usr/bin/env python3
"""Convert console teleop episodes (homeplus_console/recorder.py) into a LeRobot dataset.

Run it in an environment that has `lerobot` installed (the console machine doesn't need it):

    pip install lerobot            # in its own venv
    python3 tools/teleop_to_lerobot.py ~/.homeplus_console/teleop_episodes \
        --repo-id local/homeplus_teleop --root ~/datasets/homeplus_teleop [--only-success]

Schema (one row per recorded sample, fps from the episodes, default 10):
    observation.state         float32[8]  measured joints, SI (m, rad):
                              base_x, base_y, base_theta, grip, hand, wrist, elbow, shoulder
    action                    float32[8]  --base-action velocity (default):
                              [base_vx, base_vy, base_wz (body frame, m/s, rad/s), grip, hand, wrist, elbow, shoulder]
                              — arm/gripper are absolute commanded setpoints, the base is a velocity
                              (odometry x/y/theta are not meaningful across episodes)
                              --base-action absolute: the commanded joint setpoint, same order as the state
    observation.images.hand   D405 hand camera (video)
    observation.images.front  D455 overview (video), when recorded
    task                      the language instruction typed in the console

Samples taken while no stream was active (action null) use the measured arm pose and zero base
velocity as the action ("hold"). Written against LeRobotDataset.create / add_frame / save_episode
(lerobot >= 0.2, dataset format v2.x); older APIs that take the task in save_episode are handled too.
Not tested here (no lerobot on the robot machine): check the first converted episode with
`python -m lerobot.scripts.visualize_dataset --repo-id ... --root ... --episode-index 0`.
"""
import argparse
import inspect
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ARM = slice(3, 8)


def load_episodes(paths, only_success):
    eps = []
    for p in paths:
        p = Path(p).expanduser()
        for d in sorted([p] if (p / "meta.json").exists() else [q for q in p.iterdir() if (q / "meta.json").exists()]):
            meta = json.loads((d / "meta.json").read_text())
            if meta.get("outcome") in (None, "aborted", "discard"):
                print(f"skip {d.name}: outcome {meta.get('outcome')}")
                continue
            if only_success and meta.get("outcome") != "success":
                print(f"skip {d.name}: outcome {meta.get('outcome')}")
                continue
            rows = [json.loads(line) for line in (d / "steps.jsonl").read_text().splitlines() if line.strip()]
            eps.append((d, meta, rows))
    return eps


def image(d, rel, shape):
    if rel and (d / rel).exists():
        return np.asarray(Image.open(d / rel).convert("RGB"))
    return np.zeros(shape, dtype=np.uint8)   # missing frame (writer dropped it): black


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episodes", nargs="+", help="episode directories or their parent (teleop_episodes)")
    ap.add_argument("--repo-id", required=True)
    ap.add_argument("--root", required=True)
    ap.add_argument("--only-success", action="store_true")
    ap.add_argument("--base-action", choices=["velocity", "absolute"], default="velocity")
    args = ap.parse_args()
    try:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError:
            sys.exit("lerobot is not installed in this environment (pip install lerobot)")

    eps = load_episodes(args.episodes, args.only_success)
    if not eps:
        sys.exit("no usable episodes")
    fps = int(round(eps[0][1].get("fps", 10)))
    cams = {}
    for d, meta, rows in eps:
        for r in rows:
            for cam, rel in (r.get("images") or {}).items():
                if rel and cam not in cams and (d / rel).exists():
                    cams[cam] = np.asarray(Image.open(d / rel)).shape
    joint_names = eps[0][1]["joint_names"]
    act_names = (["base_vx", "base_vy", "base_wz"] + joint_names[3:]) if args.base_action == "velocity" else joint_names
    features = {
        "observation.state": {"dtype": "float32", "shape": (8,), "names": joint_names},
        "action": {"dtype": "float32", "shape": (8,), "names": act_names},
    }
    for cam, shape in cams.items():
        features[f"observation.images.{cam}"] = {"dtype": "video", "shape": shape, "names": ["height", "width", "channels"]}
    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=fps, root=args.root, robot_type="homeplus",
                               features=features, use_videos=True)
    takes_task_in_frame = "task" in inspect.signature(ds.add_frame).parameters or \
        "task" not in inspect.signature(ds.save_episode).parameters
    for d, meta, rows in eps:
        task = meta["instruction"]
        for r in rows:
            state = np.asarray(r["state"], dtype=np.float32)
            cmd = np.asarray(r["action"], dtype=np.float32) if r.get("action") else state.copy()
            if args.base_action == "velocity":
                bv = np.asarray(r.get("base_vel") or [0.0, 0.0, 0.0], dtype=np.float32) if r.get("action") else np.zeros(3, np.float32)
                action = np.concatenate([bv, cmd[ARM]])
            else:
                action = cmd
            frame = {"observation.state": state, "action": action.astype(np.float32)}
            for cam, shape in cams.items():
                frame[f"observation.images.{cam}"] = image(d, (r.get("images") or {}).get(cam), shape)
            if takes_task_in_frame:
                frame["task"] = task
            ds.add_frame(frame)
        if takes_task_in_frame:
            ds.save_episode()
        else:
            ds.save_episode(task=task)
        print(f"{d.name}: {len(rows)} frames, task {task!r}")
    if hasattr(ds, "finalize"):
        ds.finalize()
    print(f"dataset written to {args.root} ({len(eps)} episodes, {fps} fps, cameras {list(cams)})")


if __name__ == "__main__":
    main()
