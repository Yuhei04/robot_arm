#!/usr/bin/env python3
"""Evaluate or replay a trained tabular Q policy in PyBullet."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from rl_reach_env import DEFAULT_URDF, PyBulletReachEnv
from train_pybullet_reacher_q import DEFAULT_MODEL


DEFAULT_TRAJECTORY = Path(__file__).resolve().parents[1] / "outputs" / "rl_reacher_eval_trajectory.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a saved PyBullet Q-learning policy.")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="Policy JSON")
    parser.add_argument("--urdf", type=Path, default=None, help="Override URDF path")
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY, help="Output trajectory CSV")
    parser.add_argument("--render", action="store_true", help="Replay in PyBullet GUI")
    parser.add_argument("--sleep-s", type=float, default=0.04, help="Delay between rendered steps")
    return parser.parse_args()


def load_policy(path: Path) -> tuple[dict[str, object], dict[tuple[int, ...], np.ndarray]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    metadata = payload["metadata"]
    q_table = {
        tuple(int(part) for part in key.split(";")): np.array(values, dtype=np.float64)
        for key, values in payload["q_table"].items()
    }
    return metadata, q_table


def main() -> None:
    args = parse_args()
    metadata, q_table = load_policy(args.model)
    urdf_path = args.urdf or Path(str(metadata.get("urdf") or DEFAULT_URDF))
    env = PyBulletReachEnv(
        urdf_path=urdf_path,
        target_mm=tuple(float(v) for v in metadata["target_mm"]),
        active_joints=tuple(str(v) for v in metadata["active_joints"]),
        joint_step_deg=float(metadata["joint_step_deg"]),
        success_threshold_mm=float(metadata["success_threshold_mm"]),
        max_steps=int(metadata["max_steps"]),
        render=args.render,
        sleep_s=args.sleep_s if args.render else 0.0,
    )
    rows: list[dict[str, object]] = []
    try:
        env.reset()
        rows.append({"step": 0, "action": "reset", "distance_mm": f"{env.distance_mm():.6f}", **env.angles_deg})
        for step in range(env.max_steps):
            state = env.discrete_state(float(metadata["state_bin_deg"]))
            values = q_table.get(state, np.zeros(env.action_count, dtype=np.float64))
            action = int(np.argmax(values))
            result = env.step(action)
            rows.append(
                {
                    "step": step + 1,
                    "action": env.action_names[action],
                    "distance_mm": f"{float(result.info['distance_mm']):.6f}",
                    **{name: f"{value:.6f}" for name, value in result.info["angles_deg"].items()},
                }
            )
            if result.terminated or result.truncated:
                break
    finally:
        env.close()

    args.trajectory.parent.mkdir(parents=True, exist_ok=True)
    with args.trajectory.open("w", newline="", encoding="utf-8") as f:
        fieldnames = ["step", "action", "distance_mm", "j1", "j2", "j3", "j4", "j5", "j6"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    final = rows[-1]
    print(f"steps={final['step']}")
    print(f"final_distance_mm={float(final['distance_mm']):.3f}")
    print(f"trajectory={args.trajectory}")


if __name__ == "__main__":
    main()
