#!/usr/bin/env python3
"""Train a tiny tabular Q-learning policy for a PyBullet tool0 reach task."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path

import numpy as np

from rl_reach_env import DEFAULT_URDF, JOINT_COMMANDS, PyBulletReachEnv


DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "outputs" / "rl_reacher_q_policy.json"
DEFAULT_LOG = Path(__file__).resolve().parents[1] / "outputs" / "rl_reacher_training.csv"


def parse_active_joints(value: str) -> tuple[str, ...]:
    joints = tuple(part.strip() for part in value.split(",") if part.strip())
    unknown = [name for name in joints if name not in JOINT_COMMANDS]
    if unknown:
        raise argparse.ArgumentTypeError(f"Unknown joints: {', '.join(unknown)}")
    if not joints:
        raise argparse.ArgumentTypeError("At least one joint is required")
    return joints


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train tabular Q-learning in PyBullet.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--episodes", type=int, default=800, help="Training episodes")
    parser.add_argument("--max-steps", type=int, default=80, help="Steps per episode")
    parser.add_argument("--target-x", type=float, default=30.0, help="Target tool0 X [mm]")
    parser.add_argument("--target-y", type=float, default=-130.0, help="Target tool0 Y [mm]")
    parser.add_argument("--target-z", type=float, default=250.0, help="Target tool0 Z [mm]")
    parser.add_argument("--active-joints", type=parse_active_joints, default=("j1", "j2", "j3"), help="Comma-separated joints")
    parser.add_argument("--joint-step-deg", type=float, default=3.0, help="Action step [deg]")
    parser.add_argument("--state-bin-deg", type=float, default=3.0, help="Q-table angle bin [deg]")
    parser.add_argument("--success-threshold-mm", type=float, default=8.0, help="Success threshold [mm]")
    parser.add_argument("--alpha", type=float, default=0.18, help="Learning rate")
    parser.add_argument("--gamma", type=float, default=0.96, help="Discount factor")
    parser.add_argument("--epsilon-start", type=float, default=0.95, help="Initial exploration rate")
    parser.add_argument("--epsilon-end", type=float, default=0.05, help="Final exploration rate")
    parser.add_argument("--seed", type=int, default=7, help="Random seed")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="Output policy JSON")
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG, help="Output training CSV")
    return parser.parse_args()


def q_values(q_table: dict[tuple[int, ...], np.ndarray], state: tuple[int, ...], action_count: int) -> np.ndarray:
    if state not in q_table:
        q_table[state] = np.zeros(action_count, dtype=np.float64)
    return q_table[state]


def epsilon_for_episode(args: argparse.Namespace, episode: int) -> float:
    if args.episodes <= 1:
        return args.epsilon_end
    fraction = episode / (args.episodes - 1)
    return args.epsilon_end + (args.epsilon_start - args.epsilon_end) * math.exp(-4.0 * fraction)


def main() -> None:
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)

    env = PyBulletReachEnv(
        urdf_path=args.urdf,
        target_mm=(args.target_x, args.target_y, args.target_z),
        active_joints=args.active_joints,
        joint_step_deg=args.joint_step_deg,
        success_threshold_mm=args.success_threshold_mm,
        max_steps=args.max_steps,
        render=False,
    )
    q_table: dict[tuple[int, ...], np.ndarray] = {}
    rows: list[dict[str, object]] = []

    try:
        for episode in range(args.episodes):
            env.reset()
            state = env.discrete_state(args.state_bin_deg)
            epsilon = epsilon_for_episode(args, episode)
            total_reward = 0.0
            final_info: dict[str, object] = {"distance_mm": env.distance_mm()}
            success = False

            for step in range(args.max_steps):
                if random.random() < epsilon:
                    action = random.randrange(env.action_count)
                else:
                    action = int(np.argmax(q_values(q_table, state, env.action_count)))
                result = env.step(action)
                next_state = env.discrete_state(args.state_bin_deg)
                current_q = q_values(q_table, state, env.action_count)
                next_q = q_values(q_table, next_state, env.action_count)
                target = result.reward + args.gamma * float(np.max(next_q)) * (not result.terminated)
                current_q[action] += args.alpha * (target - current_q[action])
                total_reward += result.reward
                state = next_state
                final_info = result.info
                success = result.terminated
                if result.terminated or result.truncated:
                    break

            rows.append(
                {
                    "episode": episode + 1,
                    "steps": step + 1,
                    "reward": f"{total_reward:.6f}",
                    "distance_mm": f"{float(final_info['distance_mm']):.6f}",
                    "success": int(success),
                    "epsilon": f"{epsilon:.6f}",
                }
            )
            if (episode + 1) % max(1, args.episodes // 10) == 0 or episode == 0:
                recent = rows[-min(50, len(rows)) :]
                mean_distance = sum(float(row["distance_mm"]) for row in recent) / len(recent)
                success_rate = sum(int(row["success"]) for row in recent) / len(recent)
                print(
                    f"episode={episode + 1:4d} "
                    f"recent_distance_mm={mean_distance:7.2f} "
                    f"recent_success={success_rate:4.0%} "
                    f"epsilon={epsilon:.3f}"
                )
    finally:
        env.close()

    args.model.parent.mkdir(parents=True, exist_ok=True)
    args.log.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "metadata": {
            "urdf": str(args.urdf),
            "target_mm": [args.target_x, args.target_y, args.target_z],
            "active_joints": list(args.active_joints),
            "joint_step_deg": args.joint_step_deg,
            "state_bin_deg": args.state_bin_deg,
            "success_threshold_mm": args.success_threshold_mm,
            "max_steps": args.max_steps,
            "action_names": env.action_names,
        },
        "q_table": {";".join(map(str, key)): values.tolist() for key, values in q_table.items()},
    }
    args.model.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    with args.log.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["episode", "steps", "reward", "distance_mm", "success", "epsilon"])
        writer.writeheader()
        writer.writerows(rows)

    best_distance = min(float(row["distance_mm"]) for row in rows)
    print(f"saved_model={args.model}")
    print(f"saved_log={args.log}")
    print(f"best_episode_distance_mm={best_distance:.3f}")


if __name__ == "__main__":
    main()
