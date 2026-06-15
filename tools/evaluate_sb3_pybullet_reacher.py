#!/usr/bin/env python3
"""Evaluate or replay a Stable-Baselines3 reach policy."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from stable_baselines3 import PPO

from rl_reach_env import DEFAULT_URDF
from sb3_reach_env import RobotArmReachGymEnv, parse_active_joints, target_from_args
from train_sb3_pybullet_reacher import DEFAULT_RUN_DIR


DEFAULT_TRAJECTORY = DEFAULT_RUN_DIR / "eval_trajectory.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a Stable-Baselines3 PPO reach policy.")
    parser.add_argument("--model", type=Path, default=DEFAULT_RUN_DIR / "ppo_reacher.zip", help="Model zip")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--target-x", type=float, default=30.0, help="Target tool0 X [mm]")
    parser.add_argument("--target-y", type=float, default=-130.0, help="Target tool0 Y [mm]")
    parser.add_argument("--target-z", type=float, default=250.0, help="Target tool0 Z [mm]")
    parser.add_argument("--active-joints", type=parse_active_joints, default=("j1", "j2", "j3"), help="Comma-separated joints")
    parser.add_argument("--max-joint-step-deg", type=float, default=3.0, help="Maximum joint delta per action [deg]")
    parser.add_argument("--success-threshold-mm", type=float, default=8.0, help="Success threshold [mm]")
    parser.add_argument("--max-steps", type=int, default=80, help="Episode step limit")
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY, help="Output trajectory CSV")
    parser.add_argument("--render", action="store_true", help="Replay in PyBullet GUI")
    parser.add_argument("--sleep-s", type=float, default=0.04, help="Delay between GUI steps")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model = PPO.load(args.model, device="cpu")
    env = RobotArmReachGymEnv(
        urdf_path=args.urdf,
        target_mm=target_from_args(args),
        active_joints=args.active_joints,
        max_joint_step_deg=args.max_joint_step_deg,
        success_threshold_mm=args.success_threshold_mm,
        max_steps=args.max_steps,
        randomize_start_deg=0.0,
        render_mode="human" if args.render else None,
        sleep_s=args.sleep_s if args.render else 0.0,
    )
    rows: list[dict[str, object]] = []
    try:
        obs, info = env.reset(seed=0)
        rows.append({"step": 0, "reward": "", "distance_mm": f"{info['distance_mm']:.6f}", **env.env.angles_deg})
        total_reward = 0.0
        for step in range(args.max_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            rows.append(
                {
                    "step": step + 1,
                    "reward": f"{reward:.6f}",
                    "distance_mm": f"{info['distance_mm']:.6f}",
                    **{name: f"{value:.6f}" for name, value in env.env.angles_deg.items()},
                }
            )
            if terminated or truncated:
                break
    finally:
        env.close()

    args.trajectory.parent.mkdir(parents=True, exist_ok=True)
    with args.trajectory.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["step", "reward", "distance_mm", "j1", "j2", "j3", "j4", "j5", "j6"])
        writer.writeheader()
        writer.writerows(rows)

    final_distance = float(rows[-1]["distance_mm"])
    print(f"steps={rows[-1]['step']}")
    print(f"total_reward={total_reward:.3f}")
    print(f"final_distance_mm={final_distance:.3f}")
    print(f"trajectory={args.trajectory}")


if __name__ == "__main__":
    main()
