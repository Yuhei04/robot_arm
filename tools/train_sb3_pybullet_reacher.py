#!/usr/bin/env python3
"""Train a Stable-Baselines3 PPO policy for the PyBullet reach task."""

from __future__ import annotations

import argparse
from pathlib import Path

from gymnasium.wrappers import TimeLimit
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from plot_sb3_training import plot_monitor_log
from rl_reach_env import DEFAULT_URDF
from sb3_reach_env import RobotArmReachGymEnv, parse_active_joints, target_from_args


DEFAULT_RUN_DIR = Path(__file__).resolve().parents[1] / "outputs" / "sb3_reacher"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train Stable-Baselines3 PPO on the PyBullet reach task.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--total-timesteps", type=int, default=20000, help="PPO training timesteps")
    parser.add_argument("--target-x", type=float, default=30.0, help="Target tool0 X [mm]")
    parser.add_argument("--target-y", type=float, default=-130.0, help="Target tool0 Y [mm]")
    parser.add_argument("--target-z", type=float, default=250.0, help="Target tool0 Z [mm]")
    parser.add_argument("--active-joints", type=parse_active_joints, default=("j1", "j2", "j3"), help="Comma-separated joints")
    parser.add_argument("--max-joint-step-deg", type=float, default=3.0, help="Maximum joint delta per action [deg]")
    parser.add_argument("--success-threshold-mm", type=float, default=8.0, help="Success threshold [mm]")
    parser.add_argument("--max-steps", type=int, default=80, help="Episode step limit")
    parser.add_argument("--randomize-start-deg", type=float, default=8.0, help="Initial joint randomization [deg]")
    parser.add_argument("--random-target-radius-mm", type=float, default=0.0, help="Uniform target randomization radius [mm]")
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN_DIR, help="Output directory")
    parser.add_argument("--seed", type=int, default=7, help="Random seed")
    parser.add_argument("--check-env", action="store_true", help="Run Gymnasium/SB3 environment checker before training")
    return parser.parse_args()


def make_env(args: argparse.Namespace, monitor_path: Path | None = None) -> Monitor:
    env = RobotArmReachGymEnv(
        urdf_path=args.urdf,
        target_mm=target_from_args(args),
        active_joints=args.active_joints,
        max_joint_step_deg=args.max_joint_step_deg,
        success_threshold_mm=args.success_threshold_mm,
        max_steps=args.max_steps,
        randomize_start_deg=args.randomize_start_deg,
        random_target_radius_mm=args.random_target_radius_mm,
    )
    env = TimeLimit(env, max_episode_steps=args.max_steps)
    return Monitor(env, filename=str(monitor_path) if monitor_path else None, info_keywords=("distance_mm", "is_success"))


def main() -> None:
    args = parse_args()
    args.run_dir.mkdir(parents=True, exist_ok=True)
    monitor_path = args.run_dir / "monitor.csv"
    model_path = args.run_dir / "ppo_reacher.zip"
    plot_path = args.run_dir / "training_curves.png"

    if args.check_env:
        check_env(make_env(args), warn=True)

    vec_env = DummyVecEnv([lambda: make_env(args, monitor_path)])
    checkpoint = CheckpointCallback(save_freq=max(1000, args.total_timesteps // 5), save_path=str(args.run_dir), name_prefix="ppo_reacher")
    model = PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=3e-4,
        n_steps=1024,
        batch_size=128,
        gamma=0.96,
        gae_lambda=0.92,
        ent_coef=0.01,
        verbose=1,
        seed=args.seed,
        device="cpu",
    )
    model.learn(total_timesteps=args.total_timesteps, callback=checkpoint, progress_bar=False)
    model.save(model_path)
    vec_env.close()

    plot_monitor_log(monitor_path, plot_path)
    print(f"saved_model={model_path}")
    print(f"monitor_log={monitor_path}")
    print(f"training_plot={plot_path}")


if __name__ == "__main__":
    main()
