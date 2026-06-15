#!/usr/bin/env python3
"""Gymnasium environment for Stable-Baselines3 PyBullet reach training."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from rl_reach_env import DEFAULT_URDF, PyBulletReachEnv


class RobotArmReachGymEnv(gym.Env):
    """Continuous-action tool0 reaching task backed by PyBullet."""

    metadata = {"render_modes": ["human", None], "render_fps": 25}

    def __init__(
        self,
        urdf_path: str | Path = DEFAULT_URDF,
        target_mm: tuple[float, float, float] = (30.0, -130.0, 250.0),
        active_joints: tuple[str, ...] = ("j1", "j2", "j3"),
        max_joint_step_deg: float = 3.0,
        success_threshold_mm: float = 8.0,
        max_steps: int = 80,
        randomize_start_deg: float = 8.0,
        random_target_radius_mm: float = 0.0,
        render_mode: str | None = None,
        sleep_s: float = 0.0,
    ) -> None:
        super().__init__()
        self.base_target_mm = np.array(target_mm, dtype=np.float64)
        self.active_joints = active_joints
        self.max_joint_step_deg = max_joint_step_deg
        self.randomize_start_deg = randomize_start_deg
        self.random_target_radius_mm = random_target_radius_mm
        self.render_mode = render_mode
        self.env = PyBulletReachEnv(
            urdf_path=Path(urdf_path),
            target_mm=target_mm,
            active_joints=active_joints,
            joint_step_deg=max_joint_step_deg,
            success_threshold_mm=success_threshold_mm,
            max_steps=max_steps,
            render=render_mode == "human",
            sleep_s=sleep_s,
        )
        obs_size = len(active_joints) + 3
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(len(active_joints),), dtype=np.float32)

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        target_mm = self.base_target_mm.copy()
        if self.random_target_radius_mm > 0:
            target_mm += self.np_random.uniform(-self.random_target_radius_mm, self.random_target_radius_mm, size=3)
        self.env.target_mm = target_mm

        seed_angles: dict[str, float] = {}
        for name in self.active_joints:
            lo, hi = self.env.limits_deg[name]
            value = float(self.np_random.uniform(-self.randomize_start_deg, self.randomize_start_deg))
            seed_angles[name] = float(np.clip(value, lo, hi))
        obs = self.env.reset(seed_angles)
        return obs, self._info()

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float64)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        prev_distance = self.env.distance_mm()

        for joint_name, action_value in zip(self.active_joints, action):
            lo, hi = self.env.limits_deg[joint_name]
            self.env.angles_deg[joint_name] = float(
                np.clip(self.env.angles_deg[joint_name] + action_value * self.max_joint_step_deg, lo, hi)
            )
        self.env.steps += 1
        self.env._apply_angles()
        self.env._draw_debug()

        distance = self.env.distance_mm()
        improvement = prev_distance - distance
        action_cost = 0.02 * float(np.linalg.norm(action, ord=2))
        reward = improvement * 0.10 - distance * 0.012 - action_cost
        terminated = distance <= self.env.success_threshold_mm
        truncated = self.env.steps >= self.env.max_steps
        if terminated:
            reward += 12.0
        if self.env.sleep_s > 0:
            import time

            time.sleep(self.env.sleep_s)
        obs = self.env.observation()
        info = self._info(distance)
        info["action_norm"] = float(np.linalg.norm(action, ord=2))
        return obs, float(reward), terminated, truncated, info

    def close(self) -> None:
        self.env.close()

    def render(self) -> None:
        return None

    def _info(self, distance_mm: float | None = None) -> dict[str, Any]:
        distance = self.env.distance_mm() if distance_mm is None else distance_mm
        return {
            "distance_mm": float(distance),
            "is_success": bool(distance <= self.env.success_threshold_mm),
            "target_mm": tuple(float(v) for v in self.env.target_mm),
            "tool_mm": tuple(float(v) for v in self.env.tool_position_mm()),
            "angles_deg": self.env.angles_deg.copy(),
        }


def parse_active_joints(value: str) -> tuple[str, ...]:
    joints = tuple(part.strip() for part in value.split(",") if part.strip())
    if not joints:
        raise ValueError("At least one active joint is required")
    return joints


def target_from_args(args: Any) -> tuple[float, float, float]:
    return (float(args.target_x), float(args.target_y), float(args.target_z))
