#!/usr/bin/env python3
"""Small PyBullet reaching environment for the robot arm.

This module intentionally avoids Gymnasium/Stable-Baselines dependencies so the
first RL loop can run in the lightweight PyBullet venv already used here.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

from joint_limits import read_joint_limits


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_fusion.urdf"
DEFAULT_BASE_Z_MM = 10.25
JOINT_COMMANDS = {
    "j1": ("base_yaw_joint", 1.0),
    "j2": ("shoulder_pitch_joint", 1.0),
    "j3": ("elbow_pitch_joint", -1.0),
    "j4": ("elbow_roll_joint", 1.0),
    "j5": ("wrist_pitch_joint", -1.0),
    "j6": ("wrist_roll_joint", -1.0),
}


@dataclass(frozen=True)
class StepResult:
    observation: np.ndarray
    reward: float
    terminated: bool
    truncated: bool
    info: dict[str, object]


def motor_to_joint_rad(name: str, motor_deg: float) -> float:
    _, sign = JOINT_COMMANDS[name]
    return math.radians(sign * motor_deg)


def joint_rad_to_motor_deg(name: str, joint_rad: float) -> float:
    _, sign = JOINT_COMMANDS[name]
    return math.degrees(joint_rad) / sign


class PyBulletReachEnv:
    """Position-only reach task with discrete joint nudge actions."""

    def __init__(
        self,
        urdf_path: Path = DEFAULT_URDF,
        target_mm: tuple[float, float, float] = (30.0, -130.0, 250.0),
        active_joints: tuple[str, ...] = ("j1", "j2", "j3"),
        joint_step_deg: float = 3.0,
        success_threshold_mm: float = 8.0,
        max_steps: int = 80,
        base_z_mm: float = DEFAULT_BASE_Z_MM,
        render: bool = False,
        sleep_s: float = 0.0,
    ) -> None:
        self.urdf_path = urdf_path.expanduser().resolve()
        self.target_mm = np.array(target_mm, dtype=np.float64)
        self.active_joints = active_joints
        self.joint_step_deg = joint_step_deg
        self.success_threshold_mm = success_threshold_mm
        self.max_steps = max_steps
        self.base_z_mm = base_z_mm
        self.render = render
        self.sleep_s = sleep_s
        self.limits_deg = read_joint_limits()
        self.client_id = -1
        self.robot_id = -1
        self.joint_indices: dict[str, int] = {}
        self.link_indices: dict[str, int] = {}
        self.tool_link = "tool0"
        self.steps = 0
        self.angles_deg = {name: 0.0 for name in JOINT_COMMANDS}
        self.prev_distance_mm = float("inf")
        self.action_names = [f"{name}{sign:+d}" for name in active_joints for sign in (-1, 1)]

    @property
    def action_count(self) -> int:
        return len(self.action_names)

    def connect(self) -> None:
        if self.client_id >= 0 and p.isConnected(self.client_id):
            return
        mode = p.GUI if self.render else p.DIRECT
        self.client_id = p.connect(mode)
        if self.client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet")
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.resetSimulation(physicsClientId=self.client_id)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        if self.render:
            p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        if not self.urdf_path.exists():
            raise FileNotFoundError(self.urdf_path)
        self.robot_id = p.loadURDF(
            str(self.urdf_path),
            basePosition=[0, 0, self.base_z_mm / 1000.0],
            useFixedBase=True,
            physicsClientId=self.client_id,
        )
        self.joint_indices.clear()
        self.link_indices.clear()
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            self.joint_indices[info[1].decode("utf-8")] = joint_index
            self.link_indices[info[12].decode("utf-8")] = joint_index
        if self.tool_link not in self.link_indices:
            available = ", ".join(sorted(self.link_indices))
            raise ValueError(f"Link {self.tool_link!r} not found. Available: {available}")
        if self.render:
            p.resetDebugVisualizerCamera(
                cameraDistance=0.55,
                cameraYaw=50,
                cameraPitch=-25,
                cameraTargetPosition=[0.03, -0.08, 0.18],
                physicsClientId=self.client_id,
            )

    def close(self) -> None:
        if self.client_id >= 0 and p.isConnected(self.client_id):
            p.disconnect(self.client_id)
        self.client_id = -1

    def reset(self, seed_angles_deg: dict[str, float] | None = None) -> np.ndarray:
        self.connect()
        self.steps = 0
        seed_angles_deg = seed_angles_deg or {}
        for name in JOINT_COMMANDS:
            lo, hi = self.limits_deg[name]
            self.angles_deg[name] = float(np.clip(seed_angles_deg.get(name, 0.0), lo, hi))
        self._apply_angles()
        self.prev_distance_mm = self.distance_mm()
        self._draw_debug()
        return self.observation()

    def step(self, action: int) -> StepResult:
        joint_name = self.active_joints[action // 2]
        direction = -1.0 if action % 2 == 0 else 1.0
        lo, hi = self.limits_deg[joint_name]
        self.angles_deg[joint_name] = float(
            np.clip(self.angles_deg[joint_name] + direction * self.joint_step_deg, lo, hi)
        )
        self.steps += 1
        self._apply_angles()
        distance = self.distance_mm()
        improvement = self.prev_distance_mm - distance
        action_cost = 0.01 * abs(direction)
        reward = improvement * 0.08 - distance * 0.01 - action_cost
        terminated = distance <= self.success_threshold_mm
        truncated = self.steps >= self.max_steps
        if terminated:
            reward += 10.0
        self.prev_distance_mm = distance
        self._draw_debug()
        if self.sleep_s > 0:
            time.sleep(self.sleep_s)
        return StepResult(
            observation=self.observation(),
            reward=float(reward),
            terminated=terminated,
            truncated=truncated,
            info={
                "distance_mm": distance,
                "tool_mm": tuple(float(v) for v in self.tool_position_mm()),
                "angles_deg": self.angles_deg.copy(),
            },
        )

    def observation(self) -> np.ndarray:
        angles = np.array([self.angles_deg[name] for name in self.active_joints], dtype=np.float64)
        tool_delta = (self.target_mm - self.tool_position_mm()) / 100.0
        return np.concatenate([angles / 90.0, tool_delta]).astype(np.float32)

    def discrete_state(self, angle_bin_deg: float) -> tuple[int, ...]:
        return tuple(int(round(self.angles_deg[name] / angle_bin_deg)) for name in self.active_joints)

    def distance_mm(self) -> float:
        return float(np.linalg.norm(self.tool_position_mm() - self.target_mm))

    def tool_position_mm(self) -> np.ndarray:
        state = p.getLinkState(
            self.robot_id,
            self.link_indices[self.tool_link],
            computeForwardKinematics=True,
            physicsClientId=self.client_id,
        )
        return np.array(state[4], dtype=np.float64) * 1000.0

    def _apply_angles(self) -> None:
        for name, (joint_name, _) in JOINT_COMMANDS.items():
            p.resetJointState(
                self.robot_id,
                self.joint_indices[joint_name],
                motor_to_joint_rad(name, self.angles_deg[name]),
                physicsClientId=self.client_id,
            )
        p.performCollisionDetection(physicsClientId=self.client_id)

    def _draw_debug(self) -> None:
        if not self.render:
            return
        target_m = (self.target_mm / 1000.0).tolist()
        p.addUserDebugText("target", target_m, [1, 0.1, 0.1], 1.0, 0.05, physicsClientId=self.client_id)
        p.addUserDebugLine(
            (self.tool_position_mm() / 1000.0).astype(float).tolist(),
            (self.target_mm / 1000.0).astype(float).tolist(),
            [1, 0.2, 0.2],
            2,
            0.05,
            physicsClientId=self.client_id,
        )
