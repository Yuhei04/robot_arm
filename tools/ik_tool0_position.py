#!/usr/bin/env python3
"""Solve a first position-only IK target for tool0 using PyBullet."""

import argparse
import math
from pathlib import Path

import pybullet as p

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
MOTOR_LIMITS_DEG = read_joint_limits()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Solve position-only IK for tool0.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height above floor/table [mm]")
    parser.add_argument("--link", default="tool0", help="Target link name")
    parser.add_argument("--x", type=float, required=True, help="Target X [mm]")
    parser.add_argument("--y", type=float, required=True, help="Target Y [mm]")
    parser.add_argument("--z", type=float, required=True, help="Target Z [mm]")
    parser.add_argument("--max-iter", type=int, default=300, help="IK iterations")
    parser.add_argument("--threshold-mm", type=float, default=1.0, help="Acceptable position error [mm]")
    parser.add_argument("--current-j1", type=float, default=0.0, help="Seed/current J1 motor angle [deg]")
    parser.add_argument("--current-j2", type=float, default=0.0, help="Seed/current J2 motor angle [deg]")
    parser.add_argument("--current-j3", type=float, default=0.0, help="Seed/current J3 motor angle [deg]")
    parser.add_argument("--current-j4", type=float, default=0.0, help="Seed/current J4 motor angle [deg]")
    parser.add_argument("--current-j5", type=float, default=0.0, help="Seed/current J5 motor angle [deg]")
    parser.add_argument("--current-j6", type=float, default=0.0, help="Seed/current J6 motor angle [deg]")
    return parser.parse_args()


def motor_seed(args: argparse.Namespace) -> dict[str, float]:
    return {name: getattr(args, f"current_{name}") for name in JOINT_COMMANDS}


def motor_to_joint_rad(name: str, motor_deg: float) -> float:
    _, sign = JOINT_COMMANDS[name]
    return math.radians(sign * motor_deg)


def joint_rad_to_motor_deg(name: str, joint_rad: float) -> float:
    _, sign = JOINT_COMMANDS[name]
    return math.degrees(joint_rad) / sign


def clamp_motor(name: str, value: float) -> float:
    lo, hi = MOTOR_LIMITS_DEG[name]
    return min(max(value, lo), hi)


def main() -> None:
    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(urdf_path)

    p.connect(p.DIRECT)
    robot_id = p.loadURDF(str(urdf_path), basePosition=[0, 0, args.base_z_mm / 1000.0], useFixedBase=True)

    joint_indices: dict[str, int] = {}
    link_indices: dict[str, int] = {}
    ordered_joint_names: list[str] = []
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        joint_indices[joint_name] = joint_index
        link_indices[link_name] = joint_index
        if info[2] == p.JOINT_REVOLUTE:
            ordered_joint_names.append(joint_name)

    if args.link not in link_indices:
        available = ", ".join(sorted(link_indices))
        raise ValueError(f"Link {args.link!r} not found. Available: {available}")

    seed = motor_seed(args)
    for name, (joint_name, _) in JOINT_COMMANDS.items():
        p.resetJointState(robot_id, joint_indices[joint_name], motor_to_joint_rad(name, seed[name]))

    target_m = [args.x / 1000.0, args.y / 1000.0, args.z / 1000.0]
    rest_poses = [motor_to_joint_rad(name, seed[name]) for name in JOINT_COMMANDS]
    lower_limits = [math.radians(JOINT_COMMANDS[name][1] * MOTOR_LIMITS_DEG[name][0]) for name in JOINT_COMMANDS]
    upper_limits = [math.radians(JOINT_COMMANDS[name][1] * MOTOR_LIMITS_DEG[name][1]) for name in JOINT_COMMANDS]
    # PyBullet expects lower < upper in joint coordinate space even when motor sign is inverted.
    lower_limits = [min(a, b) for a, b in zip(lower_limits, upper_limits)]
    upper_limits = [max(a, b) for a, b in zip([math.radians(JOINT_COMMANDS[name][1] * MOTOR_LIMITS_DEG[name][0]) for name in JOINT_COMMANDS], [math.radians(JOINT_COMMANDS[name][1] * MOTOR_LIMITS_DEG[name][1]) for name in JOINT_COMMANDS])]
    joint_ranges = [upper_limits[i] - lower_limits[i] for i in range(len(lower_limits))]

    solution = p.calculateInverseKinematics(
        robot_id,
        link_indices[args.link],
        target_m,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=rest_poses,
        maxNumIterations=args.max_iter,
        residualThreshold=args.threshold_mm / 1000.0,
    )

    result: dict[str, float] = {}
    for i, name in enumerate(JOINT_COMMANDS):
        result[name] = clamp_motor(name, joint_rad_to_motor_deg(name, solution[i]))
        joint_name, _ = JOINT_COMMANDS[name]
        p.resetJointState(robot_id, joint_indices[joint_name], motor_to_joint_rad(name, result[name]))

    p.performCollisionDetection()
    state = p.getLinkState(robot_id, link_indices[args.link], computeForwardKinematics=True)
    actual_m = state[4]
    err_mm = [actual_m[i] * 1000.0 - [args.x, args.y, args.z][i] for i in range(3)]
    err_norm = math.sqrt(sum(value * value for value in err_mm))
    rpy = p.getEulerFromQuaternion(state[5])

    print(f"target_mm: x={args.x:.3f}, y={args.y:.3f}, z={args.z:.3f}")
    print("motor_solution_deg: " + ", ".join(f"{name}={result[name]:.3f}" for name in JOINT_COMMANDS))
    print(f"fk_mm: x={actual_m[0] * 1000.0:.3f}, y={actual_m[1] * 1000.0:.3f}, z={actual_m[2] * 1000.0:.3f}")
    print(f"error_mm: dx={err_mm[0]:.3f}, dy={err_mm[1]:.3f}, dz={err_mm[2]:.3f}, norm={err_norm:.3f}")
    print(f"tool0_rpy_deg: roll={math.degrees(rpy[0]):.3f}, pitch={math.degrees(rpy[1]):.3f}, yaw={math.degrees(rpy[2]):.3f}")
    print("status: " + ("OK" if err_norm <= args.threshold_mm else "CHECK_ERROR"))
    p.disconnect()


if __name__ == "__main__":
    main()
