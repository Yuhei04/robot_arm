#!/usr/bin/env python3
"""Print tool0 pose from the Fusion URDF using PyBullet FK."""

import argparse
import math
from pathlib import Path

import pybullet as p


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print tool0 FK pose from motor angles.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height above the floor/table [mm]")
    parser.add_argument("--link", default="tool0", help="Link name to report")
    for joint in JOINT_COMMANDS:
        parser.add_argument(f"--{joint}", type=float, default=0.0, help=f"ID {joint[1]} motor angle [deg]")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(urdf_path)

    p.connect(p.DIRECT)
    robot_id = p.loadURDF(str(urdf_path), basePosition=[0, 0, args.base_z_mm / 1000.0], useFixedBase=True)

    joint_indices = {}
    link_indices = {}
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_indices[info[1].decode("utf-8")] = joint_index
        link_indices[info[12].decode("utf-8")] = joint_index

    for arg_name, (joint_name, sign) in JOINT_COMMANDS.items():
        if joint_name not in joint_indices:
            continue
        motor_deg = getattr(args, arg_name)
        p.resetJointState(robot_id, joint_indices[joint_name], math.radians(sign * motor_deg))

    p.performCollisionDetection()
    if args.link not in link_indices:
        available = ", ".join(sorted(link_indices))
        raise ValueError(f"Link {args.link!r} not found. Available: {available}")

    state = p.getLinkState(robot_id, link_indices[args.link], computeForwardKinematics=True)
    pos_m = state[4]
    quat = state[5]
    rpy = p.getEulerFromQuaternion(quat)

    print(f"URDF: {urdf_path}")
    print(f"link: {args.link}")
    print(
        "motor_deg: "
        + ", ".join(f"{name}={getattr(args, name):.2f}" for name in JOINT_COMMANDS)
    )
    print(f"x_mm: {pos_m[0] * 1000.0:.3f}")
    print(f"y_mm: {pos_m[1] * 1000.0:.3f}")
    print(f"z_mm: {pos_m[2] * 1000.0:.3f}")
    print(f"roll_deg: {math.degrees(rpy[0]):.3f}")
    print(f"pitch_deg: {math.degrees(rpy[1]):.3f}")
    print(f"yaw_deg: {math.degrees(rpy[2]):.3f}")
    p.disconnect()


if __name__ == "__main__":
    main()
