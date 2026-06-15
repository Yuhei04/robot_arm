#!/usr/bin/env python3
"""Generate a small PyBullet FK table for tool0 check poses."""

import argparse
import csv
import math
from pathlib import Path

import pybullet as p


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_fusion.urdf"
DEFAULT_BASE_Z_MM = 10.25
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "outputs" / "tool0_pose_table.csv"
JOINT_COMMANDS = {
    "j1": ("base_yaw_joint", 1.0),
    "j2": ("shoulder_pitch_joint", 1.0),
    "j3": ("elbow_pitch_joint", -1.0),
    "j4": ("elbow_roll_joint", 1.0),
    "j5": ("wrist_pitch_joint", -1.0),
    "j6": ("wrist_roll_joint", -1.0),
}
CHECK_POSES = [
    ("all_zero", dict(j1=0, j2=0, j3=0, j4=0, j5=0, j6=0)),
    ("j1_plus20", dict(j1=20, j2=0, j3=0, j4=0, j5=0, j6=0)),
    ("j2_plus20", dict(j1=0, j2=20, j3=0, j4=0, j5=0, j6=0)),
    ("j3_plus20", dict(j1=0, j2=0, j3=20, j4=0, j5=0, j6=0)),
    ("j4_plus20", dict(j1=0, j2=0, j3=0, j4=20, j5=0, j6=0)),
    ("j5_plus20", dict(j1=0, j2=0, j3=0, j4=0, j5=20, j6=0)),
    ("j6_plus20", dict(j1=0, j2=0, j3=0, j4=0, j5=0, j6=20)),
    ("mixed_small", dict(j1=20, j2=20, j3=20, j4=10, j5=10, j6=10)),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write a tool0 FK check table.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height above the floor/table [mm]")
    parser.add_argument("--link", default="tool0", help="Link name to report")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="CSV output path")
    parser.add_argument("--z-deflection-mm", type=float, default=0.0, help="Expected real-arm downward Z deflection to subtract in an extra column [mm]")
    return parser.parse_args()


def reset_pose(robot_id: int, joint_indices: dict[str, int], pose: dict[str, float]) -> None:
    for arg_name, (joint_name, sign) in JOINT_COMMANDS.items():
        joint_index = joint_indices.get(joint_name)
        if joint_index is None:
            continue
        p.resetJointState(robot_id, joint_index, math.radians(sign * pose[arg_name]))


def main() -> None:
    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(urdf_path)

    p.connect(p.DIRECT)
    robot_id = p.loadURDF(str(urdf_path), basePosition=[0, 0, args.base_z_mm / 1000.0], useFixedBase=True)

    joint_indices: dict[str, int] = {}
    link_indices: dict[str, int] = {}
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_indices[info[1].decode("utf-8")] = joint_index
        link_indices[info[12].decode("utf-8")] = joint_index

    if args.link not in link_indices:
        available = ", ".join(sorted(link_indices))
        raise ValueError(f"Link {args.link!r} not found. Available: {available}")

    rows = []
    for name, pose in CHECK_POSES:
        reset_pose(robot_id, joint_indices, pose)
        p.performCollisionDetection()
        state = p.getLinkState(robot_id, link_indices[args.link], computeForwardKinematics=True)
        pos_m = state[4]
        rpy = p.getEulerFromQuaternion(state[5])
        rows.append(
            {
                "pose": name,
                **{joint: f"{pose[joint]:.3f}" for joint in JOINT_COMMANDS},
                "x_mm": f"{pos_m[0] * 1000.0:.3f}",
                "y_mm": f"{pos_m[1] * 1000.0:.3f}",
                "z_mm": f"{pos_m[2] * 1000.0:.3f}",
                "real_expected_z_mm": f"{pos_m[2] * 1000.0 - args.z_deflection_mm:.3f}",
                "roll_deg": f"{math.degrees(rpy[0]):.3f}",
                "pitch_deg": f"{math.degrees(rpy[1]):.3f}",
                "yaw_deg": f"{math.degrees(rpy[2]):.3f}",
            }
        )

    p.disconnect()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["pose", *JOINT_COMMANDS.keys(), "x_mm", "y_mm", "z_mm", "real_expected_z_mm", "roll_deg", "pitch_deg", "yaw_deg"]
    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(output_path)
    for row in rows:
        print(
            f"{row['pose']}: xyz=({row['x_mm']}, {row['y_mm']}, {row['z_mm']}) mm, "
            f"rpy=({row['roll_deg']}, {row['pitch_deg']}, {row['yaw_deg']}) deg"
        )


if __name__ == "__main__":
    main()
