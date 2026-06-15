#!/usr/bin/env python3
"""Replay a taught joint-pose CSV in the PyBullet GUI."""

import argparse
import csv
import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data

from move_tool0_points import JOINT_COMMANDS, motor_to_joint_rad


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_fusion.urdf"
DEFAULT_POSES = Path(__file__).resolve().parents[1] / "outputs" / "taught_poses.csv"
DEFAULT_BASE_Z_MM = 10.25


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View a taught joint pose sequence in PyBullet.")
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSES, help="Pose CSV with name,j1..j6 columns")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height [mm]")
    parser.add_argument("--row-delay", type=float, default=1.0, help="Delay between rows [s]")
    parser.add_argument("--loop", action="store_true", help="Loop until the PyBullet window closes")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"No poses in {path}")
    return rows


def setup_robot(args: argparse.Namespace) -> tuple[int, dict[str, int], dict[str, int]]:
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(cameraDistance=0.65, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=[0, -0.08, 0.18])
    robot_id = p.loadURDF(str(args.urdf.expanduser().resolve()), basePosition=[0, 0, args.base_z_mm / 1000.0], useFixedBase=True)
    joint_indices = {}
    link_indices = {}
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_indices[info[1].decode("utf-8")] = joint_index
        link_indices[info[12].decode("utf-8")] = joint_index
    return robot_id, joint_indices, link_indices


def apply_pose(robot_id: int, joint_indices: dict[str, int], row: dict[str, str]) -> None:
    for joint, (joint_name, _) in JOINT_COMMANDS.items():
        p.resetJointState(robot_id, joint_indices[joint_name], motor_to_joint_rad(joint, float(row[joint])))


def tool0_position(robot_id: int, link_indices: dict[str, int]) -> tuple[float, float, float]:
    state = p.getLinkState(robot_id, link_indices["tool0"], computeForwardKinematics=True)
    return tuple(state[4])


def main() -> None:
    args = parse_args()
    rows = read_rows(args.poses)
    robot_id, joint_indices, link_indices = setup_robot(args)
    print(f"poses: {args.poses.expanduser().resolve()}")
    print("PyBullet: taught joint-pose replay. Close window to exit.")

    debug_id = -1
    try:
        while True:
            for index, row in enumerate(rows):
                if not p.isConnected():
                    return
                apply_pose(robot_id, joint_indices, row)
                pos = tool0_position(robot_id, link_indices)
                name = row.get("name", f"row{index}").strip() or f"row{index}"
                label = f"{index}: {name}\\ntool0=({pos[0]*1000:.1f},{pos[1]*1000:.1f},{pos[2]*1000:.1f})mm"
                debug_id = p.addUserDebugText(label, [pos[0], pos[1], pos[2] + 0.03], [1, 0.85, 0], 0.65, replaceItemUniqueId=debug_id)
                print(
                    f"row {index}: {name}, "
                    + ", ".join(f"{joint}={float(row[joint]):.2f}" for joint in JOINT_COMMANDS)
                    + f", tool0=({pos[0]*1000:.1f},{pos[1]*1000:.1f},{pos[2]*1000:.1f})mm"
                )
                deadline = time.time() + args.row_delay
                while time.time() < deadline:
                    if not p.isConnected():
                        return
                    time.sleep(1.0 / 60.0)
            if not args.loop:
                while p.isConnected():
                    time.sleep(1.0 / 60.0)
                return
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
