#!/usr/bin/env python3
"""View explicit tool0 XYZ waypoints and their IK solutions in PyBullet."""

import argparse
import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data

from move_tool0_points import JOINT_COMMANDS, max_joint_delta, parse_active_joints, read_points, solve_ik


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_fusion.urdf"
DEFAULT_POINTS = Path(__file__).resolve().parents[1] / "outputs" / "tool0_cube_points.csv"
DEFAULT_BASE_Z_MM = 10.25


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View waypoint IK solutions in PyBullet.")
    parser.add_argument("--points", type=Path, default=DEFAULT_POINTS, help="CSV with x_mm,y_mm,z_mm columns")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height [mm]")
    parser.add_argument("--active-joints", default="j1,j2,j3,j4,j5,j6", help="Comma-separated joints IK may move")
    parser.add_argument("--row-delay", type=float, default=0.8, help="Delay between points [s]")
    parser.add_argument("--loop", action="store_true", help="Loop points until window closes")
    parser.add_argument("--check-only", action="store_true", help="Solve and print summary without GUI")
    parser.add_argument("--distance", type=float, default=0.42, help="Initial camera distance")
    parser.add_argument("--yaw", type=float, default=35.0, help="Initial camera yaw [deg]")
    parser.add_argument("--pitch", type=float, default=-25.0, help="Initial camera pitch [deg]")
    parser.add_argument("--target-x", type=float, default=0.0, help="Initial camera target X [m]")
    parser.add_argument("--target-y", type=float, default=-0.16, help="Initial camera target Y [m]")
    parser.add_argument("--target-z", type=float, default=0.21, help="Initial camera target Z [m]")
    for joint in JOINT_COMMANDS:
        parser.add_argument(f"--current-{joint}", type=float, default=0.0, help=f"Seed/current {joint.upper()} angle [deg]")
    return parser.parse_args()


def motor_to_joint_rad(name: str, motor_deg: float) -> float:
    return math.radians(JOINT_COMMANDS[name][1] * motor_deg)


def setup_robot(args: argparse.Namespace) -> tuple[int, dict[str, int], dict[str, int]]:
    robot_id = p.loadURDF(str(args.urdf.expanduser().resolve()), basePosition=[0, 0, args.base_z_mm / 1000.0], useFixedBase=True)
    joint_indices = {}
    link_indices = {}
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_indices[info[1].decode("utf-8")] = joint_index
        link_indices[info[12].decode("utf-8")] = joint_index
    return robot_id, joint_indices, link_indices


def apply_joints(robot_id: int, joint_indices: dict[str, int], joints: dict[str, float]) -> None:
    for name, (joint_name, _) in JOINT_COMMANDS.items():
        p.resetJointState(robot_id, joint_indices[joint_name], motor_to_joint_rad(name, joints[name]))


def tool0_pos(robot_id: int, link_indices: dict[str, int]) -> tuple[float, float, float]:
    state = p.getLinkState(robot_id, link_indices["tool0"], computeForwardKinematics=True)
    return state[4]


def marker(pos, color: list[float], size: float = 0.006) -> None:
    p.addUserDebugLine([pos[0] - size, pos[1], pos[2]], [pos[0] + size, pos[1], pos[2]], color, 3, 0)
    p.addUserDebugLine([pos[0], pos[1] - size, pos[2]], [pos[0], pos[1] + size, pos[2]], color, 3, 0)
    p.addUserDebugLine([pos[0], pos[1], pos[2] - size], [pos[0], pos[1], pos[2] + size], color, 3, 0)


def draw_static_targets(points: list[dict[str, float]]) -> None:
    target_positions = []
    for point in points:
        pos = [point["x_mm"] / 1000.0, point["y_mm"] / 1000.0, point["z_mm"] / 1000.0]
        target_positions.append(pos)
        marker(pos, [1, 1, 0], 0.005)
    for i in range(1, len(target_positions)):
        p.addUserDebugLine(target_positions[i - 1], target_positions[i], [1, 0.8, 0], 2, 0)


def solve_all(args: argparse.Namespace, robot_id: int, joint_indices: dict[str, int], link_indices: dict[str, int]) -> list[dict[str, object]]:
    points = read_points(args.points)
    active = parse_active_joints(args.active_joints)
    current = {name: getattr(args, f"current_{name}") for name in JOINT_COMMANDS}
    rows = []
    for index, point in enumerate(points):
        target = (point["x_mm"], point["y_mm"], point["z_mm"])
        previous = current.copy()
        joints, error_mm = solve_ik(robot_id, joint_indices, link_indices, current, target, active)
        apply_joints(robot_id, joint_indices, joints)
        fk_m = tool0_pos(robot_id, link_indices)
        rows.append({
            "index": index,
            "target_mm": target,
            "target_m": [v / 1000.0 for v in target],
            "joints": joints,
            "fk_m": fk_m,
            "error_mm": error_mm,
            "jump_deg": max_joint_delta(previous, joints),
        })
        current = joints
    return rows


def main() -> None:
    args = parse_args()
    connection = p.DIRECT if args.check_only else p.GUI
    p.connect(connection)
    if not args.check_only:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
    robot_id, joint_indices, link_indices = setup_robot(args)
    rows = solve_all(args, robot_id, joint_indices, link_indices)

    print(f"points: {args.points.expanduser().resolve()}")
    print(f"active_joints: {args.active_joints}")
    print(f"rows={len(rows)}, max_error_mm={max(r['error_mm'] for r in rows):.3f}, max_jump_deg={max(r['jump_deg'] for r in rows):.3f}")
    for row in rows:
        target = row["target_mm"]
        print(f"point {row['index']}: target=({target[0]:.1f},{target[1]:.1f},{target[2]:.1f}) err={row['error_mm']:.2f}mm jump={row['jump_deg']:.2f}deg")
    if args.check_only:
        p.disconnect()
        return

    p.resetDebugVisualizerCamera(args.distance, args.yaw, args.pitch, [args.target_x, args.target_y, args.target_z])
    draw_static_targets(read_points(args.points))
    print("PyBullet: yellow=target waypoint, red=IK/FK tool0, red line=IK error. Close window to exit.")

    while p.isConnected():
        for row in rows:
            if not p.isConnected():
                break
            apply_joints(robot_id, joint_indices, row["joints"])
            target = row["target_m"]
            fk = tool0_pos(robot_id, link_indices)
            marker(fk, [1, 0, 0], 0.006)
            p.addUserDebugLine(target, fk, [1, 0, 0], 2, args.row_delay)
            p.addUserDebugText(f"{row['index']} err={row['error_mm']:.1f}mm", [fk[0], fk[1], fk[2] + 0.015], [1, 1, 1], 0.75, args.row_delay)
            p.stepSimulation()
            time.sleep(args.row_delay)
        if not args.loop:
            while p.isConnected():
                p.stepSimulation()
                time.sleep(1 / 240)


if __name__ == "__main__":
    main()
