#!/usr/bin/env python3
"""Replay a generated j1..j6 trajectory CSV in PyBullet before sending it to OpenCR."""

import argparse
import csv
import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_fusion.urdf"
DEFAULT_CSV = Path(__file__).resolve().parents[1] / "outputs" / "tool0_line_trajectory.csv"
DEFAULT_BASE_Z_MM = 10.25
JOINT_MAP = {
    "j1": ("base_yaw_joint", 1.0),
    "j2": ("shoulder_pitch_joint", 1.0),
    "j3": ("elbow_pitch_joint", -1.0),
    "j4": ("elbow_roll_joint", 1.0),
    "j5": ("wrist_pitch_joint", -1.0),
    "j6": ("wrist_roll_joint", -1.0),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a tool0 trajectory CSV in PyBullet.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="Trajectory CSV path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height above floor/table [mm]")
    parser.add_argument("--tool-link", default="tool0", help="Tool link name to mark")
    parser.add_argument("--tool-marker-size", type=float, default=0.02, help="Tool marker axis length [m]")
    parser.add_argument("--row-delay", type=float, default=0.35, help="Delay between trajectory rows [s]")
    parser.add_argument("--loop", action="store_true", help="Loop the trajectory until the window closes")
    parser.add_argument("--check-only", action="store_true", help="Load URDF and CSV in DIRECT mode, then print summary and exit")
    parser.add_argument("--distance", type=float, default=0.32, help="Initial camera distance")
    parser.add_argument("--yaw", type=float, default=35.0, help="Initial camera yaw [deg]")
    parser.add_argument("--pitch", type=float, default=-20.0, help="Initial camera pitch [deg]")
    parser.add_argument("--target-x", type=float, default=0.0, help="Initial camera target X [m]")
    parser.add_argument("--target-y", type=float, default=-0.14, help="Initial camera target Y [m]")
    parser.add_argument("--target-z", type=float, default=0.24, help="Initial camera target Z [m]")
    parser.add_argument("--no-camera-sliders", action="store_true", help="Disable camera control sliders")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"Trajectory CSV has no rows: {path}")
    for row_index, row in enumerate(rows):
        for joint in JOINT_MAP:
            if joint not in row:
                raise ValueError(f"Row {row_index} is missing {joint}")
    return rows


def motor_to_joint_rad(name: str, motor_deg: float) -> float:
    return math.radians(JOINT_MAP[name][1] * motor_deg)


def apply_row(robot_id: int, joint_indices: dict[str, int], row: dict[str, str]) -> None:
    for name, (joint_name, _) in JOINT_MAP.items():
        joint_index = joint_indices.get(joint_name)
        if joint_index is not None:
            p.resetJointState(robot_id, joint_index, motor_to_joint_rad(name, float(row[name])))


def link_pose(robot_id: int, link_index: int) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    state = p.getLinkState(robot_id, link_index, computeForwardKinematics=True)
    return state[4], state[5]


def draw_tool_marker(robot_id: int, link_index: int, size: float, item_ids: list[int]) -> list[int]:
    pos, orn = link_pose(robot_id, link_index)
    rot = p.getMatrixFromQuaternion(orn)
    axes = (
        ([rot[0], rot[3], rot[6]], [1, 0, 0]),
        ([rot[1], rot[4], rot[7]], [0, 1, 0]),
        ([rot[2], rot[5], rot[8]], [0, 0.2, 1]),
    )
    next_ids = []
    for i, (axis, color) in enumerate(axes):
        end = [pos[j] + axis[j] * size for j in range(3)]
        replace_id = item_ids[i] if i < len(item_ids) else -1
        next_ids.append(p.addUserDebugLine(pos, end, color, 4, 0.2, replaceItemUniqueId=replace_id))
    label_pos = [pos[0], pos[1], pos[2] + size * 0.6]
    replace_id = item_ids[3] if len(item_ids) > 3 else -1
    next_ids.append(p.addUserDebugText("tool0", label_pos, [1, 0.85, 0], 0.65, 0.2, replaceItemUniqueId=replace_id))
    return next_ids


def draw_target_points(rows: list[dict[str, str]]) -> None:
    points = []
    for row in rows:
        if not all(key in row for key in ("target_x_mm", "target_y_mm", "target_z_mm")):
            return
        points.append([
            float(row["target_x_mm"]) / 1000.0,
            float(row["target_y_mm"]) / 1000.0,
            float(row["target_z_mm"]) / 1000.0,
        ])
    for i, pos in enumerate(points):
        size = 0.004
        p.addUserDebugLine([pos[0] - size, pos[1], pos[2]], [pos[0] + size, pos[1], pos[2]], [1, 1, 0], 2, 0)
        p.addUserDebugLine([pos[0], pos[1] - size, pos[2]], [pos[0], pos[1] + size, pos[2]], [1, 1, 0], 2, 0)
        p.addUserDebugLine([pos[0], pos[1], pos[2] - size], [pos[0], pos[1], pos[2] + size], [1, 1, 0], 2, 0)
        if i > 0:
            p.addUserDebugLine(points[i - 1], pos, [1, 0.8, 0], 2, 0)


def summarize(rows: list[dict[str, str]]) -> str:
    statuses = sorted(set(row.get("status", "") for row in rows if row.get("status", "")))
    max_error = max(float(row.get("error_mm", "0") or 0) for row in rows)
    max_step = max(float(row.get("max_joint_step_deg", "0") or 0) for row in rows)
    return f"rows={len(rows)}, statuses={statuses or ['(none)']}, max_error_mm={max_error:.3f}, max_joint_step_deg={max_step:.3f}"


def add_camera_sliders(args: argparse.Namespace) -> dict[str, int]:
    return {
        "distance": p.addUserDebugParameter("camera_distance", 0.1, 1.5, args.distance),
        "yaw": p.addUserDebugParameter("camera_yaw", -180.0, 180.0, args.yaw),
        "pitch": p.addUserDebugParameter("camera_pitch", -89.0, 30.0, args.pitch),
        "target_x": p.addUserDebugParameter("camera_target_x", -0.4, 0.4, args.target_x),
        "target_y": p.addUserDebugParameter("camera_target_y", -0.5, 0.3, args.target_y),
        "target_z": p.addUserDebugParameter("camera_target_z", 0.0, 0.6, args.target_z),
    }


def update_camera_from_sliders(slider_ids: dict[str, int]) -> None:
    if not slider_ids:
        return
    distance = p.readUserDebugParameter(slider_ids["distance"])
    yaw = p.readUserDebugParameter(slider_ids["yaw"])
    pitch = p.readUserDebugParameter(slider_ids["pitch"])
    target = [
        p.readUserDebugParameter(slider_ids["target_x"]),
        p.readUserDebugParameter(slider_ids["target_y"]),
        p.readUserDebugParameter(slider_ids["target_z"]),
    ]
    p.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=target,
    )


def main() -> None:
    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    csv_path = args.csv.expanduser().resolve()
    rows = read_rows(csv_path)

    connection = p.DIRECT if args.check_only else p.GUI
    physics_client = p.connect(connection)
    if physics_client < 0:
        raise RuntimeError("Failed to connect to PyBullet")

    if not args.check_only:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(str(urdf_path), basePosition=[0, 0, args.base_z_mm / 1000.0], useFixedBase=True)

    joint_indices = {}
    link_indices = {}
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_indices[info[1].decode("utf-8")] = joint_index
        link_indices[info[12].decode("utf-8")] = joint_index

    if args.tool_link not in link_indices:
        available = ", ".join(sorted(link_indices))
        raise ValueError(f"Link {args.tool_link!r} not found. Available: {available}")
    tool_index = link_indices[args.tool_link]

    print(f"URDF: {urdf_path}")
    print(f"CSV: {csv_path}")
    print(summarize(rows))

    apply_row(robot_id, joint_indices, rows[0])
    pos, orn = link_pose(robot_id, tool_index)
    rpy = p.getEulerFromQuaternion(orn)
    print(f"first_tool0_mm: x={pos[0]*1000:.3f}, y={pos[1]*1000:.3f}, z={pos[2]*1000:.3f}")
    print(f"first_tool0_rpy_deg: roll={math.degrees(rpy[0]):.3f}, pitch={math.degrees(rpy[1]):.3f}, yaw={math.degrees(rpy[2]):.3f}")
    if args.check_only:
        p.disconnect()
        return

    p.resetDebugVisualizerCamera(
        cameraDistance=args.distance,
        cameraYaw=args.yaw,
        cameraPitch=args.pitch,
        cameraTargetPosition=[args.target_x, args.target_y, args.target_z],
    )
    draw_target_points(rows)
    camera_slider_ids = {} if args.no_camera_sliders else add_camera_sliders(args)
    print("Replaying trajectory. Close the PyBullet window to exit.")
    if camera_slider_ids:
        print("Camera sliders: distance/yaw/pitch/target_x/target_y/target_z")

    marker_ids: list[int] = []
    while p.isConnected():
        for row_index, row in enumerate(rows):
            if not p.isConnected():
                break
            update_camera_from_sliders(camera_slider_ids)
            apply_row(robot_id, joint_indices, row)
            marker_ids = draw_tool_marker(robot_id, tool_index, args.tool_marker_size, marker_ids)
            p.addUserDebugText(f"row {row_index}", [args.target_x, args.target_y, args.target_z + 0.04], [1, 1, 1], 0.8, args.row_delay)
            p.stepSimulation()
            time.sleep(args.row_delay)
        if not args.loop:
            while p.isConnected():
                update_camera_from_sliders(camera_slider_ids)
                marker_ids = draw_tool_marker(robot_id, tool_index, args.tool_marker_size, marker_ids)
                p.stepSimulation()
                time.sleep(1.0 / 240.0)


if __name__ == "__main__":
    main()
