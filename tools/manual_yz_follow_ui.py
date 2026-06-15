#!/usr/bin/env python3
"""Manual Y/Z slider UI for fixed-X tool0 following.

Use this to test IK, serial command timing, and real-arm smoothness without any
camera input. Dry-run is the default; add --execute to move the arm.
"""

import argparse
import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
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
    parser = argparse.ArgumentParser(description="Manual Y/Z slider follower for OpenCR.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height [mm]")
    parser.add_argument("--port", default="/dev/ttyACM0", help="OpenCR serial port")
    parser.add_argument("--baud", type=int, default=115200, help="OpenCR debug serial baudrate")
    parser.add_argument("--execute", action="store_true", help="Actually send q commands to OpenCR")
    parser.add_argument("--fixed-x-mm", type=float, default=0.0, help="Fixed tool0 X target [mm]")
    parser.add_argument("--y-min-mm", type=int, default=-240, help="Slider minimum Y [mm]")
    parser.add_argument("--y-max-mm", type=int, default=-80, help="Slider maximum Y [mm]")
    parser.add_argument("--z-min-mm", type=int, default=120, help="Slider minimum Z [mm]")
    parser.add_argument("--z-max-mm", type=int, default=280, help="Slider maximum Z [mm]")
    parser.add_argument("--start-y-mm", type=int, default=-160, help="Initial target Y [mm]")
    parser.add_argument("--start-z-mm", type=int, default=190, help="Initial target Z [mm]")
    parser.add_argument("--rate-hz", type=float, default=8.0, help="Max q command rate")
    parser.add_argument("--max-step-deg", type=float, default=1.2, help="Per-command joint step clamp [deg]")
    parser.add_argument("--deadband-mm", type=float, default=1.0, help="Ignore tiny target changes [mm]")
    parser.add_argument("--active-joints", default="j1,j2,j3", help="Comma-separated joints IK may move; inactive joints stay fixed")
    parser.add_argument("--window", default="manual_yz_follow", help="OpenCV window name")
    for joint in JOINT_COMMANDS:
        parser.add_argument(f"--current-{joint}", type=float, default=0.0, help=f"Seed/current {joint.upper()} angle [deg]")
    return parser.parse_args()


def motor_to_joint_rad(name: str, motor_deg: float) -> float:
    return math.radians(JOINT_COMMANDS[name][1] * motor_deg)


def joint_rad_to_motor_deg(name: str, joint_rad: float) -> float:
    return math.degrees(joint_rad) / JOINT_COMMANDS[name][1]


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def parse_active_joints(value: str) -> set[str]:
    active = {item.strip() for item in value.split(",") if item.strip()}
    unknown = active - set(JOINT_COMMANDS)
    if unknown:
        raise ValueError(f"Unknown active joints: {', '.join(sorted(unknown))}")
    return active


def setup_ik(args: argparse.Namespace) -> tuple[int, dict[str, int], dict[str, int]]:
    urdf_path = args.urdf.expanduser().resolve()
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
    return robot_id, joint_indices, link_indices


def solve_ik(
    robot_id: int,
    joint_indices: dict[str, int],
    link_indices: dict[str, int],
    current: dict[str, float],
    target_xyz_mm: tuple[float, float, float],
    active_joints: set[str],
) -> dict[str, float]:
    for name, (joint_name, _) in JOINT_COMMANDS.items():
        p.resetJointState(robot_id, joint_indices[joint_name], motor_to_joint_rad(name, current[name]))
    lower = []
    upper = []
    ranges = []
    rest = []
    for name in JOINT_COMMANDS:
        current_rad = motor_to_joint_rad(name, current[name])
        rest.append(current_rad)
        if name in active_joints:
            lo_deg, hi_deg = MOTOR_LIMITS_DEG[name]
            a = motor_to_joint_rad(name, lo_deg)
            b = motor_to_joint_rad(name, hi_deg)
            lower.append(min(a, b))
            upper.append(max(a, b))
        else:
            lower.append(current_rad - 1e-6)
            upper.append(current_rad + 1e-6)
        ranges.append(max(upper[-1] - lower[-1], 1e-6))
    solution = p.calculateInverseKinematics(
        robot_id,
        link_indices["tool0"],
        [v / 1000.0 for v in target_xyz_mm],
        lowerLimits=lower,
        upperLimits=upper,
        jointRanges=ranges,
        restPoses=rest,
        maxNumIterations=120,
        residualThreshold=0.003,
    )
    result = {}
    for i, name in enumerate(JOINT_COMMANDS):
        if name not in active_joints:
            result[name] = current[name]
            continue
        lo, hi = MOTOR_LIMITS_DEG[name]
        result[name] = clamp(joint_rad_to_motor_deg(name, solution[i]), lo, hi)
    return result


def clamp_joint_step(current: dict[str, float], target: dict[str, float], max_step_deg: float) -> dict[str, float]:
    stepped = {}
    for name in JOINT_COMMANDS:
        delta = clamp(target[name] - current[name], -max_step_deg, max_step_deg)
        stepped[name] = current[name] + delta
    return stepped


def open_serial(args: argparse.Namespace) -> Any:
    import serial  # type: ignore

    port = serial.Serial(args.port, baudrate=args.baud, timeout=1.0)
    time.sleep(2.0)
    return port


def q_line(joints: dict[str, float]) -> str:
    return "q " + " ".join(f"{joints[name]:.3f}" for name in JOINT_COMMANDS)


def set_slider(window: str, name: str, value_mm: int, min_mm: int) -> None:
    cv2.setTrackbarPos(name, window, int(value_mm - min_mm))


def get_slider(window: str, name: str, min_mm: int) -> int:
    return cv2.getTrackbarPos(name, window) + min_mm


def draw_panel(args: argparse.Namespace, target_y: float, target_z: float, current: dict[str, float], active_joints: set[str], line: str | None) -> Any:
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    mode = "EXECUTE" if args.execute else "DRY RUN"
    cv2.putText(image, f"Manual Y/Z follower - {mode}", (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(image, f"target: X={args.fixed_x_mm:.1f}  Y={target_y:.1f}  Z={target_z:.1f} mm", (20, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(image, "Keys: q/Esc quit, space send current target now", (20, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
    cv2.putText(image, "active IK joints: " + ",".join(sorted(active_joints)), (20, 142), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 220, 255), 1)
    y_bar_x = int(np.interp(target_y, [args.y_min_mm, args.y_max_mm], [80, 560]))
    z_bar_x = int(np.interp(target_z, [args.z_min_mm, args.z_max_mm], [80, 560]))
    cv2.line(image, (80, 185), (560, 185), (80, 80, 80), 2)
    cv2.circle(image, (y_bar_x, 185), 8, (0, 255, 255), -1)
    cv2.putText(image, "Y", (35, 191), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.line(image, (80, 240), (560, 240), (80, 80, 80), 2)
    cv2.circle(image, (z_bar_x, 240), 8, (0, 180, 255), -1)
    cv2.putText(image, "Z", (35, 246), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2)
    cv2.putText(image, ", ".join(f"{j}={current[j]:.1f}" for j in JOINT_COMMANDS), (20, 286), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1)
    if line:
        cv2.putText(image, line[:82], (20, 326), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 220, 255), 1)
    return image


def main() -> None:
    args = parse_args()
    if args.y_min_mm >= args.y_max_mm or args.z_min_mm >= args.z_max_mm:
        raise ValueError("Slider min must be less than max")

    active_joints = parse_active_joints(args.active_joints)
    robot_id, joint_indices, link_indices = setup_ik(args)
    current = {name: getattr(args, f"current_{name}") for name in JOINT_COMMANDS}
    serial_port = open_serial(args) if args.execute else None
    last_sent_at = 0.0
    last_target: tuple[float, float] | None = None
    last_line: str | None = None

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window, 760, 520)
    cv2.createTrackbar("Y mm", args.window, 0, args.y_max_mm - args.y_min_mm, lambda _v: None)
    cv2.createTrackbar("Z mm", args.window, 0, args.z_max_mm - args.z_min_mm, lambda _v: None)
    set_slider(args.window, "Y mm", int(clamp(args.start_y_mm, args.y_min_mm, args.y_max_mm)), args.y_min_mm)
    set_slider(args.window, "Z mm", int(clamp(args.start_z_mm, args.z_min_mm, args.z_max_mm)), args.z_min_mm)

    print(f"mode: {'EXECUTE' if args.execute else 'DRY_RUN'}")
    print(f"fixed_x_mm: {args.fixed_x_mm:.3f}")
    print("active_joints: " + ",".join(sorted(active_joints)))
    print("seed_deg: " + ", ".join(f"{name}={current[name]:.2f}" for name in JOINT_COMMANDS))

    try:
        while True:
            target_y = float(get_slider(args.window, "Y mm", args.y_min_mm))
            target_z = float(get_slider(args.window, "Z mm", args.z_min_mm))
            now = time.monotonic()
            enough_time = now - last_sent_at >= 1.0 / args.rate_hz
            enough_motion = (
                last_target is None
                or abs(target_y - last_target[0]) >= args.deadband_mm
                or abs(target_z - last_target[1]) >= args.deadband_mm
            )
            key = cv2.waitKey(10) & 0xFF
            force_send = key == ord(" ")
            if (enough_time and enough_motion) or force_send:
                target_xyz = (args.fixed_x_mm, target_y, target_z)
                ik_target = solve_ik(robot_id, joint_indices, link_indices, current, target_xyz, active_joints)
                command = clamp_joint_step(current, ik_target, args.max_step_deg)
                last_line = q_line(command)
                print(f"target_mm: x={target_xyz[0]:.1f}, y={target_xyz[1]:.1f}, z={target_xyz[2]:.1f} -> {last_line}")
                if serial_port is not None:
                    serial_port.write((last_line + "\n").encode("ascii"))
                    serial_port.flush()
                current = command
                last_sent_at = now
                last_target = (target_y, target_z)

            cv2.imshow(args.window, draw_panel(args, target_y, target_z, current, active_joints, last_line))
            if key in (27, ord("q")):
                break
    finally:
        if serial_port is not None:
            serial_port.close()
        p.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
