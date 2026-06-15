#!/usr/bin/env python3
"""Move tool0 through explicit XYZ waypoints using one IK command per point.

No interpolation is performed. Each CSV row is solved with IK, converted to one
q j1..j6 OpenCR command, and optionally sent to the real arm.
"""

import argparse
import csv
import math
import re
import time
from pathlib import Path
from typing import Any

import pybullet as p

from joint_limits import read_joint_limits


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_fusion.urdf"
DEFAULT_POINTS = Path(__file__).resolve().parents[1] / "outputs" / "tool0_rectangle_points.csv"
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
ANGLE_RE = re.compile(r"ID\s+(\d+)\s*:\s*([-+0-9.]+)\s*deg")
ID_TO_JOINT = {1: "j1", 2: "j2", 3: "j3", 4: "j4", 5: "j5", 6: "j6"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move tool0 through explicit XYZ waypoints using IK.")
    parser.add_argument("--points", type=Path, default=DEFAULT_POINTS, help="CSV with x_mm,y_mm,z_mm columns")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height [mm]")
    parser.add_argument("--port", default="/dev/ttyACM0", help="OpenCR serial port")
    parser.add_argument("--baud", type=int, default=115200, help="OpenCR debug serial baudrate")
    parser.add_argument("--execute", action="store_true", help="Actually send q commands to OpenCR")
    parser.add_argument("--read-current", action="store_true", help="Read current joint angles from OpenCR before solving")
    parser.add_argument("--active-joints", default="j1,j2,j3", help="Comma-separated joints IK may move; inactive joints stay fixed")
    parser.add_argument("--delay", type=float, default=1.0, help="Delay after each point when executing [s]")
    parser.add_argument("--threshold-mm", type=float, default=5.0, help="Warn if IK/FK error exceeds this [mm]")
    parser.add_argument("--max-joint-jump-deg", type=float, default=30.0, help="Warn if any command jumps more than this from previous [deg]")
    parser.add_argument("--strict", action="store_true", help="Reject warnings instead of only printing them")
    for joint in JOINT_COMMANDS:
        parser.add_argument(f"--current-{joint}", type=float, default=0.0, help=f"Seed/current {joint.upper()} angle [deg]")
    return parser.parse_args()


def parse_active_joints(value: str) -> set[str]:
    active = {item.strip() for item in value.split(",") if item.strip()}
    unknown = active - set(JOINT_COMMANDS)
    if unknown:
        raise ValueError(f"Unknown active joints: {', '.join(sorted(unknown))}")
    return active


def motor_to_joint_rad(name: str, motor_deg: float) -> float:
    return math.radians(JOINT_COMMANDS[name][1] * motor_deg)


def joint_rad_to_motor_deg(name: str, joint_rad: float) -> float:
    return math.degrees(joint_rad) / JOINT_COMMANDS[name][1]


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def read_points(path: Path) -> list[dict[str, float]]:
    with path.expanduser().resolve().open(newline="") as f:
        rows = []
        for index, row in enumerate(csv.DictReader(f)):
            try:
                rows.append({"x_mm": float(row["x_mm"]), "y_mm": float(row["y_mm"]), "z_mm": float(row["z_mm"])})
            except KeyError as exc:
                raise ValueError("points CSV must have x_mm,y_mm,z_mm columns") from exc
            except ValueError as exc:
                raise ValueError(f"Invalid numeric value in row {index}: {row}") from exc
    if not rows:
        raise ValueError(f"No points in {path}")
    return rows


def setup_ik(args: argparse.Namespace) -> tuple[int, dict[str, int], dict[str, int]]:
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
    if "tool0" not in link_indices:
        raise ValueError("tool0 link not found in URDF")
    return robot_id, joint_indices, link_indices


def solve_ik(
    robot_id: int,
    joint_indices: dict[str, int],
    link_indices: dict[str, int],
    current: dict[str, float],
    target_xyz_mm: tuple[float, float, float],
    active_joints: set[str],
) -> tuple[dict[str, float], float]:
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
        maxNumIterations=300,
        residualThreshold=0.003,
    )

    result = {}
    for i, name in enumerate(JOINT_COMMANDS):
        if name not in active_joints:
            result[name] = current[name]
            continue
        lo, hi = MOTOR_LIMITS_DEG[name]
        result[name] = clamp(joint_rad_to_motor_deg(name, solution[i]), lo, hi)
        p.resetJointState(robot_id, joint_indices[JOINT_COMMANDS[name][0]], motor_to_joint_rad(name, result[name]))

    state = p.getLinkState(robot_id, link_indices["tool0"], computeForwardKinematics=True)
    fk = [state[4][i] * 1000.0 for i in range(3)]
    err = math.sqrt(sum((fk[i] - target_xyz_mm[i]) ** 2 for i in range(3)))
    return result, err


def q_line(joints: dict[str, float]) -> str:
    return "q " + " ".join(f"{joints[name]:.3f}" for name in JOINT_COMMANDS)


def max_joint_delta(a: dict[str, float], b: dict[str, float]) -> float:
    return max(abs(a[name] - b[name]) for name in JOINT_COMMANDS)


def open_serial(args: argparse.Namespace) -> Any:
    import serial  # type: ignore

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.1)
    time.sleep(2.0)
    return ser


def read_current_angles(port: str, baud: int) -> dict[str, float]:
    import serial  # type: ignore

    with serial.Serial(port, baudrate=baud, timeout=0.1) as ser:
        time.sleep(2.5)
        ser.reset_input_buffer()
        ser.write(b"a\n")
        ser.flush()
        deadline = time.time() + 3.0
        chunks = []
        while time.time() < deadline:
            data = ser.read(4096)
            if data:
                chunks.append(data)
            else:
                time.sleep(0.05)
    text = b"".join(chunks).decode("utf-8", errors="replace")
    angles = {}
    for match in ANGLE_RE.finditer(text):
        joint = ID_TO_JOINT.get(int(match.group(1)))
        if joint:
            angles[joint] = float(match.group(2))
    missing = [name for name in JOINT_COMMANDS if name not in angles]
    if missing:
        raise RuntimeError("Could not read all joint angles from OpenCR output:\n" + text)
    return angles


def warn_or_raise(message: str, strict: bool) -> None:
    if strict:
        raise RuntimeError(message)
    print("WARNING: " + message)


def main() -> None:
    args = parse_args()
    points = read_points(args.points)
    active_joints = parse_active_joints(args.active_joints)
    current = {name: getattr(args, f"current_{name}") for name in JOINT_COMMANDS}
    if args.read_current:
        current = read_current_angles(args.port, args.baud)

    robot_id, joint_indices, link_indices = setup_ik(args)
    serial_port = None
    if args.execute:
        serial_port = open_serial(args)

    print(f"points: {args.points.expanduser().resolve()}")
    print(f"mode: {'EXECUTE' if args.execute else 'DRY_RUN'}")
    print("active_joints: " + ",".join(sorted(active_joints)))
    print("start_deg: " + ", ".join(f"{name}={current[name]:.2f}" for name in JOINT_COMMANDS))

    try:
        for index, point in enumerate(points):
            target = (point["x_mm"], point["y_mm"], point["z_mm"])
            previous = current.copy()
            solved, error_mm = solve_ik(robot_id, joint_indices, link_indices, current, target, active_joints)
            jump = max_joint_delta(previous, solved)
            if error_mm > args.threshold_mm:
                warn_or_raise(f"point {index}: IK/FK error {error_mm:.2f} mm exceeds {args.threshold_mm:.2f} mm", args.strict)
            if jump > args.max_joint_jump_deg:
                warn_or_raise(f"point {index}: joint jump {jump:.2f} deg exceeds {args.max_joint_jump_deg:.2f} deg", args.strict)
            line = q_line(solved)
            print(f"point {index}: x={target[0]:.1f}, y={target[1]:.1f}, z={target[2]:.1f}, err={error_mm:.2f}mm, jump={jump:.2f}deg")
            print("  " + line)
            if serial_port is not None:
                serial_port.write((line + "\n").encode("ascii"))
                serial_port.flush()
                time.sleep(args.delay)
            current = solved
    finally:
        if serial_port is not None:
            serial_port.close()
            print("closed serial port")
        p.disconnect()


if __name__ == "__main__":
    main()
