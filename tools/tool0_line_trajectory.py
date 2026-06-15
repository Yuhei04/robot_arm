#!/usr/bin/env python3
"""Generate a first Cartesian line trajectory for tool0 with PyBullet IK."""

import argparse
import csv
import math
from pathlib import Path

import pybullet as p

from joint_limits import read_joint_limits


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_fusion.urdf"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "outputs" / "tool0_line_trajectory.csv"
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
    parser = argparse.ArgumentParser(description="Generate a position-only tool0 line trajectory.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height above floor/table [mm]")
    parser.add_argument("--link", default="tool0", help="Target link name")
    parser.add_argument("--start-x", type=float, required=True, help="Start X [mm]")
    parser.add_argument("--start-y", type=float, required=True, help="Start Y [mm]")
    parser.add_argument("--start-z", type=float, required=True, help="Start Z [mm]")
    parser.add_argument("--goal-x", type=float, required=True, help="Goal X [mm]")
    parser.add_argument("--goal-y", type=float, required=True, help="Goal Y [mm]")
    parser.add_argument("--goal-z", type=float, required=True, help="Goal Z [mm]")
    parser.add_argument("--step-mm", type=float, default=2.0, help="Cartesian interpolation step [mm]")
    parser.add_argument("--max-joint-step-deg", type=float, default=5.0, help="Warn if any joint changes more than this per row [deg]")
    parser.add_argument("--threshold-mm", type=float, default=1.5, help="Acceptable IK/FK position error [mm]")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="CSV output path")
    for joint in JOINT_COMMANDS:
        parser.add_argument(f"--current-{joint}", type=float, default=0.0, help=f"Seed/current {joint.upper()} motor angle [deg]")
    return parser.parse_args()


def motor_to_joint_rad(name: str, motor_deg: float) -> float:
    return math.radians(JOINT_COMMANDS[name][1] * motor_deg)


def joint_rad_to_motor_deg(name: str, joint_rad: float) -> float:
    return math.degrees(joint_rad) / JOINT_COMMANDS[name][1]


def clamp_motor(name: str, value: float) -> float:
    lo, hi = MOTOR_LIMITS_DEG[name]
    return min(max(value, lo), hi)


def motor_limits_joint_space() -> tuple[list[float], list[float], list[float]]:
    lower = []
    upper = []
    for name in JOINT_COMMANDS:
        lo_deg, hi_deg = MOTOR_LIMITS_DEG[name]
        a = motor_to_joint_rad(name, lo_deg)
        b = motor_to_joint_rad(name, hi_deg)
        lower.append(min(a, b))
        upper.append(max(a, b))
    return lower, upper, [upper[i] - lower[i] for i in range(len(lower))]


def main() -> None:
    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
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

    if args.link not in link_indices:
        available = ", ".join(sorted(link_indices))
        raise ValueError(f"Link {args.link!r} not found. Available: {available}")

    start = [args.start_x, args.start_y, args.start_z]
    goal = [args.goal_x, args.goal_y, args.goal_z]
    distance = math.sqrt(sum((goal[i] - start[i]) ** 2 for i in range(3)))
    steps = max(1, math.ceil(distance / args.step_mm))
    lower, upper, ranges = motor_limits_joint_space()
    current = {name: getattr(args, f"current_{name}") for name in JOINT_COMMANDS}
    rows = []
    failed = False

    for step in range(steps + 1):
        t = step / steps
        target = [start[i] + (goal[i] - start[i]) * t for i in range(3)]
        rest = [motor_to_joint_rad(name, current[name]) for name in JOINT_COMMANDS]
        for name, (joint_name, _) in JOINT_COMMANDS.items():
            p.resetJointState(robot_id, joint_indices[joint_name], motor_to_joint_rad(name, current[name]))
        solution = p.calculateInverseKinematics(
            robot_id,
            link_indices[args.link],
            [v / 1000.0 for v in target],
            lowerLimits=lower,
            upperLimits=upper,
            jointRanges=ranges,
            restPoses=rest,
            maxNumIterations=300,
            residualThreshold=args.threshold_mm / 1000.0,
        )
        previous = current.copy()
        for i, name in enumerate(JOINT_COMMANDS):
            current[name] = clamp_motor(name, joint_rad_to_motor_deg(name, solution[i]))
            joint_name, _ = JOINT_COMMANDS[name]
            p.resetJointState(robot_id, joint_indices[joint_name], motor_to_joint_rad(name, current[name]))

        state = p.getLinkState(robot_id, link_indices[args.link], computeForwardKinematics=True)
        actual = [state[4][i] * 1000.0 for i in range(3)]
        err = [actual[i] - target[i] for i in range(3)]
        err_norm = math.sqrt(sum(v * v for v in err))
        max_joint_step = max(abs(current[name] - previous[name]) for name in JOINT_COMMANDS) if step else 0.0
        status = "OK"
        if err_norm > args.threshold_mm:
            status = "IK_ERROR"
            failed = True
        elif max_joint_step > args.max_joint_step_deg:
            status = "JOINT_STEP_WARN"
        rows.append({
            "step": step,
            "t": f"{t:.4f}",
            "target_x_mm": f"{target[0]:.3f}",
            "target_y_mm": f"{target[1]:.3f}",
            "target_z_mm": f"{target[2]:.3f}",
            **{name: f"{current[name]:.3f}" for name in JOINT_COMMANDS},
            "fk_x_mm": f"{actual[0]:.3f}",
            "fk_y_mm": f"{actual[1]:.3f}",
            "fk_z_mm": f"{actual[2]:.3f}",
            "error_mm": f"{err_norm:.3f}",
            "max_joint_step_deg": f"{max_joint_step:.3f}",
            "status": status,
        })

    p.disconnect()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["step", "t", "target_x_mm", "target_y_mm", "target_z_mm", *JOINT_COMMANDS.keys(), "fk_x_mm", "fk_y_mm", "fk_z_mm", "error_mm", "max_joint_step_deg", "status"]
    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(output_path)
    print(f"points: {len(rows)}")
    print(f"max_error_mm: {max(float(r['error_mm']) for r in rows):.3f}")
    print(f"max_joint_step_deg: {max(float(r['max_joint_step_deg']) for r in rows):.3f}")
    print("status: " + ("CHECK_ERROR" if failed else "OK"))


if __name__ == "__main__":
    main()
