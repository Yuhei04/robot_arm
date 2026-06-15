#!/usr/bin/env python3
"""Send a generated joint trajectory CSV to OpenCR.

Default mode is dry-run. Add --execute to write serial commands.
The preferred OpenCR command is q <j1> <j2> <j3> <j4> <j5> <j6>,
which updates all six joints from one trajectory row together.
"""

import argparse
import csv
import time
from pathlib import Path


DEFAULT_CSV = Path(__file__).resolve().parents[1] / "outputs" / "tool0_line_trajectory.csv"
DXL_IDS = {
    "j1": 1,
    "j2": 2,
    "j3": 3,
    "j4": 4,
    "j5": 5,
    "j6": 6,
}
DEFAULT_JOINT_MIN_DEG = -180.0
DEFAULT_JOINT_MAX_DEG = 180.0
DEFAULT_WARN_STEP_DEG = 10.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dry-run or send a joint trajectory CSV to OpenCR.")
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="Trajectory CSV path")
    parser.add_argument("--port", help="Serial port, for example /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="OpenCR debug serial baudrate")
    parser.add_argument("--execute", action="store_true", help="Actually write commands to OpenCR")
    parser.add_argument("--delay", type=float, default=0.12, help="Delay after each trajectory row [s]")
    parser.add_argument("--command-delay", type=float, default=0.02, help="Delay between per-joint commands [s]")
    parser.add_argument(
        "--command-mode",
        choices=("multi", "per-joint"),
        default="multi",
        help="Use one q command per row, or legacy per-joint m commands",
    )
    parser.add_argument("--joint-min-deg", type=float, default=DEFAULT_JOINT_MIN_DEG, help="Joint warning/reject lower bound [deg]")
    parser.add_argument("--joint-max-deg", type=float, default=DEFAULT_JOINT_MAX_DEG, help="Joint warning/reject upper bound [deg]")
    parser.add_argument("--max-step-deg", type=float, default=DEFAULT_WARN_STEP_DEG, help="Warn or reject rows with any joint step over this [deg]")
    parser.add_argument("--strict-safety", action="store_true", help="Reject safety warnings instead of only printing them")
    parser.add_argument("--allow-non-ok-status", action="store_true", help="Allow non-OK CSV status rows with a warning")
    parser.add_argument("--skip-first", action="store_true", help="Skip the first CSV row, often the current pose")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def parse_joint_row(row: dict[str, str]) -> dict[str, float]:
    result = {}
    for joint in DXL_IDS:
        if joint not in row:
            raise ValueError(f"Missing {joint} column in trajectory CSV")
        result[joint] = float(row[joint])
    return result


def validate_rows(
    rows: list[dict[str, str]],
    max_step_deg: float,
    joint_min_deg: float,
    joint_max_deg: float,
    strict_safety: bool,
    allow_non_ok_status: bool,
) -> tuple[list[dict[str, float]], list[str]]:
    parsed = []
    warnings = []
    previous = None
    for index, row in enumerate(rows):
        status = row.get("status", "OK")
        if status != "OK" and not allow_non_ok_status:
            message = f"Row {index}: status={status!r}"
            if strict_safety:
                raise ValueError("Rejected: " + message)
            warnings.append(message)
        joints = parse_joint_row(row)
        for joint, value in joints.items():
            if value < joint_min_deg or value > joint_max_deg:
                message = f"Row {index}: {joint}={value:.3f} outside {joint_min_deg:.1f}..{joint_max_deg:.1f} deg"
                if strict_safety:
                    raise ValueError("Rejected: " + message)
                warnings.append(message)
        if previous is not None:
            for joint, value in joints.items():
                delta = abs(value - previous[joint])
                if delta > max_step_deg:
                    message = f"Row {index}: {joint} step {delta:.3f} deg exceeds {max_step_deg:.3f}"
                    if strict_safety:
                        raise ValueError("Rejected: " + message)
                    warnings.append(message)
        parsed.append(joints)
        previous = joints
    return parsed, warnings


def command_lines(joints: dict[str, float], command_mode: str) -> list[str]:
    if command_mode == "multi":
        ordered = " ".join(f"{joints[joint]:.3f}" for joint in DXL_IDS)
        return [f"q {ordered}"]
    return [f"m {DXL_IDS[joint]} {joints[joint]:.3f}" for joint in DXL_IDS]


def open_serial(port: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError("pyserial is required for --execute. Install it in the active Python environment.") from exc
    return serial.Serial(port, baudrate=baud, timeout=1.0)


def main() -> None:
    args = parse_args()
    rows = read_rows(args.csv.expanduser().resolve())
    joints_rows, warnings = validate_rows(
        rows,
        args.max_step_deg,
        args.joint_min_deg,
        args.joint_max_deg,
        args.strict_safety,
        args.allow_non_ok_status,
    )
    if args.skip_first and joints_rows:
        joints_rows = joints_rows[1:]

    print(f"trajectory: {args.csv.expanduser().resolve()}")
    print(f"rows_to_send: {len(joints_rows)}")
    print(f"mode: {'EXECUTE' if args.execute else 'DRY_RUN'}")
    print(f"command_mode: {args.command_mode}")
    print(f"row_delay_s: {args.delay:.3f}")
    print(f"safety: {'STRICT_REJECT' if args.strict_safety else 'WARN_ONLY'}")
    print(f"joint_range_check_deg: {args.joint_min_deg:.1f}..{args.joint_max_deg:.1f}")
    print(f"step_warning_deg: {args.max_step_deg:.1f}")
    if warnings:
        print("warnings:")
        for warning in warnings:
            print("  WARNING: " + warning)

    serial_port = None
    if args.execute:
        if not args.port:
            raise ValueError("--port is required with --execute")
        serial_port = open_serial(args.port, args.baud)
        time.sleep(2.0)
        print(f"opened: {args.port} @ {args.baud}")

    try:
        for row_index, joints in enumerate(joints_rows):
            print(f"row {row_index}: " + ", ".join(f"{j}={joints[j]:.3f}" for j in DXL_IDS))
            for line in command_lines(joints, args.command_mode):
                print("  " + line)
                if serial_port is not None:
                    serial_port.write((line + "\n").encode("ascii"))
                    serial_port.flush()
                    time.sleep(args.command_delay)
            if serial_port is not None:
                time.sleep(args.delay)
    finally:
        if serial_port is not None:
            serial_port.close()
            print("closed serial port")


if __name__ == "__main__":
    main()
