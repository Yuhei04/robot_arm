#!/usr/bin/env python3
"""Dry-run or replay a taught joint-pose sequence on OpenCR and an optional ESP32 hand."""

import argparse
import csv
import time
from pathlib import Path
from typing import Any

from joint_limits import read_joint_limits
from robot_pose_io import JOINT_NAMES, max_joint_delta, q_line, read_current_angles


DEFAULT_POSES = Path(__file__).resolve().parents[1] / "outputs" / "taught_poses.csv"
FIELDNAMES = ["name", *JOINT_NAMES, "hand", "wait_s", "notes"]
DEFAULT_HAND_COMMANDS = {
    "open": "open",
    "close": "close",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a taught joint pose CSV.")
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSES, help="Pose CSV with name,j1..j6,hand,wait_s columns")
    parser.add_argument("--port", default="/dev/ttyACM0", help="OpenCR serial port")
    parser.add_argument("--baud", type=int, default=115200, help="OpenCR debug serial baudrate")
    parser.add_argument("--execute", action="store_true", help="Actually send commands")
    parser.add_argument("--read-current", action="store_true", help="Read current arm angles before safety jump checks")
    parser.add_argument("--delay", type=float, default=None, help="Override per-row wait_s [s]")
    parser.add_argument("--max-joint-jump-deg", type=float, default=60.0, help="Warn if any row jumps more than this [deg]")
    parser.add_argument("--strict", action="store_true", help="Reject warnings instead of only printing them")
    parser.add_argument("--hand-port", default="", help="Optional ESP32-C3 serial port")
    parser.add_argument("--hand-baud", type=int, default=115200, help="ESP32-C3 serial baudrate")
    parser.add_argument("--hand-open-command", default=DEFAULT_HAND_COMMANDS["open"], help="Raw command sent for hand=open")
    parser.add_argument("--hand-close-command", default=DEFAULT_HAND_COMMANDS["close"], help="Raw command sent for hand=close")
    parser.add_argument("--hand-after-arm", action="store_true", help="Send hand action after arm pose instead of before it")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"No poses in {path}")
    missing = [name for name in FIELDNAMES[:7] if name not in rows[0]]
    if missing:
        raise ValueError(f"Pose CSV is missing columns: {', '.join(missing)}")
    return rows


def parse_joints(row: dict[str, str], index: int) -> dict[str, float]:
    joints = {}
    for joint in JOINT_NAMES:
        try:
            joints[joint] = float(row[joint])
        except ValueError as exc:
            raise ValueError(f"Invalid {joint} value at row {index}: {row[joint]!r}") from exc
    return joints


def warn_or_raise(message: str, strict: bool) -> None:
    if strict:
        raise RuntimeError(message)
    print("WARNING: " + message)


def open_serial(port: str, baud: int) -> Any:
    import serial  # type: ignore

    ser = serial.Serial(port, baudrate=baud, timeout=0.1)
    time.sleep(2.0)
    return ser


def hand_command(action: str, args: argparse.Namespace) -> str:
    action = action.strip()
    if action == "open":
        return args.hand_open_command
    if action == "close":
        return args.hand_close_command
    return action


def validate_limits(joints: dict[str, float], row_index: int, strict: bool) -> None:
    limits = read_joint_limits()
    for joint, value in joints.items():
        lo, hi = limits[joint]
        if value < lo or value > hi:
            warn_or_raise(f"row {row_index}: {joint}={value:.3f} outside {lo:.1f}..{hi:.1f} deg", strict)


def maybe_send(serial_port: Any, line: str) -> None:
    if serial_port is None:
        return
    serial_port.write((line + "\n").encode("ascii"))
    serial_port.flush()


def main() -> None:
    args = parse_args()
    rows = read_rows(args.poses)
    parsed = [parse_joints(row, index) for index, row in enumerate(rows)]

    previous = read_current_angles(args.port, args.baud) if args.read_current else parsed[0]
    for index, joints in enumerate(parsed):
        validate_limits(joints, index, args.strict)
        jump = max_joint_delta(previous, joints)
        if jump > args.max_joint_jump_deg:
            warn_or_raise(f"row {index}: joint jump {jump:.2f} deg exceeds {args.max_joint_jump_deg:.2f} deg", args.strict)
        previous = joints

    print(f"poses: {args.poses.expanduser().resolve()}")
    print(f"rows_to_send: {len(rows)}")
    print(f"mode: {'EXECUTE' if args.execute else 'DRY_RUN'}")
    print(f"arm_port: {args.port} @ {args.baud}")
    print(f"hand_port: {args.hand_port or '(disabled)'}")

    arm_serial = None
    hand_serial = None
    if args.execute:
        arm_serial = open_serial(args.port, args.baud)
        if args.hand_port:
            hand_serial = open_serial(args.hand_port, args.hand_baud)

    try:
        for index, (row, joints) in enumerate(zip(rows, parsed)):
            name = row.get("name", f"row{index}").strip() or f"row{index}"
            action = row.get("hand", "").strip()
            wait_s = args.delay if args.delay is not None else float(row.get("wait_s", "1.0") or 1.0)
            arm_line = q_line(joints)
            raw_hand_line = hand_command(action, args) if action else ""

            print(f"row {index}: {name}")
            if raw_hand_line and not args.hand_after_arm:
                print(f"  hand: {raw_hand_line}")
                maybe_send(hand_serial, raw_hand_line)
                time.sleep(0.05 if arm_serial or hand_serial else 0.0)
            print(f"  arm: {arm_line}")
            maybe_send(arm_serial, arm_line)
            if raw_hand_line and args.hand_after_arm:
                print(f"  hand: {raw_hand_line}")
                maybe_send(hand_serial, raw_hand_line)
            if arm_serial or hand_serial:
                time.sleep(wait_s)
            print(f"  wait_s: {wait_s:.3f}")
    finally:
        if arm_serial is not None:
            arm_serial.close()
            print("closed arm serial port")
        if hand_serial is not None:
            hand_serial.close()
            print("closed hand serial port")


if __name__ == "__main__":
    main()
