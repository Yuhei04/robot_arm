#!/usr/bin/env python3
"""Record the current OpenCR joint angles as a named taught pose."""

import argparse
import csv
from pathlib import Path

from robot_pose_io import JOINT_NAMES, read_current_angles


DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "outputs" / "taught_poses.csv"
FIELDNAMES = ["name", *JOINT_NAMES, "hand", "wait_s", "notes"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Append the current robot joint angles to a pose CSV.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Pose CSV to append")
    parser.add_argument("--name", required=True, help="Pose name, for example approach or grasp")
    parser.add_argument("--port", default="/dev/ttyACM0", help="OpenCR serial port")
    parser.add_argument("--baud", type=int, default=115200, help="OpenCR debug serial baudrate")
    parser.add_argument("--hand", default="", help="Optional hand action for this pose, for example open or close")
    parser.add_argument("--wait-s", type=float, default=1.0, help="Delay after replaying this row [s]")
    parser.add_argument("--notes", default="", help="Free-form notes")
    return parser.parse_args()


def append_pose(path: Path, row: dict[str, str]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists() and path.stat().st_size > 0
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def main() -> None:
    args = parse_args()
    angles = read_current_angles(args.port, args.baud)
    row = {
        "name": args.name,
        **{joint: f"{angles[joint]:.3f}" for joint in JOINT_NAMES},
        "hand": args.hand.strip(),
        "wait_s": f"{args.wait_s:.3f}",
        "notes": args.notes,
    }
    append_pose(args.output, row)
    print(f"recorded: {args.name}")
    print(f"output: {args.output.expanduser().resolve()}")
    print(", ".join(f"{joint}={angles[joint]:.3f}" for joint in JOINT_NAMES))
    if args.hand.strip():
        print(f"hand: {args.hand.strip()}")


if __name__ == "__main__":
    main()
