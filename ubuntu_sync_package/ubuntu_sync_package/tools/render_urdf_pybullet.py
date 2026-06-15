#!/usr/bin/env python3
"""Render a URDF model to a PNG using PyBullet DIRECT mode."""

import argparse
import math
import struct
import zlib
from pathlib import Path

import pybullet as p


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_simple_3axis.urdf"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "outputs" / "robot_arm_simple_6axis.png"


def write_png(path: Path, width: int, height: int, rgba_pixels) -> None:
    if hasattr(rgba_pixels, "tobytes"):
        rgba_bytes = rgba_pixels.tobytes()
    else:
        rgba_bytes = bytes(rgba_pixels)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    rgb_rows = []
    for y in range(height):
        row = bytearray()
        row.append(0)
        for x in range(width):
            i = 4 * (y * width + x)
            row.extend(rgba_bytes[i : i + 3])
        rgb_rows.append(bytes(row))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(b"".join(rgb_rows), level=9))
    png += chunk(b"IEND", b"")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def main() -> None:
    parser = argparse.ArgumentParser(description="Render a URDF model to PNG with PyBullet.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Output PNG path")
    parser.add_argument("--j1", type=float, default=0.0, help="ID 1 motor angle [deg]")
    parser.add_argument("--j2", type=float, default=0.0, help="ID 2 motor angle [deg]")
    parser.add_argument("--j3", type=float, default=0.0, help="ID 3 motor angle [deg]")
    parser.add_argument("--j4", type=float, default=0.0, help="ID 4 motor angle [deg]")
    parser.add_argument("--j5", type=float, default=0.0, help="ID 5 motor angle [deg]")
    parser.add_argument("--j6", type=float, default=0.0, help="ID 6 motor angle [deg]")
    parser.add_argument(
        "--joint",
        action="append",
        default=[],
        metavar="NAME=DEG",
        help="Set a revolute joint by URDF joint name in degrees. Can be repeated.",
    )
    parser.add_argument("--width", type=int, default=1200, help="Image width")
    parser.add_argument("--height", type=int, default=900, help="Image height")
    parser.add_argument(
        "--camera-eye",
        type=float,
        nargs=3,
        default=[0.34, -0.42, 0.36],
        metavar=("X", "Y", "Z"),
        help="Camera eye position [m]",
    )
    parser.add_argument(
        "--camera-target",
        type=float,
        nargs=3,
        default=[0.06, 0.00, 0.18],
        metavar=("X", "Y", "Z"),
        help="Camera target position [m]",
    )
    parser.add_argument(
        "--camera-up",
        type=float,
        nargs=3,
        default=[0, 0, 1],
        metavar=("X", "Y", "Z"),
        help="Camera up vector",
    )
    args = parser.parse_args()

    urdf_path = args.urdf.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(urdf_path)

    p.connect(p.DIRECT)
    robot_id = p.loadURDF(str(urdf_path), basePosition=[0, 0, 0], useFixedBase=True)

    initial_angles = {
        "base_yaw_joint": math.radians(args.j1),
        "shoulder_pitch_joint": math.radians(args.j2),
        "upper_arm_roll_joint": math.radians(-args.j6),
        "elbow_pitch_joint": math.radians(-args.j3),
        "elbow_roll_joint": math.radians(-args.j4),
        "wrist_pitch_joint": math.radians(args.j5),
    }
    for item in args.joint:
        if "=" not in item:
            raise ValueError(f"--joint must be NAME=DEG, got: {item}")
        name, value = item.split("=", 1)
        initial_angles[name] = math.radians(float(value))
    for joint_index in range(p.getNumJoints(robot_id)):
        joint_name = p.getJointInfo(robot_id, joint_index)[1].decode("utf-8")
        if joint_name in initial_angles:
            p.resetJointState(robot_id, joint_index, initial_angles[joint_name])

    view = p.computeViewMatrix(
        cameraEyePosition=args.camera_eye,
        cameraTargetPosition=args.camera_target,
        cameraUpVector=args.camera_up,
    )
    proj = p.computeProjectionMatrixFOV(
        fov=45,
        aspect=args.width / args.height,
        nearVal=0.01,
        farVal=3.0,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        args.width,
        args.height,
        view,
        proj,
        renderer=p.ER_TINY_RENDERER,
    )
    write_png(output_path, args.width, args.height, rgba)
    p.disconnect()

    print(output_path)


if __name__ == "__main__":
    main()
