#!/usr/bin/env python3
"""View a URDF model in PyBullet."""

import argparse
import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_simple_3axis.urdf"

LEGACY_JOINT_MAP = {
    "j1": ("base_yaw_joint", 1.0),
    "j2": ("shoulder_pitch_joint", 1.0),
    "j3": ("elbow_pitch_joint", -1.0),
    "j4": ("elbow_roll_joint", -1.0),
    "j5": ("wrist_pitch_joint", 1.0),
    "j6": ("upper_arm_roll_joint", -1.0),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Open a URDF model in PyBullet.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--j1", type=float, default=0.0, help="Legacy ID 1 motor angle [deg]")
    parser.add_argument("--j2", type=float, default=0.0, help="Legacy ID 2 motor angle [deg]")
    parser.add_argument("--j3", type=float, default=0.0, help="Legacy ID 3 motor angle [deg]")
    parser.add_argument("--j4", type=float, default=0.0, help="Legacy ID 4 motor angle [deg]")
    parser.add_argument("--j5", type=float, default=0.0, help="Legacy ID 5 motor angle [deg]")
    parser.add_argument("--j6", type=float, default=0.0, help="Legacy ID 6 motor angle [deg]")
    parser.add_argument("--distance", type=float, default=0.6, help="Initial camera distance")
    parser.add_argument("--yaw", type=float, default=45.0, help="Initial camera yaw [deg]")
    parser.add_argument("--pitch", type=float, default=-25.0, help="Initial camera pitch [deg]")
    parser.add_argument("--target-x", type=float, default=0.06, help="Initial camera target X [m]")
    parser.add_argument("--target-y", type=float, default=0.0, help="Initial camera target Y [m]")
    parser.add_argument("--target-z", type=float, default=0.18, help="Initial camera target Z [m]")
    parser.add_argument(
        "--no-sliders",
        action="store_true",
        help="Disable joint sliders and use only the initial command-line angles",
    )
    args = parser.parse_args()

    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(urdf_path)

    physics_client = p.connect(p.GUI)
    if physics_client < 0:
        raise RuntimeError("Failed to connect to PyBullet GUI")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(str(urdf_path), basePosition=[0, 0, 0], useFixedBase=True)

    joint_indices = {}
    movable_joints = []
    print(f"Loaded: {urdf_path}")
    print("Joint mapping:")
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]
        print(f"  {joint_index}: {joint_name}")
        joint_indices[joint_name] = joint_index
        if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            movable_joints.append((joint_index, joint_name, joint_type))

    legacy_angles = {
        "j1": args.j1,
        "j2": args.j2,
        "j3": args.j3,
        "j4": args.j4,
        "j5": args.j5,
        "j6": args.j6,
    }
    for arg_name, motor_deg in legacy_angles.items():
        joint_name, sign = LEGACY_JOINT_MAP[arg_name]
        if joint_name in joint_indices:
            p.resetJointState(robot_id, joint_indices[joint_name], math.radians(sign * motor_deg))

    slider_ids = {}
    if not args.no_sliders:
        for joint_index, joint_name, joint_type in movable_joints:
            if joint_type == p.JOINT_REVOLUTE:
                value = math.degrees(p.getJointState(robot_id, joint_index)[0])
                slider_ids[joint_name] = p.addUserDebugParameter(joint_name, -180.0, 180.0, value)
            else:
                value = p.getJointState(robot_id, joint_index)[0]
                slider_ids[joint_name] = p.addUserDebugParameter(joint_name, -0.2, 0.2, value)

    p.resetDebugVisualizerCamera(
        cameraDistance=args.distance,
        cameraYaw=args.yaw,
        cameraPitch=args.pitch,
        cameraTargetPosition=[args.target_x, args.target_y, args.target_z],
    )

    print("Camera controls:")
    print("  Drag with the mouse in the PyBullet window to rotate/pan/zoom.")
    print("  Or restart with --yaw, --pitch, --distance, --target-x/y/z.")
    if slider_ids:
        print("Joint sliders are shown in the PyBullet GUI using URDF joint names.")
    print("Close the PyBullet window to exit.")
    while p.isConnected():
        for joint_name, slider_id in slider_ids.items():
            value = p.readUserDebugParameter(slider_id)
            joint_index = joint_indices[joint_name]
            joint_type = p.getJointInfo(robot_id, joint_index)[2]
            if joint_type == p.JOINT_REVOLUTE:
                value = math.radians(value)
            p.resetJointState(robot_id, joint_index, value)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


if __name__ == "__main__":
    main()
