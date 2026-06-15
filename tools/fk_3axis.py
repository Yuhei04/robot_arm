#!/usr/bin/env python3
"""Compute a simple 3-axis forward kinematics estimate.

This model uses only base yaw, shoulder pitch, and elbow pitch.
It estimates the wrist reference point before wrist/tool offsets.

Zero posture:
  shoulder-to-elbow link points along +Z
  elbow-to-wrist link points along +X
"""

import argparse
import math


L0_MM = 106.0
L1_MM = 111.0
L2_MM = 70.0


def fk_3axis(base_motor_deg: float, shoulder_motor_deg: float, elbow_motor_deg: float):
    base = math.radians(base_motor_deg)
    shoulder = math.radians(shoulder_motor_deg)
    elbow = math.radians(-elbow_motor_deg)

    radial = L1_MM * math.sin(shoulder) + L2_MM * math.cos(shoulder + elbow)
    z = L0_MM + L1_MM * math.cos(shoulder) - L2_MM * math.sin(shoulder + elbow)
    x = radial * math.cos(base)
    y = radial * math.sin(base)

    return x, y, z


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute wrist reference position from motor angles in degrees."
    )
    parser.add_argument("--j1", type=float, required=True, help="ID 1 motor angle [deg]")
    parser.add_argument("--j2", type=float, required=True, help="ID 2 motor angle [deg]")
    parser.add_argument("--j3", type=float, required=True, help="ID 3 motor angle [deg]")
    args = parser.parse_args()

    x, y, z = fk_3axis(args.j1, args.j2, args.j3)
    print("3-axis FK wrist reference estimate")
    print(f"input motor angles: j1={args.j1:.2f} deg, j2={args.j2:.2f} deg, j3={args.j3:.2f} deg")
    print("model signs: base=+j1, shoulder=+j2, elbow=-j3")
    print(f"x = {x:.2f} mm")
    print(f"y = {y:.2f} mm")
    print(f"z = {z:.2f} mm")


if __name__ == "__main__":
    main()
