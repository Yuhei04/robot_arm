#!/usr/bin/env python3
"""Check whether Fusion coordinate exports look current on Ubuntu."""

import csv
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OCCURRENCES = ROOT / "cad" / "exports" / "fusion_tables" / "occurrences.csv"
VISUAL_URDF = ROOT / "urdf" / "robot_arm_fusion_visual.urdf"

TARGET_MESHES = [
    "XC-430_idle_1_M_DC11_A01_IDLER_ASM_1_M_DC11_A01_IDLER_1.stl",
    "XC-430_idle_1_M_DC11_A01_HORN_ASM_1_M_DC11_A01_HORN_1.stl",
    "2XC-430_1_DC11_A01_HORN_REF_ASM_1_DC11_A01_HORN_REF_1.stl",
]

PARENT_MOTOR_STL_RE = re.compile(
    r"XC-430_idle_[12]\.stl|2XC-430_1\.stl|XM_H-430_idler_[12]\.stl"
)


def near_zero(values: tuple[float, float, float], threshold: float = 0.1) -> bool:
    return all(abs(value) <= threshold for value in values)


def main() -> None:
    rows = list(csv.DictReader(OCCURRENCES.open(newline="")))
    by_mesh = {row["mesh_file"]: row for row in rows}
    failed = False

    print("Coordinate CSV check:")
    for mesh in TARGET_MESHES:
        row = by_mesh.get(mesh)
        print(mesh)
        if row is None:
            failed = True
            print("  not found")
            continue

        origin = (
            float(row["origin_x_cm"]),
            float(row["origin_y_cm"]),
            float(row["origin_z_cm"]),
        )
        has_matrix = bool(row.get("m00"))
        print(f"  origin_cm: {origin[0]}, {origin[1]}, {origin[2]}")
        print(f"  has_matrix: {has_matrix}")
        if not has_matrix:
            failed = True
            print("  NG: occurrences.csv is missing transform matrix columns")
        if near_zero(origin):
            failed = True
            print("  NG: origin is near 0,0,0; Fusion export may be old")

    print("\nParent motor STL exclusion check:")
    if not VISUAL_URDF.exists():
        failed = True
        print(f"  NG: missing {VISUAL_URDF}")
    else:
        matches = sorted(set(PARENT_MOTOR_STL_RE.findall(VISUAL_URDF.read_text())))
        if matches:
            failed = True
            print("  NG: parent motor STLs are still included")
            for match in matches:
                print(f"  {match}")
        else:
            print("  OK: parent motor STLs are excluded")

    if failed:
        raise SystemExit(1)
    print("\nOK: coordinate export looks current")


if __name__ == "__main__":
    main()
