#!/usr/bin/env python3
"""Build a jointed URDF from Fusion-exported occurrence meshes.

Inputs:
  cad/exports/fusion_tables/occurrences.csv
  cad/exports/fusion_tables/link_assignment_template.csv
  cad/exports/fusion_tables/joints_template.csv

The script intentionally derives link names and joint order from the CSV files
instead of hard-coding the old arm structure. When motors or links change,
update the CSV files and regenerate the URDF.
"""

import csv
import math
from collections import defaultdict
from pathlib import Path
from xml.sax.saxutils import escape


ROOT = Path(__file__).resolve().parents[1]
TABLE_DIR = ROOT / "cad" / "exports" / "fusion_tables"
MESH_DIR = ROOT / "cad" / "exports" / "fusion_meshes"
OCCURRENCES = TABLE_DIR / "occurrences.csv"
MANUAL_OCCURRENCES = TABLE_DIR / "manual_occurrences.csv"
VISUAL_LINK_CONFIG = TABLE_DIR / "visual_link_config.csv"
LINK_ASSIGNMENT = TABLE_DIR / "link_assignment_template.csv"
JOINTS = TABLE_DIR / "joints_template.csv"
OUTPUT = ROOT / "urdf" / "robot_arm_fusion.urdf"


COLORS = [
    ("base_gray", "0.45 0.45 0.45 1"),
    ("yaw_blue", "0.25 0.35 0.85 1"),
    ("shoulder_green", "0.2 0.65 0.35 1"),
    ("upper_orange", "0.95 0.55 0.15 1"),
    ("forearm_purple", "0.55 0.3 0.8 1"),
    ("elbow_red", "0.85 0.25 0.2 1"),
    ("wrist_cyan", "0.1 0.7 0.8 1"),
    ("tool_yellow", "0.9 0.8 0.1 1"),
]


def mm_to_m(value_mm: float) -> float:
    return value_mm / 1000.0


def cm_to_m(value_cm: str) -> float:
    return float(value_cm) * 0.01


def fmt(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def matrix_to_rpy(row: dict[str, str]) -> tuple[float, float, float]:
    if not row.get("m00"):
        return (0.0, 0.0, 0.0)

    r00 = float(row["m00"])
    r10 = float(row["m10"])
    r20 = float(row["m20"])
    r21 = float(row["m21"])
    r22 = float(row["m22"])

    pitch = math.atan2(-r20, math.sqrt(r00 * r00 + r10 * r10))
    if abs(abs(pitch) - math.pi / 2) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-float(row["m01"]), float(row["m11"]))
    else:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    return (roll, pitch, yaw)


def occurrence_xyz(row: dict[str, str]) -> tuple[float, float, float]:
    return (
        cm_to_m(row["origin_x_cm"]),
        cm_to_m(row["origin_y_cm"]),
        cm_to_m(row["origin_z_cm"]),
    )


def joint_origin(row: dict[str, str]) -> tuple[float, float, float]:
    if not (row["origin_x_mm"] and row["origin_y_mm"] and row["origin_z_mm"]):
        return (0.0, 0.0, 0.0)
    return (
        mm_to_m(float(row["origin_x_mm"])),
        mm_to_m(float(row["origin_y_mm"])),
        mm_to_m(float(row["origin_z_mm"])),
    )


def joint_limit(row: dict[str, str]) -> tuple[float, float]:
    lower = row.get("lower_rad", "").strip()
    upper = row.get("upper_rad", "").strip()
    if lower and upper:
        return (float(lower), float(upper))
    motor_id = row.get("motor_id", "").strip()
    if motor_id == "1":
        return (-math.pi, math.pi)
    return (-math.pi / 2, math.pi / 2)


def material_for_index(index: int) -> str:
    return COLORS[index % len(COLORS)][0]


def load_occurrences() -> dict[tuple[str, str], dict[str, str]]:
    if VISUAL_LINK_CONFIG.exists():
        rows = []
        for row in csv.DictReader(VISUAL_LINK_CONFIG.open()):
            if row.get("include", "").strip() not in ("1", "true", "TRUE", "yes", "YES"):
                continue
            if not row.get("origin_x_cm") or not row.get("m00"):
                continue
            row = row.copy()
            row["status"] = "exported"
            rows.append(row)
    else:
        rows = list(csv.DictReader(OCCURRENCES.open()))
        if MANUAL_OCCURRENCES.exists():
            rows.extend(csv.DictReader(MANUAL_OCCURRENCES.open()))
    by_key = {}
    for index, row in enumerate(rows):
        row_id = row.get("row_id") or f"occ_{index:03d}"
        row["row_id"] = row_id
        by_key[("row_id", row_id)] = row
        by_key[(row["full_path_name"], row["mesh_file"])] = row
    return by_key


def load_assignments(
    occurrences: dict[tuple[str, str], dict[str, str]],
) -> dict[str, list[dict[str, str]]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    skipped_stale = 0
    for row in csv.DictReader(LINK_ASSIGNMENT.open()):
        link = row["link_name"].strip()
        mesh_file = row["mesh_file"].strip()
        if not link or row["status"] != "exported":
            continue
        if not (MESH_DIR / mesh_file).exists():
            continue
        row_id = row.get("row_id", "").strip()
        occ = occurrences.get(("row_id", row_id)) if row_id else None
        if occ is None:
            occ = occurrences.get((row["full_path_name"], mesh_file))
        if occ is None:
            skipped_stale += 1
            continue
        grouped[link].append(occ)
    if skipped_stale:
        print(
            f"warning: skipped {skipped_stale} stale link-assignment rows. "
            "Regenerate or edit link_assignment_template.csv for the current CAD."
        )
    return grouped


def load_joints() -> list[dict[str, str]]:
    return list(csv.DictReader(JOINTS.open()))


def ordered_links(
    assignments: dict[str, list[dict[str, str]]],
    joints: list[dict[str, str]],
) -> list[str]:
    result = []

    def add(link: str) -> None:
        if link and link not in result:
            result.append(link)

    for row in joints:
        add(row["parent_link"].strip())
        add(row["child_link"].strip())
    for link in assignments:
        add(link)
    return result


def link_world_origins(
    links: list[str],
    joints: list[dict[str, str]],
) -> dict[str, tuple[float, float, float]]:
    children = {row["child_link"].strip() for row in joints}
    roots = [link for link in links if link not in children]
    if not roots and links:
        roots = [links[0]]

    origins = {root: (0.0, 0.0, 0.0) for root in roots}
    unresolved = joints[:]
    while unresolved:
        progressed = False
        next_unresolved = []
        for row in unresolved:
            parent = row["parent_link"].strip()
            child = row["child_link"].strip()
            if parent not in origins:
                next_unresolved.append(row)
                continue
            offset = joint_origin(row)
            parent_origin = origins[parent]
            origins[child] = tuple(parent_origin[i] + offset[i] for i in range(3))
            progressed = True
        if not progressed:
            missing = ", ".join(row["joint_name"] for row in next_unresolved)
            raise ValueError(f"Cannot resolve joint tree. Check parent/child links: {missing}")
        unresolved = next_unresolved

    for link in links:
        origins.setdefault(link, (0.0, 0.0, 0.0))
    return origins


def main() -> None:
    occurrences = load_occurrences()
    assignments = load_assignments(occurrences)
    joints = load_joints()
    links = ordered_links(assignments, joints)
    origins = link_world_origins(links, joints)

    lines = ['<?xml version="1.0"?>', '<robot name="robot_arm_fusion">']
    for name, color in COLORS:
        lines.append(f'  <material name="{name}"><color rgba="{color}"/></material>')
    lines.append("")

    for link_index, link in enumerate(links):
        link_origin = origins[link]
        lines.append(f'  <link name="{link}">')
        lines.extend(
            [
                "    <inertial>",
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                '      <mass value="0.1"/>',
                '      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>',
                "    </inertial>",
            ]
        )
        for occ in assignments.get(link, []):
            occ_xyz = occurrence_xyz(occ)
            visual_xyz = tuple(occ_xyz[i] - link_origin[i] for i in range(3))
            visual_rpy = matrix_to_rpy(occ)
            mesh_uri = f"../cad/exports/fusion_meshes/{escape(occ['mesh_file'])}"
            lines.extend(
                [
                    "    <visual>",
                    f'      <origin xyz="{fmt(visual_xyz)}" rpy="{fmt(visual_rpy)}"/>',
                    f'      <geometry><mesh filename="{mesh_uri}" scale="0.001 0.001 0.001"/></geometry>',
                    f'      <material name="{material_for_index(link_index)}"/>',
                    "    </visual>",
                ]
            )
        lines.append("  </link>")
        lines.append("")

    for row in joints:
        lower, upper = joint_limit(row)
        axis = (float(row["axis_x"]), float(row["axis_y"]), float(row["axis_z"]))
        joint_type = row.get("joint_type", "").strip() or "revolute"
        lines.extend(
            [
                f'  <joint name="{row["joint_name"]}" type="{joint_type}">',
                f'    <parent link="{row["parent_link"].strip()}"/>',
                f'    <child link="{row["child_link"].strip()}"/>',
                f'    <origin xyz="{fmt(joint_origin(row))}" rpy="0 0 0"/>',
            ]
        )
        if joint_type != "fixed":
            lines.extend(
                [
                    f'    <axis xyz="{fmt(axis)}"/>',
                    f'    <limit lower="{lower:.9g}" upper="{upper:.9g}" effort="2.0" velocity="1.0"/>',
                ]
            )
        lines.extend(["  </joint>", ""])

    lines.append("</robot>")
    OUTPUT.write_text("\n".join(lines) + "\n")
    print(OUTPUT)
    print(f"links: {len(links)}, joints: {len(joints)}")


if __name__ == "__main__":
    main()
