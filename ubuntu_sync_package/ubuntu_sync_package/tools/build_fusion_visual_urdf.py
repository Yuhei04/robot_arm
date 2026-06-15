#!/usr/bin/env python3
"""Build a fixed-joint URDF from Fusion-exported occurrence meshes.

This is only for visual inspection. It does not contain real robot joints yet.
"""

import csv
import math
from pathlib import Path
from xml.sax.saxutils import escape


ROOT = Path(__file__).resolve().parents[1]
OCCURRENCES = ROOT / "cad" / "exports" / "fusion_tables" / "occurrences.csv"
MANUAL_OCCURRENCES = ROOT / "cad" / "exports" / "fusion_tables" / "manual_occurrences.csv"
VISUAL_LINK_CONFIG = ROOT / "cad" / "exports" / "fusion_tables" / "visual_link_config.csv"
MESH_DIR = ROOT / "cad" / "exports" / "fusion_meshes"
OUTPUT = ROOT / "urdf" / "robot_arm_fusion_visual.urdf"


def link_name(index: int, mesh_file: str) -> str:
    stem = Path(mesh_file).stem
    safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in stem)
    return f"fusion_part_{index:03d}_{safe}"


def matrix_to_rpy(row: dict[str, str]) -> tuple[float, float, float]:
    """Convert Fusion occurrence rotation matrix to URDF roll/pitch/yaw."""
    if not row.get("m00"):
        return (0.0, 0.0, 0.0)

    r00 = float(row["m00"])
    r10 = float(row["m10"])
    r20 = float(row["m20"])
    r21 = float(row["m21"])
    r22 = float(row["m22"])

    # URDF uses fixed-axis RPY. This is the common ZYX extraction.
    pitch = math.atan2(-r20, math.sqrt(r00 * r00 + r10 * r10))
    if abs(abs(pitch) - math.pi / 2) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-float(row["m01"]), float(row["m11"]))
    else:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    return (roll, pitch, yaw)


def occurrence_origin(row: dict[str, str]) -> tuple[float, float, float]:
    # Fusion API lengths are cm. URDF uses m.
    return (
        float(row["origin_x_cm"]) * 0.01,
        float(row["origin_y_cm"]) * 0.01,
        float(row["origin_z_cm"]) * 0.01,
    )


def fmt(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def has_exported_descendant(row: dict[str, str], exported_paths: set[str]) -> bool:
    prefix = row["full_path_name"] + "+"
    return any(path.startswith(prefix) for path in exported_paths)


def should_skip_row(row: dict[str, str], exported_paths: set[str]) -> bool:
    component_name = row["component_name"]
    # XM/H idler parent occurrences are offset in the current export. Their
    # DUMMY_DC12 child meshes have the useful assembly-frame placement.
    if component_name == "XM,H-430_idler":
        return True
    if component_name == "DUMMY_DC12":
        return False
    # XC-430 parent occurrences are offset in the current export. Keep the
    # case children instead and drop internal idler/horn hardware.
    if component_name == "XC-430_idle":
        return True
    if row["full_path_name"].startswith("XC-430_idle:"):
        if component_name.startswith("DC11_B01_CASE"):
            return False
        if component_name.startswith("M_DC11_A01_HORN") and "HORN_ASM" not in component_name:
            return False
    if "IDLER" in component_name or "HORN_INSERT" in component_name or "SCREW" in component_name:
        return True
    return has_exported_descendant(row, exported_paths)


def load_rows() -> list[dict[str, str]]:
    if VISUAL_LINK_CONFIG.exists():
        rows = []
        for row in csv.DictReader(VISUAL_LINK_CONFIG.open()):
            if row.get("include", "").strip() not in ("1", "true", "TRUE", "yes", "YES"):
                continue
            if not row.get("origin_x_cm") or not row.get("m00"):
                raise ValueError(
                    f"{VISUAL_LINK_CONFIG} includes {row['mesh_file']} but transform columns are empty"
                )
            normalized = {
                "row_id": row["row_id"],
                "occurrence_name": row["occurrence_name"] or row["mesh_file"],
                "full_path_name": row["full_path_name"],
                "component_name": row["component_name"],
                "mesh_file": row["mesh_file"],
                "status": "exported",
                "from_visual_link_config": "1",
            }
            for key in (
                "origin_x_cm",
                "origin_y_cm",
                "origin_z_cm",
                "m00",
                "m01",
                "m02",
                "m03",
                "m10",
                "m11",
                "m12",
                "m13",
                "m20",
                "m21",
                "m22",
                "m23",
                "m30",
                "m31",
                "m32",
                "m33",
            ):
                normalized[key] = row[key]
            rows.append(normalized)
        return rows

    rows = list(csv.DictReader(OCCURRENCES.open()))
    if MANUAL_OCCURRENCES.exists():
        rows.extend(csv.DictReader(MANUAL_OCCURRENCES.open()))
    return rows


def main() -> None:
    rows = load_rows()
    exported_paths = {
        row["full_path_name"]
        for row in rows
        if row["status"] == "exported" and (MESH_DIR / row["mesh_file"]).exists()
    }

    lines = ['<?xml version="1.0"?>', '<robot name="robot_arm_fusion_visual">']
    lines.append('  <material name="fusion_gray"><color rgba="0.55 0.55 0.55 1"/></material>')
    lines.append('  <link name="world"/>')

    for i, row in enumerate(rows):
        mesh_file = row["mesh_file"]
        mesh_path = MESH_DIR / mesh_file
        if row["status"] != "exported" or not mesh_path.exists():
            continue
        if not row.get("from_visual_link_config") and should_skip_row(row, exported_paths):
            continue

        name = link_name(i, f"{row.get('row_id', i)}_{mesh_file}")
        mesh_uri = f"../cad/exports/fusion_meshes/{escape(mesh_file)}"
        xyz = occurrence_origin(row)
        rpy = matrix_to_rpy(row)
        lines.extend(
            [
                f'  <link name="{name}">',
                "    <visual>",
                f'      <geometry><mesh filename="{mesh_uri}" scale="0.001 0.001 0.001"/></geometry>',
                '      <material name="fusion_gray"/>',
                "    </visual>",
                "  </link>",
                f'  <joint name="{name}_fixed" type="fixed">',
                '    <parent link="world"/>',
                f'    <child link="{name}"/>',
                f'    <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"/>',
                "  </joint>",
            ]
        )

    lines.append("</robot>")
    OUTPUT.write_text("\n".join(lines) + "\n")
    print(OUTPUT)


if __name__ == "__main__":
    main()
