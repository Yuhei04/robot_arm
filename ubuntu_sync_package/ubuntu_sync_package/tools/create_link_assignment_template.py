#!/usr/bin/env python3
"""Create a link assignment template from Fusion occurrence exports."""

import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OCCURRENCES = ROOT / "cad" / "exports" / "fusion_tables" / "occurrences.csv"
MANUAL_OCCURRENCES = ROOT / "cad" / "exports" / "fusion_tables" / "manual_occurrences.csv"
VISUAL_LINK_CONFIG = ROOT / "cad" / "exports" / "fusion_tables" / "visual_link_config.csv"
OUTPUT = ROOT / "cad" / "exports" / "fusion_tables" / "link_assignment_template.csv"
MESH_DIR = ROOT / "cad" / "exports" / "fusion_meshes"


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


def guess_link(component_name: str, full_path: str) -> str:
    text = f"{component_name} {full_path}".lower()
    if "arm_part1" in text:
        return "linkC"
    if "arm_thrustc" in text:
        return "linkD"
    if "xm,h-430_idler:1" in text:
        return "linkB"
    if "xm,h-430_idler:2" in text:
        return "linkC"
    if "xm,h-430_idler:3" in text:
        return "linkD"
    if "xm,h-430_idler:4" in text:
        return "linkG"
    if "arm_plate" in text:
        return "linkA"
    if "arm_thrusta" in text:
        return "linkB"
    if "arm_part1_v2" in text or "fr12_s102_k:1" in text:
        return "linkC"
    if "fr12_h101_k:1" in text or "arm_hand_v2:2" in text or "arm_thrustd" in text:
        return "linkD"
    if "arm_thrustb" in text or "fr12_s102_k:2" in text:
        return "linkD"
    if "xc-430_idle:1" in text or "arm_hand_v2:1" in text or "fr12_h101_k:3" in text:
        return "linkE"
    if "xc-430_idle:2" in text:
        return "linkG"
    if "m_dc11_a01_horn" in text:
        return "linkG"
    return ""


def load_rows() -> list[dict[str, str]]:
    rows = list(csv.DictReader(OCCURRENCES.open()))
    if MANUAL_OCCURRENCES.exists():
        rows.extend(csv.DictReader(MANUAL_OCCURRENCES.open()))
    return rows


def main() -> None:
    if VISUAL_LINK_CONFIG.exists():
        fieldnames = [
            "row_id",
            "link_name",
            "occurrence_name",
            "full_path_name",
            "component_name",
            "mesh_file",
            "status",
        ]
        with OUTPUT.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in csv.DictReader(VISUAL_LINK_CONFIG.open()):
                if row.get("include", "").strip() not in ("1", "true", "TRUE", "yes", "YES"):
                    continue
                if not row.get("link_name", "").strip():
                    continue
                if not row.get("origin_x_cm") or not row.get("m00"):
                    continue
                if not (MESH_DIR / row["mesh_file"]).exists():
                    continue
                writer.writerow(
                    {
                        "row_id": row["row_id"],
                        "link_name": row["link_name"].strip(),
                        "occurrence_name": row["occurrence_name"],
                        "full_path_name": row["full_path_name"],
                        "component_name": row["component_name"],
                        "mesh_file": row["mesh_file"],
                        "status": "exported",
                    }
                )
        print(OUTPUT)
        return

    rows = load_rows()
    exported_paths = {
        row["full_path_name"]
        for row in rows
        if row["status"] == "exported" and (MESH_DIR / row["mesh_file"]).exists()
    }
    fieldnames = [
        "row_id",
        "link_name",
        "occurrence_name",
        "full_path_name",
        "component_name",
        "mesh_file",
        "status",
    ]
    with OUTPUT.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for index, row in enumerate(rows):
            if row["status"] != "exported" or not (MESH_DIR / row["mesh_file"]).exists():
                continue
            if should_skip_row(row, exported_paths):
                continue
            writer.writerow(
                {
                    "row_id": f"occ_{index:03d}",
                    "link_name": guess_link(row["component_name"], row["full_path_name"]),
                    "occurrence_name": row["occurrence_name"],
                    "full_path_name": row["full_path_name"],
                    "component_name": row["component_name"],
                    "mesh_file": row["mesh_file"],
                    "status": row["status"],
                }
            )
    print(OUTPUT)


if __name__ == "__main__":
    main()
