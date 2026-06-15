#!/usr/bin/env python3
"""Create an editable visual/link configuration CSV.

This CSV is the user-facing source of truth for:
  - which exported CAD meshes should be shown
  - which robot link each mesh belongs to

Rows with transform data come from occurrences.csv. Mesh files that exist on
disk but are missing from occurrences.csv are listed as source=orphan_mesh so
they are visible during review, but they cannot be included until transform
columns are filled in or the Fusion export is fixed.
"""

import argparse
import csv
from pathlib import Path

from create_link_assignment_template import guess_link, should_skip_row


ROOT = Path(__file__).resolve().parents[1]
TABLE_DIR = ROOT / "cad" / "exports" / "fusion_tables"
MESH_DIR = ROOT / "cad" / "exports" / "fusion_meshes"
OCCURRENCES = TABLE_DIR / "occurrences.csv"
OUTPUT = TABLE_DIR / "visual_link_config.csv"
PRESERVE_EXISTING = False

TRANSFORM_FIELDS = [
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
]

FIELDNAMES = [
    "row_id",
    "include",
    "link_name",
    "source",
    "occurrence_name",
    "full_path_name",
    "component_name",
    "mesh_file",
    "status",
    *TRANSFORM_FIELDS,
    "note",
]


def load_existing_choices() -> dict[tuple[str, str], dict[str, str]]:
    if not PRESERVE_EXISTING:
        return {}
    if not OUTPUT.exists():
        return {}
    choices = {}
    for row in csv.DictReader(OUTPUT.open()):
        if row.get("row_id"):
            choices[("row_id", row["row_id"])] = row
        choices[(row["full_path_name"], row["mesh_file"])] = row
    return choices


def occurrence_rows() -> list[dict[str, str]]:
    rows = list(csv.DictReader(OCCURRENCES.open()))
    exported_paths = {
        row["full_path_name"]
        for row in rows
        if row["status"] == "exported" and (MESH_DIR / row["mesh_file"]).exists()
    }
    existing = load_existing_choices()
    output_rows = []

    for index, row in enumerate(rows):
        mesh_file = row["mesh_file"]
        if row["status"] != "exported" or not (MESH_DIR / mesh_file).exists():
            default_include = "0"
            note = "not exported or missing mesh"
        elif should_skip_row(row, exported_paths):
            default_include = "0"
            note = "default excluded by mesh filter"
        else:
            default_include = "1"
            note = ""

        key = (row["full_path_name"], mesh_file)
        row_id = f"occ_{index:03d}"
        previous = existing.get(("row_id", row_id), existing.get(key, {}))
        output_rows.append(
            {
                "row_id": row_id,
                "include": previous.get("include", default_include),
                "link_name": previous.get(
                    "link_name",
                    guess_link(row["component_name"], row["full_path_name"]),
                ),
                "source": "occurrences",
                "occurrence_name": row["occurrence_name"],
                "full_path_name": row["full_path_name"],
                "component_name": row["component_name"],
                "mesh_file": mesh_file,
                "status": row["status"],
                **{field: row.get(field, "") for field in TRANSFORM_FIELDS},
                "note": previous.get("note", note),
            }
        )
    return output_rows


def orphan_mesh_rows(known_meshes: set[str]) -> list[dict[str, str]]:
    existing = load_existing_choices()
    rows = []
    for index, mesh_path in enumerate(sorted(MESH_DIR.glob("*.stl"))):
        mesh_file = mesh_path.name
        if mesh_file in known_meshes:
            continue
        key = (f"orphan:{mesh_file}", mesh_file)
        row_id = f"orphan_{index:03d}"
        previous = existing.get(("row_id", row_id), existing.get(key, {}))
        has_transform = bool(previous.get("origin_x_cm") and previous.get("m00"))
        rows.append(
            {
                "row_id": row_id,
                "include": previous.get("include", "0") if has_transform else "0",
                "link_name": previous.get("link_name", ""),
                "source": "orphan_mesh",
                "occurrence_name": "",
                "full_path_name": key[0],
                "component_name": "",
                "mesh_file": mesh_file,
                "status": "mesh_without_occurrence",
                **{field: previous.get(field, "") for field in TRANSFORM_FIELDS},
                "note": previous.get(
                    "note",
                    "mesh exists, but occurrences.csv has no transform for it",
                ),
            }
        )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Create visual_link_config.csv from Fusion export data.")
    parser.add_argument(
        "--preserve",
        action="store_true",
        help="Preserve existing include/link/note edits when row_id or mesh path matches.",
    )
    args = parser.parse_args()

    global PRESERVE_EXISTING
    PRESERVE_EXISTING = args.preserve

    rows = occurrence_rows()
    known_meshes = {row["mesh_file"] for row in rows}
    rows.extend(orphan_mesh_rows(known_meshes))
    rows.sort(key=lambda row: (row["mesh_file"], row["source"], row["row_id"]))

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    print(OUTPUT)


if __name__ == "__main__":
    main()
