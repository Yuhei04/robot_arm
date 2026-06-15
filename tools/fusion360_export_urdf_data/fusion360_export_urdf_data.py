# Run this inside Fusion 360:
#   Utilities > Scripts and Add-Ins > Scripts > + > select this folder
#
# It exports visible occurrences as STL files and writes a basic occurrence list.
# Joint origins/axes should still be checked in Fusion and completed in joints.csv.

import csv
import traceback
from pathlib import Path

import adsk.core
import adsk.fusion


OUTPUT_DIR = Path.home() / "Documents" / "robot_arm" / "cad" / "exports"
MESH_DIR = OUTPUT_DIR / "fusion_meshes"
TABLE_DIR = OUTPUT_DIR / "fusion_tables"


def safe_name(name):
    keep = []
    for ch in name:
        if ch.isalnum() or ch in ("_", "-"):
            keep.append(ch)
        else:
            keep.append("_")
    return "".join(keep).strip("_") or "unnamed"


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface

    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open the robot arm Fusion design first.")
            return

        MESH_DIR.mkdir(parents=True, exist_ok=True)
        TABLE_DIR.mkdir(parents=True, exist_ok=True)

        export_manager = design.exportManager
        root = design.rootComponent

        rows = []
        exported = 0
        for occ in root.allOccurrences:
            if not occ.isLightBulbOn:
                continue

            name = safe_name(occ.fullPathName.replace("+", "_"))
            mesh_path = MESH_DIR / f"{name}.stl"

            status = "not_exported"
            try:
                options = export_manager.createSTLExportOptions(occ, str(mesh_path))
                options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                export_manager.execute(options)
                status = "exported"
                exported += 1
            except Exception as exc:
                status = f"export_failed: {exc}"

            # transform2 is the assembly-context transform. For nested
            # occurrences, transform can be local to the parent occurrence,
            # which places motor sub-parts near the origin in the URDF.
            transform = getattr(occ, "transform2", None) or occ.transform
            translation = transform.translation
            row = {
                "occurrence_name": occ.name,
                "full_path_name": occ.fullPathName,
                "component_name": occ.component.name,
                "mesh_file": mesh_path.name,
                "status": status,
                "origin_x_cm": translation.x,
                "origin_y_cm": translation.y,
                "origin_z_cm": translation.z,
            }
            for r in range(4):
                for c in range(4):
                    row[f"m{r}{c}"] = transform.getCell(r, c)
            rows.append(row)

        occurrence_csv = TABLE_DIR / "occurrences.csv"
        with occurrence_csv.open("w", newline="") as f:
            fieldnames = [
                "occurrence_name",
                "full_path_name",
                "component_name",
                "mesh_file",
                "status",
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
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        joints_csv = TABLE_DIR / "joints_template.csv"
        if not joints_csv.exists():
            with joints_csv.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "joint_name",
                        "parent_link",
                        "child_link",
                        "motor_id",
                        "axis_x",
                        "axis_y",
                        "axis_z",
                        "origin_x_mm",
                        "origin_y_mm",
                        "origin_z_mm",
                        "sign",
                    ]
                )
                writer.writerow(["base_yaw_joint", "base_link", "base_yaw_link", 1, 0, 0, 1, 0, 0, 0, 1])
                writer.writerow(["shoulder_pitch_joint", "base_yaw_link", "shoulder_pitch_link", 2, 0, 1, 0, "", "", "", 1])
                writer.writerow(["upper_arm_roll_joint", "shoulder_pitch_link", "upper_arm_link", 6, 0, 0, -1, "", "", "", -1])
                writer.writerow(["elbow_pitch_joint", "upper_arm_link", "forearm_link", 3, 0, 1, 0, "", "", "", -1])
                writer.writerow(["elbow_roll_joint", "forearm_link", "elbow_roll_link", 4, -1, 0, 0, "", "", "", -1])
                writer.writerow(["wrist_pitch_joint", "elbow_roll_link", "wrist_pitch_link", 5, 0, 1, 0, "", "", "", 1])

        ui.messageBox(
            "Fusion export finished.\n"
            f"Exported meshes: {exported}\n"
            f"Occurrences CSV: {occurrence_csv}\n"
            f"Joint template: {joints_csv}"
        )

    except Exception:
        ui.messageBox(traceback.format_exc())
