# Tool0 Point Workflow

This is the normal workflow for explicit tool0 XYZ point moves.

Use this order every time:

1. Create or edit a point CSV.
2. Check the IK result without opening PyBullet.
3. View the same points in the PyBullet GUI.
4. Send the same points to the real robot.

## Point CSV

The CSV must have these columns:

```csv
x_mm,y_mm,z_mm
-50,-200,160
40,-200,160
40,-120,160
-50,-120,160
```

Coordinates are in the PyBullet/world frame, in millimeters. The base is at the world origin, and scripts use the real base foot/spacer height of `10.25 mm` in Z by default.

Common files:

- `outputs/tool0_rectangle_points.csv`
- `outputs/tool0_cube_points.csv`

## 1. Check IK

Run this first. It does not move the real robot.

```bash
make points-check POINTS_CSV=outputs/tool0_cube_points.csv
```

Look at `err=...mm` and `jump=...deg`. Large error means the target is hard or impossible under the current joint limits. Large jump means the arm may move sharply between rows.

## 2. View In PyBullet

Use this to confirm that the IK result matches the intended motion visually.

```bash
make points-view POINTS_CSV=outputs/tool0_cube_points.csv
```

In the PyBullet GUI:

- yellow points are requested target waypoints
- red marker is the solved FK position of `tool0`
- red line shows IK error

## 3. Dry Run Real Commands

This prints the same `q j1 j2 j3 j4 j5 j6` commands that would be sent, but does not execute them.

```bash
make points-dry-run POINTS_CSV=outputs/tool0_cube_points.csv
```

## 4. Move The Real Robot

Only run this after the check and PyBullet view look correct.

```bash
make points-run POINTS_CSV=outputs/tool0_cube_points.csv PORT=/dev/ttyACM0
```

The real-arm command uses the same IK output as the PyBullet check. If PyBullet and the real arm differ, first confirm the current motor angles:

```bash
make opencr-angles PORT=/dev/ttyACM0
```

## Short Aliases

Default point CSV is controlled by `POINTS_CSV` in the Makefile. With the default file:

```bash
make points-check
make points-view
make points-run PORT=/dev/ttyACM0
```

For the cube file, these aliases are also available:

```bash
make cube-check
make cube-view
make cube-run PORT=/dev/ttyACM0
```

## Related Files

- `tools/move_tool0_points.py`: solves IK for each CSV row and optionally sends commands to OpenCR.
- `tools/view_tool0_points_ik.py`: checks or displays the same IK result in PyBullet.
- `tools/build_fusion_jointed_urdf.py`: generates `urdf/robot_arm_fusion.urdf` from Fusion mesh/link/joint tables.
- `cad/exports/fusion_tables/joint_limits.csv`: joint limit definitions used by IK and URDF generation.
- `cad/exports/fusion_tables/joints_template.csv`: joint axes, origins, signs, and motor IDs.
