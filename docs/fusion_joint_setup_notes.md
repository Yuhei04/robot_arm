# Fusion CAD joint setup notes

This note records the current Ubuntu/Fusion-to-URDF setup so it can be checked again later.

## Main files

- Joint centers, axes, motor IDs: `cad/exports/fusion_tables/joints_template.csv`
- Mesh-to-link grouping: `cad/exports/fusion_tables/link_assignment_template.csv`
- Hidden/unwanted mesh filters: `cad/exports/fusion_tables/excluded_mesh_patterns.txt`
- Fusion occurrence transforms: `cad/exports/fusion_tables/occurrences.csv`
- Jointed URDF generator: `tools/build_fusion_jointed_urdf.py`
- PyBullet UI viewer: `tools/view_urdf_pybullet.py`
- PyBullet PNG renderer: `tools/render_urdf_pybullet.py`
- Generated URDF: `urdf/robot_arm_fusion.urdf`
- Check images: `outputs/`

## Current link chain

The current joint tree is:

`base_link -> base_yaw_link -> shoulder_pitch_link -> elbow_pitch_link -> elbow_roll_link -> wrist_pitch_link -> wrist_roll_link`

The link names in `link_assignment_template.csv` should match the link names used in `joints_template.csv`. If a new link name is added only to `link_assignment_template.csv`, those parts may be skipped or may not move until the joint tree/generator also knows that link.

## Current joint CSV values

These are the final working values from `joints_template.csv`.

```csv
joint_name,parent_link,child_link,motor_id,axis_x,axis_y,axis_z,origin_x_mm,origin_y_mm,origin_z_mm,sign
base_yaw_joint,base_link,base_yaw_link,1,0,0,1,0,0,0,1
shoulder_pitch_joint,base_yaw_link,shoulder_pitch_link,2,1,0,0,0,0,95.5,1
elbow_pitch_joint,shoulder_pitch_link,elbow_pitch_link,3,1,0,0,0,0,205.45,-1
elbow_roll_joint,elbow_pitch_link,elbow_roll_link,4,0,1,0,-0.02,-28.52,229.45,1
wrist_pitch_joint,elbow_roll_link,wrist_pitch_link,5,-1,0,0,-0.02,-69.02,229.45,-1
wrist_roll_joint,wrist_pitch_link,wrist_roll_link,6,0,1,0,-0.02,-136.52,229.45,-1
```

`origin_x_mm/origin_y_mm/origin_z_mm` are CAD/world coordinates in millimeters. The generator converts them into each parent link frame for URDF.

## Axis notes

- J1 is base yaw.
- J2 is shoulder pitch.
- J3 is elbow pitch. Its center is `(0, 0, 205.45) mm`.
- J4 is elbow roll. It uses the next-link-side `arm_thrustB` center: `(-0.02, -28.52, 229.45) mm`.
- J5 is wrist pitch. The working center is the visible connection-side center: `(-0.02, -69.02, 229.45) mm`.
- J6 is wrist roll. The working horn center is `(-0.02, -136.52, 229.45) mm`; its direction is inverted with `sign=-1`.

For the final correction, J6 was already correct. J5 was fixed by using the visible connection-side center instead of the deeper CAD horn assembly point `(-94.24, -133.41, 83.81) mm`.

## Current tool0

A temporary `tool0` is added to the generated URDF.

- Parent link: `wrist_roll_link`
- Fixed joint: `tool0_fixed_joint`
- Offset from `wrist_roll_link`: `(0.0, -5.4, 0.0) mm`
- URDF origin: `(0, -0.0054, 0) m`
- Orientation: `rpy=(0, 0, -pi/2)`, so tool0 +X points toward the wrist_roll_link distal tip and +Z points upward.
- Meaning: provisional wrist_roll_link distal tip, based on the wrist roll horn mesh negative-Y end.

PyBullet sees it as link `tool0` after the six revolute joints.

## Base Foot Height

The real robot has a 10.25 mm foot/spacer under the base. PyBullet loading scripts apply this as a default base Z offset, so reported `tool0` coordinates are floor/table-frame coordinates by default.

- Default base height: `10.25 mm`
- Override to CAD/base-frame coordinates: add `--base-z-mm 0`
- Zero-pose `tool0` with the foot offset: `(-0.020, -141.920, 239.700) mm`
- Zero-pose `tool0` without the foot offset: `(-0.020, -141.920, 229.450) mm`

## Real-Arm Z Deflection

The geometry/FK is considered basically correct. On the real arm, measured Z is about 8 mm lower than the PyBullet value because of gravity deflection. Do not bake this into the URDF joint geometry; treat it as a real-arm compensation/measurement offset.

- Simulation Z: `z_mm`
- Expected real Z: `z_mm - 8 mm`
- Generate a comparison table with this correction:

```bash
make tool0-table-real
```

The output is `outputs/tool0_pose_table_real_expected.csv`, with an extra `real_expected_z_mm` column.

## Useful commands

Generate the jointed URDF:

```bash
make fusion-jointed
```

Open the interactive PyBullet UI:

```bash
make view-fusion-urdf
```

Print the current `tool0` pose from the generated URDF:

```bash
make tool0-pose
```

Generate a small FK table for standard check poses:

```bash
make tool0-table
```

The output is written to `outputs/tool0_pose_table.csv`. This is the PyBullet reference table to compare with real-arm measurements. By default it includes the 10.25 mm base foot height.

Solve the first position-only IK target for `tool0`:

```bash
make ik-tool0
```

Generate a short Cartesian line trajectory as joint-angle CSV:

```bash
make tool0-line
```

The output is `outputs/tool0_line_trajectory.csv`. Latest 3D trajectory check: 47 rows, max error about 1.45 mm, max joint step about 0.60 deg. The default sample trajectory is now a larger visible 3D move from `(-0.020, -141.920, 239.700)` to `(49.980, -111.920, 274.700)`: X +50 mm, Y +30 mm, Z +35 mm. Check `error_mm`, `max_joint_step_deg`, and `status` before sending anything to the real arm.

Print `tool0` for a specific motor pose:

```bash
.venv-ubuntu/bin/python tools/print_tool0_pose.py --urdf urdf/robot_arm_fusion.urdf --j1 20 --j2 20 --j3 20 --j4 10 --j5 10 --j6 10
```

Render the standard check PNG:

```bash
make render-fusion-urdf
```

Render a single-axis check pose, for example J5 at 45 degrees:

```bash
.venv-ubuntu/bin/python tools/render_urdf_pybullet.py --urdf urdf/robot_arm_fusion.urdf --output outputs/robot_arm_fusion_j5_check.png --j5 45
```

Replay the generated trajectory in PyBullet before sending it to OpenCR:

```bash
make view-tool0-line
```

This opens a UI, applies each CSV row to the robot, and shows the moving `tool0` marker. Target points are drawn as yellow cross/line markers. Use this before `--execute` on the real arm. To validate without opening a GUI:

```bash
.venv-ubuntu/bin/python tools/view_tool0_trajectory.py --urdf urdf/robot_arm_fusion.urdf --csv outputs/tool0_line_trajectory.csv --check-only
```

## Sending a Trajectory to OpenCR

The generated Cartesian line trajectory can be converted to OpenCR debug-serial commands. The sender is dry-run by default. The preferred command mode is now `multi`, which prints one `q j1 j2 j3 j4 j5 j6` command per CSV row so all joints receive each trajectory row together. Use `--command-mode per-joint` only for the older `m <id> <deg>` style.

Dry-run only, no robot motion:

```bash
make send-trajectory-dry-run
```

Execute on the real arm only after checking the CSV and dry-run output:

```bash
.venv-ubuntu/bin/python tools/send_joint_trajectory_opencr.py --csv outputs/tool0_line_trajectory.csv --skip-first --execute --port /dev/ttyACM0 --delay 0.12
```

Safety checks in the PC sender:

- Default behavior is warning-only, not rejection.
- Joint range warning defaults to `-180..180 deg`; change with `--joint-min-deg` and `--joint-max-deg`.
- Per-row joint step warning defaults to `10 deg`; change with `--max-step-deg`.
- Add `--strict-safety` to reject warnings instead of only printing them.
- Non-`OK` trajectory rows warn by default; add `--strict-safety` to reject them, or `--allow-non-ok-status` to explicitly allow them.
- Requires `--execute` before opening the serial port.

## Current-Pose Real-Arm Test Trajectory

The real arm was read at approximately:

```text
j1=-0.26, j2=20.39, j3=-0.26, j4=0.00, j5=0.09, j6=0.00 deg
```

The corresponding simulated `tool0` pose is approximately:

```text
x=-0.835, y=-179.531, z=181.112 mm
```

A small current-pose trajectory was generated from that pose to:

```text
x=19.165, y=-169.531, z=191.112 mm
```

This is a small 3D move: X +20 mm, Y +10 mm, Z +10 mm.

- CSV: `outputs/tool0_line_trajectory_current.csv`
- Rows: 14
- Max error: about 1.46 mm
- Max joint step: about 0.60 deg

Dry-run:

```bash
.venv-ubuntu/bin/python tools/send_joint_trajectory_opencr.py --csv outputs/tool0_line_trajectory_current.csv --skip-first
```

Execute only after checking the PyBullet replay and keeping torque-off/emergency stop ready:

```bash
.venv-ubuntu/bin/python tools/send_joint_trajectory_opencr.py --csv outputs/tool0_line_trajectory_current.csv --skip-first --execute --port /dev/ttyACM0
```

## Existing check images

- `outputs/robot_arm_fusion_ubuntu_check.png`: standard rendered preview
- `outputs/robot_arm_fusion_j4_thrustB_center.png`: J4 at 45 degrees after setting the thrustB center
- `outputs/robot_arm_fusion_j5_center_retry.png`: J5 at 45 degrees after the final center fix
- `outputs/robot_arm_fusion_j6_horn_center.png`: J6 at 45 degrees with the horn center

## When CAD is exported again

1. Copy or sync the newest Fusion mesh/table export into `cad/exports/fusion_meshes` and `cad/exports/fusion_tables`.
2. Re-check `link_assignment_template.csv` so the link names match the current joint chain.
3. Keep `joints_template.csv` origins in CAD/world mm unless the physical CAD assembly changes.
4. Rebuild with `make fusion-jointed`.
5. Check motion with `make view-fusion-urdf` or single-axis render commands.

## Current status

As of this note, visual grouping and J1-J6 motion were checked manually and reported as working.

### Larger and smoother trajectory check

For UI inspection from the zero/CAD pose:

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make tool0-line
make view-tool0-line
```

For the current real-arm starting pose that was read on 2026-06-07 (J1=-0.26, J2=20.39, J3=-0.26, J4=0.00, J5=0.09, J6=0.00 deg):

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make current-tool0-line
make view-current-tool0-line
make send-current-trajectory-dry-run
```

The current-pose trajectory writes `outputs/tool0_line_trajectory_current.csv`. Latest check: 44 points, max error about 1.46 mm, max joint step about 0.47 deg. Dry-run prints `command_mode: multi` and `q ...` rows.

To use `q` on the real OpenCR, compile/upload the updated sketch first:

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make build
make upload PORT=/dev/ttyACM0
```

Then send the current-pose trajectory only after the dry-run looks correct and the arm area is clear:

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
.venv-ubuntu/bin/python tools/send_joint_trajectory_opencr.py --csv outputs/tool0_line_trajectory_current.csv --skip-first --execute --port /dev/ttyACM0 --delay 0.12
```

## Coordinate Frame Used for Tool0 Commands

The point-command CSVs such as `outputs/tool0_cube_points.csv` use the PyBullet world frame in millimeters. The robot is loaded with the base fixed at world origin, plus the default real foot/spacer height of `10.25 mm` in Z.

Frame convention for the current setup:

- Origin: base center/world origin, with the real base foot height included by default.
- `+Z`: upward.
- `X/Y`: horizontal table plane, inherited from the Fusion CAD/world frame.
- At the zero pose, `tool0` is approximately `x=-0.020, y=-141.920, z=239.700 mm`.
- The current temporary `tool0` frame has `+X` pointing toward the wrist_roll_link distal tip and `+Z` upward.

So a waypoint like:

```csv
x_mm,y_mm,z_mm
0,-190,160
```

means: keep tool0 on the world X=0 plane, place it at Y=-190 mm from the base/world origin, and Z=160 mm above the table/floor frame including the 10.25 mm foot offset.

To work in the CAD/base frame without the foot offset, run tools with `--base-z-mm 0`.

## Joint Limits

Joint motor-angle limits are now centralized here:

```text
cad/exports/fusion_tables/joint_limits.csv
```

Current provisional limits:

```csv
joint,motor_id,min_deg,max_deg
j1,1,-90,90
j2,2,-90,90
j3,3,-90,90
j4,4,-90,90
j5,5,-90,90
j6,6,-90,90
```

These limits are motor-command degrees, not URDF joint-space degrees. The helper `tools/joint_limits.py` converts them through each joint's sign when needed. The following tools now read this CSV:

- `tools/build_fusion_jointed_urdf.py` for URDF joint limits
- `tools/move_tool0_points.py` for point-command IK
- `tools/manual_yz_follow_ui.py` for manual slider IK
- `tools/hand_yz_follow_opencr.py` for camera-follow IK
- `tools/tool0_line_trajectory.py` and `tools/ik_tool0_position.py`

OpenCR still has its own firmware-side hard limit arrays in `sketches/opencr_dxl_check/opencr_dxl_check.ino`. Keep that firmware limit equal to or wider than this CSV. For real safety, narrow both after measuring each physical joint's actual safe range.
