# Robot Arm Control Goal

The current long-term goal is to make the robot arm move smoothly using forward kinematics, inverse kinematics, and trajectory generation, then use camera-based human arm/hand skeleton estimation to operate the robot by moving a human arm.

After that, attach the robot hand from the separate hand project to the end of this arm, use the same camera-based operation approach, and grasp a paper cup.

## Current Milestone

The current arm-only milestone is:

1. Keep the Fusion-derived URDF aligned with the real robot.
2. Use `tool0` as the control point at the wrist roll distal tip.
3. Confirm FK against the real arm.
4. Generate position-only IK for small `tool0` moves.
5. Generate safe joint-angle trajectories from Cartesian targets.
6. Only then send slow, limited joint targets to the real OpenCR/DYNAMIXEL arm.

## Notes

- The real robot has a 10.25 mm foot/spacer under the base.
- The real measured Z is about 8 mm lower than PyBullet/FK because of gravity deflection.
- Do not bake the gravity deflection into URDF geometry; keep it as a measurement/compensation term.
- For tool orientation, `tool0 +X` points toward the wrist-roll distal tip and `tool0 +Z` points upward.

## Latest Progress

- Added position-only IK for `tool0`: `tools/ik_tool0_position.py`.
- Added Cartesian line trajectory generation: `tools/tool0_line_trajectory.py`.
- Added dry-run/execute OpenCR trajectory sender: `tools/send_joint_trajectory_opencr.py`.
- The next real-arm step is to run `make send-trajectory-dry-run`, inspect commands, then execute with a very small trajectory and hand near the emergency torque-off path.

Safety checks are warning-only by default in `send_joint_trajectory_opencr.py`; use `--strict-safety` when you want warnings to reject execution.

- Added PyBullet trajectory replay before real-arm execution: `tools/view_tool0_trajectory.py`.
