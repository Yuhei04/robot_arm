# Hand Y/Z Camera Follow

This is the side-view camera control setup. X is fixed, and the camera reads hand position as robot Y/Z. The normal input is the Mac camera HTTP MJPEG stream, not RealSense.

## Files

- Detector and UI: `tools/hand_yz_tracker.py`
- Real-time dry-run/execute follower: `tools/hand_yz_follow_opencr.py`
- Latest target JSON: `outputs/latest_hand_yz.json`
- Detector CSV log: `outputs/hand_yz_targets.csv`

## Install state

The Ubuntu venv `.venv-ubuntu` now includes:

- `numpy`
- `opencv-python`
- `mediapipe`
- `pyserial`

## Mac camera stream

Start the camera stream on the Mac:

```bash
ffmpeg -f avfoundation -framerate 30 -video_size 640x480 -i "0" \
  -f mpjpeg -listen 1 http://0.0.0.0:8080/video
```

The current default Ubuntu source is:

```text
http://100.82.6.16:8080/video
```

`tools/hand_yz_tracker.py` and `tools/hand_yz_follow_opencr.py` accept local camera indexes, video files, HTTP MJPEG URLs, and RTSP URLs. Numeric `--source` values such as `0` and `1` are treated as local camera indexes. URL values such as `http://...` and `rtsp://...` are passed directly to `cv2.VideoCapture(source)`; no RealSense or `/dev/video*` specific path is required.

## Step 1: camera/video detection only

Mac camera stream:

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make hand-yz-camera
```

Local Ubuntu camera, if needed:

```bash
make hand-yz-camera HAND_SOURCE=0
```

Video file:

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make hand-yz-video VIDEO=/path/to/side_view.mp4
```

The green marker is the tracked hand landmark. The yellow cross is the pixel origin. Press `q` or `Esc` to close.

## Coordinate mapping

Default mapping is simple side-view affine mapping:

- image X -> robot Y
- image Y upward -> robot Z
- robot X is not estimated here

Defaults:

- `HAND_ORIGIN_Y_MM=-160`
- `HAND_ORIGIN_Z_MM=190`
- `HAND_MM_PER_PX=0.7`

Example with a wider motion scale:

```bash
make hand-yz-camera HAND_MM_PER_PX=1.0
```

If the direction is reversed, call the Python script directly with `--flip-y` or `--flip-z`:

```bash
.venv-ubuntu/bin/python tools/hand_yz_tracker.py --source http://100.82.6.16:8080/video --flip-y
```

## Step 2: real-time follower dry-run

This computes IK per camera frame and prints `q j1 j2 j3 j4 j5 j6` commands. It does not move the arm unless `--execute` is used.

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make hand-follow-dry-run
```

Defaults are conservative:

- command rate: `5 Hz`
- per-command joint clamp: `1.5 deg`
- deadband: `4 mm`
- X target: `0 mm`

## Step 3: real-arm execution

Run this only after the dry-run target values and commands look reasonable, the arm area is clear, and OpenCR has the `q` command sketch uploaded.

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make hand-follow-execute PORT=/dev/ttyACM0
```

If the real current angles have changed, read them first:

```bash
make opencr-angles PORT=/dev/ttyACM0
```

Then pass them as Make variables, for example:

```bash
make hand-follow-dry-run HAND_SEED_J1=17.1 HAND_SEED_J2=10.5 HAND_SEED_J3=-3.0 HAND_SEED_J4=-0.2 HAND_SEED_J5=1.5 HAND_SEED_J6=0.0
```

## Next recommended work

1. Confirm the camera side view and choose whether `wrist` or `index_tip` is easier to control.
2. Tune `HAND_MM_PER_PX`, origin Y/Z, and optional flips until the displayed Y/Z values feel intuitive.
3. Dry-run the follower and check the printed `q` commands are small and stable.
4. Execute at low speed only after the dry-run behaves well.

## Responsiveness tuning

Live camera sources use a latest-frame reader by default. It continuously reads the Mac HTTP MJPEG stream and drops stale buffered frames, so MediaPipe processes the newest available image instead of old queued frames. Disable this only for debugging with `--keep-buffered-frames`.

The follower defaults were tuned away from the earlier choppy setting:

- `HAND_RATE_HZ=12`
- `HAND_DEADBAND_MM=1.0`
- `HAND_MAX_STEP_DEG=1.0`
- `HAND_EMA_ALPHA=0.30`

If the arm feels delayed, try a little less smoothing or a higher command rate:

```bash
make hand-follow-dry-run HAND_EMA_ALPHA=0.45 HAND_RATE_HZ=15
```

If the arm jitters, add smoothing or deadband:

```bash
make hand-follow-dry-run HAND_EMA_ALPHA=0.20 HAND_DEADBAND_MM=2.0
```

If the arm follows too slowly even with stable detection, allow larger per-command joint steps:

```bash
make hand-follow-dry-run HAND_MAX_STEP_DEG=1.5
```

## Lag/crash stabilization

If the camera feels laggy or the tracker exits, first use the lighter defaults now in the Makefile:

- MediaPipe processing width: `HAND_PROCESS_WIDTH=320`
- Follower rate: `HAND_RATE_HZ=8`
- Per-command joint step: `HAND_MAX_STEP_DEG=1.2`
- Follower skips MediaPipe skeleton drawing by default
- Live streams keep only the newest frame and tolerate brief frame stalls

Run detection only:

```bash
make hand-yz-camera
```

Run follower dry-run:

```bash
make hand-follow-dry-run
```

If it is still heavy, reduce processing width further:

```bash
make hand-yz-camera HAND_PROCESS_WIDTH=240
make hand-follow-dry-run HAND_PROCESS_WIDTH=240 HAND_RATE_HZ=6
```

A lower-load Mac stream is also recommended over 640x480/30fps:

```bash
ffmpeg -f avfoundation -framerate 15 -video_size 320x240 -i "0" \
  -f mpjpeg -listen 1 http://0.0.0.0:8080/video
```

If the latest-frame thread seems unstable with a specific OpenCV/backend combination, fall back to buffered reads for debugging:

```bash
.venv-ubuntu/bin/python tools/hand_yz_tracker.py --source http://100.82.6.16:8080/video --keep-buffered-frames
```

## Manual Y/Z slider test

Use this when camera input might be the problem. It removes MediaPipe and video capture completely, then drives the same fixed-X IK and `q` command path from OpenCV trackbars.

Dry-run UI:

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make manual-yz-dry-run
```

Real-arm UI:

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make manual-yz-execute PORT=/dev/ttyACM0
```

Defaults:

- Fixed X: `HAND_FIXED_X_MM=0`
- Y range: `-240..-80 mm`
- Z range: `120..280 mm`
- Initial target: `Y=-160 mm, Z=190 mm`
- Command rate: `HAND_RATE_HZ=8`
- Per-command joint step: `HAND_MAX_STEP_DEG=1.2`
- Deadband: `HAND_DEADBAND_MM=1.0`

Example, slower and gentler:

```bash
make manual-yz-execute PORT=/dev/ttyACM0 HAND_RATE_HZ=5 HAND_MAX_STEP_DEG=0.8
```

Example, wider Y/Z range:

```bash
make manual-yz-dry-run MANUAL_Y_MIN_MM=-280 MANUAL_Y_MAX_MM=-60 MANUAL_Z_MIN_MM=100 MANUAL_Z_MAX_MM=300
```

If this manual UI is smooth but camera follow is laggy, the problem is likely camera/Mediapipe input. If this UI is also choppy, focus on IK, command timing, OpenCR, or DYNAMIXEL motion settings.

### Manual Y/Z IK stability note

The manual Y/Z UI now defaults to `MANUAL_ACTIVE_JOINTS=j1,j2,j3`. This keeps J4-J6 fixed during IK so redundant wrist/elbow-roll solutions do not jump between equivalent poses. If you need to test the old full 6-axis IK behavior, run:

```bash
make manual-yz-dry-run MANUAL_ACTIVE_JOINTS=j1,j2,j3,j4,j5,j6
```

Before real-arm execution, read the current angles and pass them as `HAND_SEED_J*` values if they differ from the defaults. A wrong seed can make the first IK command look like a sudden return toward an old pose.

## Explicit Tool0 Point Movement

For debugging, use explicit XYZ waypoints with no interpolation. Each row is solved once with IK and sent as one `q j1 j2 j3 j4 j5 j6` command.

Default rectangle CSV:

```text
outputs/tool0_rectangle_points.csv
```

Current rectangle:

```csv
x_mm,y_mm,z_mm
0,-190,160
0,-110,160
0,-110,240
0,-190,240
0,-190,160
```

Dry-run:

```bash
cd /home/yuhei/Documents/robot_arm/robot_arm
make move-tool0-points-dry-run
```

Real arm execution reads current joint angles from OpenCR first, then sends one command per point:

```bash
make move-tool0-points-execute PORT=/dev/ttyACM0
```

This intentionally does not interpolate between points. If the arm jumps between corners, that is expected for this test; the goal is to see whether each point command behaves correctly and whether IK produces reasonable corner poses.

### Explicit Tool0 Cube Movement

Cube waypoint CSV:

```text
outputs/tool0_cube_points.csv
```

It changes X as well as Y/Z:

- X: `-40..40 mm`
- Y: `-190..-110 mm`
- Z: `160..240 mm`

Dry-run:

```bash
make move-tool0-cube-dry-run
```

Real arm:

```bash
make move-tool0-cube-execute PORT=/dev/ttyACM0
```

Dry-run check on 2026-06-07: IK/FK errors were about 3..7.5 mm, with max joint jumps around 37 deg because this test intentionally sends one command per cube corner with no interpolation.
