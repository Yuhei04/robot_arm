# Pose Teaching Workflow

This workflow records real robot joint angles as named poses, checks them in PyBullet, then replays them on the real robot.

The pose CSV is:

```text
outputs/taught_poses.csv
```

Columns:

```csv
name,j1,j2,j3,j4,j5,j6,hand,wait_s,notes
```

- `name`: pose name such as `approach`, `grasp`, `lift`, `release`
- `j1..j6`: motor-command angles in degrees
- `hand`: empty, `open`, `close`, or a raw ESP32-C3 command
- `wait_s`: delay after this row during replay
- `notes`: free-form notes

## 1. Record A Pose

Move the robot by hand to the desired pose, then record current angles:

```bash
make pose-record POSE_NAME=approach PORT=/dev/ttyACM0
```

Record a pose that also opens or closes the hand during replay:

```bash
make pose-record POSE_NAME=grasp HAND_ACTION=close PORT=/dev/ttyACM0
```

## 2. Check In PyBullet

Use this before moving the real robot:

```bash
make pose-view
```

PyBullet replays `outputs/taught_poses.csv` with the same joint angles.

## 3. Dry Run Replay

Print the exact arm and hand commands without sending them:

```bash
make pose-play-dry-run
```

## 4. Replay On The Real Robot

Run this only after PyBullet and dry-run look correct:

```bash
make pose-play PORT=/dev/ttyACM0
```

If the hand ESP32-C3 is connected over serial, pass its port:

```bash
make pose-play PORT=/dev/ttyACM0 HAND_PORT=/dev/ttyUSB0
```

By default, `hand=open` sends `open`, and `hand=close` sends `close`. Override the raw commands if the ESP32-C3 firmware uses different text:

```bash
make pose-play PORT=/dev/ttyACM0 HAND_PORT=/dev/ttyUSB0 HAND_OPEN_COMMAND=o HAND_CLOSE_COMMAND=c
```

## Typical Pick And Place Shape

Use rows like this:

```csv
name,j1,j2,j3,j4,j5,j6,hand,wait_s,notes
home,0,0,0,0,0,0,open,1.0,start open
approach,10,20,-15,5,30,0,,1.0,near object
grasp,10,25,-20,5,30,0,close,0.8,close hand
lift,5,10,-10,5,25,0,,1.0,lift object
release,-10,15,-10,5,25,0,open,0.8,release object
```

The same CSV is used for PyBullet and real replay, so the basic loop is:

```bash
make pose-record POSE_NAME=...
make pose-view
make pose-play-dry-run
make pose-play PORT=/dev/ttyACM0
```
