#!/usr/bin/env python3
"""Follow a side-view hand Y/Z target with fixed tool0 X.

This is a real-time point follower, not a pre-generated trajectory. Dry-run is
the default; add --execute only after the overlay and printed q commands look
reasonable.
"""

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import mediapipe as mp
import pybullet as p

from joint_limits import read_joint_limits


DEFAULT_URDF = Path(__file__).resolve().parents[1] / "urdf" / "robot_arm_fusion.urdf"
DEFAULT_BASE_Z_MM = 10.25
JOINT_COMMANDS = {
    "j1": ("base_yaw_joint", 1.0),
    "j2": ("shoulder_pitch_joint", 1.0),
    "j3": ("elbow_pitch_joint", -1.0),
    "j4": ("elbow_roll_joint", 1.0),
    "j5": ("wrist_pitch_joint", -1.0),
    "j6": ("wrist_roll_joint", -1.0),
}
MOTOR_LIMITS_DEG = read_joint_limits()
LANDMARKS = {"wrist": 0, "index_tip": 8, "middle_tip": 12}


def parse_source(value: str) -> int | str:
    """Return local camera indexes as int, and URLs/paths as strings."""
    try:
        return int(value)
    except ValueError:
        return value


def is_live_source(source_text: str) -> bool:
    source = parse_source(source_text)
    if isinstance(source, int):
        return True
    lowered = source_text.lower()
    return lowered.startswith(("http://", "https://", "rtsp://", "rtmp://"))


def open_capture(source_text: str) -> cv2.VideoCapture:
    """Open local camera indexes or network/video sources.

    URL sources such as HTTP MJPEG and RTSP must use the normal OpenCV backend.
    Do not force cv2.CAP_V4L2 there; it is only appropriate for Linux local
    camera devices.
    """
    source = parse_source(source_text)
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class LatestFrameCapture:
    """Continuously read live sources and expose only the newest frame."""

    def __init__(self, cap: cv2.VideoCapture, threaded: bool) -> None:
        self.cap = cap
        self.threaded = threaded
        self.lock = threading.Lock()
        self.running = False
        self.thread: threading.Thread | None = None
        self.frame: Any | None = None
        self.frame_id = 0

    def start(self) -> None:
        if not self.threaded:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self.lock:
                self.frame = frame
                self.frame_id += 1

    def read(self, last_frame_id: int) -> tuple[bool, Any | None, int]:
        if not self.threaded:
            ok, frame = self.cap.read()
            if not ok:
                return False, None, last_frame_id
            return True, frame, last_frame_id + 1

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with self.lock:
                if self.frame is not None and self.frame_id != last_frame_id:
                    return True, self.frame.copy(), self.frame_id
            time.sleep(0.002)
        with self.lock:
            if self.frame is not None:
                return True, self.frame.copy(), self.frame_id
        return False, None, last_frame_id

    def release(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=0.5)
        self.cap.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Camera hand Y/Z follower for OpenCR.")
    parser.add_argument("--source", default="0", help="Camera index, video path, HTTP MJPEG URL, or RTSP URL")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF path")
    parser.add_argument("--base-z-mm", type=float, default=DEFAULT_BASE_Z_MM, help="Robot base height [mm]")
    parser.add_argument("--port", default="/dev/ttyACM0", help="OpenCR serial port")
    parser.add_argument("--baud", type=int, default=115200, help="OpenCR debug serial baudrate")
    parser.add_argument("--execute", action="store_true", help="Actually send q commands to OpenCR")
    parser.add_argument("--landmark", choices=sorted(LANDMARKS), default="wrist", help="Hand landmark to track")
    parser.add_argument("--fixed-x-mm", type=float, default=0.0, help="Fixed tool0 X target [mm]")
    parser.add_argument("--origin-px-x", type=float, help="Image pixel X that maps to origin Y")
    parser.add_argument("--origin-px-y", type=float, help="Image pixel Y that maps to origin Z")
    parser.add_argument("--origin-y-mm", type=float, default=-160.0, help="Robot Y at origin pixel [mm]")
    parser.add_argument("--origin-z-mm", type=float, default=190.0, help="Robot Z at origin pixel [mm]")
    parser.add_argument("--mm-per-px-y", type=float, default=0.7, help="Robot Y millimeters per image pixel X")
    parser.add_argument("--mm-per-px-z", type=float, default=0.7, help="Robot Z millimeters per image pixel Y upward")
    parser.add_argument("--flip-y", action="store_true", help="Invert image X to robot Y mapping")
    parser.add_argument("--flip-z", action="store_true", help="Invert image Y to robot Z mapping")
    parser.add_argument("--ema-alpha", type=float, default=0.30, help="Target smoothing factor")
    parser.add_argument("--rate-hz", type=float, default=12.0, help="Max q command rate")
    parser.add_argument("--max-step-deg", type=float, default=1.0, help="Per-command joint step clamp [deg]")
    parser.add_argument("--deadband-mm", type=float, default=1.0, help="Ignore tiny target changes [mm]")
    parser.add_argument("--no-display", action="store_true", help="Disable OpenCV display window")
    parser.add_argument("--no-landmark-draw", action="store_true", help="Skip drawing MediaPipe hand skeleton to reduce CPU load")
    parser.add_argument("--process-width", type=int, default=320, help="Resize frames to this width for MediaPipe; 0 disables resizing")
    parser.add_argument("--keep-buffered-frames", action="store_true", help="Process buffered frames instead of dropping stale live frames")
    for joint in JOINT_COMMANDS:
        parser.add_argument(f"--current-{joint}", type=float, default=0.0, help=f"Seed/current {joint.upper()} angle [deg]")
    return parser.parse_args()


def motor_to_joint_rad(name: str, motor_deg: float) -> float:
    return math.radians(JOINT_COMMANDS[name][1] * motor_deg)


def joint_rad_to_motor_deg(name: str, joint_rad: float) -> float:
    return math.degrees(joint_rad) / JOINT_COMMANDS[name][1]


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def pixel_to_robot_yz(args: argparse.Namespace, px: float, py: float, width: int, height: int) -> tuple[float, float]:
    origin_px_x = args.origin_px_x if args.origin_px_x is not None else width / 2.0
    origin_px_y = args.origin_px_y if args.origin_px_y is not None else height / 2.0
    y_sign = -1.0 if args.flip_y else 1.0
    z_sign = -1.0 if args.flip_z else 1.0
    y_mm = args.origin_y_mm + y_sign * (px - origin_px_x) * args.mm_per_px_y
    z_mm = args.origin_z_mm + z_sign * (origin_px_y - py) * args.mm_per_px_z
    return y_mm, z_mm


def blend(previous: tuple[float, float] | None, current: tuple[float, float], alpha: float) -> tuple[float, float]:
    if previous is None:
        return current
    return (
        previous[0] * (1.0 - alpha) + current[0] * alpha,
        previous[1] * (1.0 - alpha) + current[1] * alpha,
    )


def processing_frame(frame: Any, process_width: int) -> Any:
    if process_width <= 0:
        return frame
    height, width = frame.shape[:2]
    if width <= process_width:
        return frame
    process_height = max(1, round(height * process_width / width))
    return cv2.resize(frame, (process_width, process_height), interpolation=cv2.INTER_AREA)


def setup_ik(args: argparse.Namespace) -> tuple[int, dict[str, int], dict[str, int], list[float], list[float], list[float]]:
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(urdf_path)
    p.connect(p.DIRECT)
    robot_id = p.loadURDF(str(urdf_path), basePosition=[0, 0, args.base_z_mm / 1000.0], useFixedBase=True)
    joint_indices: dict[str, int] = {}
    link_indices: dict[str, int] = {}
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_indices[info[1].decode("utf-8")] = joint_index
        link_indices[info[12].decode("utf-8")] = joint_index
    lower = []
    upper = []
    for name in JOINT_COMMANDS:
        lo_deg, hi_deg = MOTOR_LIMITS_DEG[name]
        a = motor_to_joint_rad(name, lo_deg)
        b = motor_to_joint_rad(name, hi_deg)
        lower.append(min(a, b))
        upper.append(max(a, b))
    ranges = [upper[i] - lower[i] for i in range(len(lower))]
    return robot_id, joint_indices, link_indices, lower, upper, ranges


def solve_ik(
    robot_id: int,
    joint_indices: dict[str, int],
    link_indices: dict[str, int],
    lower: list[float],
    upper: list[float],
    ranges: list[float],
    current: dict[str, float],
    target_xyz_mm: tuple[float, float, float],
) -> dict[str, float]:
    for name, (joint_name, _) in JOINT_COMMANDS.items():
        p.resetJointState(robot_id, joint_indices[joint_name], motor_to_joint_rad(name, current[name]))
    rest = [motor_to_joint_rad(name, current[name]) for name in JOINT_COMMANDS]
    solution = p.calculateInverseKinematics(
        robot_id,
        link_indices["tool0"],
        [v / 1000.0 for v in target_xyz_mm],
        lowerLimits=lower,
        upperLimits=upper,
        jointRanges=ranges,
        restPoses=rest,
        maxNumIterations=120,
        residualThreshold=0.003,
    )
    result = {}
    for i, name in enumerate(JOINT_COMMANDS):
        lo, hi = MOTOR_LIMITS_DEG[name]
        result[name] = clamp(joint_rad_to_motor_deg(name, solution[i]), lo, hi)
    return result


def clamp_joint_step(current: dict[str, float], target: dict[str, float], max_step_deg: float) -> dict[str, float]:
    stepped = {}
    for name in JOINT_COMMANDS:
        delta = clamp(target[name] - current[name], -max_step_deg, max_step_deg)
        stepped[name] = current[name] + delta
    return stepped


def open_serial(args: argparse.Namespace) -> Any:
    import serial  # type: ignore

    port = serial.Serial(args.port, baudrate=args.baud, timeout=1.0)
    time.sleep(2.0)
    return port


def q_line(joints: dict[str, float]) -> str:
    return "q " + " ".join(f"{joints[name]:.3f}" for name in JOINT_COMMANDS)


def draw_overlay(frame: Any, args: argparse.Namespace, px: int | None, py: int | None, yz: tuple[float, float] | None, line: str | None) -> None:
    height, width = frame.shape[:2]
    ox = int(args.origin_px_x if args.origin_px_x is not None else width / 2)
    oy = int(args.origin_px_y if args.origin_px_y is not None else height / 2)
    cv2.line(frame, (ox - 20, oy), (ox + 20, oy), (0, 255, 255), 1)
    cv2.line(frame, (ox, oy - 20), (ox, oy + 20), (0, 255, 255), 1)
    if px is not None and py is not None and yz is not None:
        cv2.circle(frame, (px, py), 8, (0, 255, 0), 2)
        cv2.putText(frame, f"X={args.fixed_x_mm:.1f} Y={yz[0]:.1f} Z={yz[1]:.1f} mm", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "hand not found", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    if line:
        cv2.putText(frame, line[:70], (20, frame.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)


def main() -> None:
    args = parse_args()
    cap = open_capture(args.source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open source: {args.source}")
    capture = LatestFrameCapture(cap, threaded=is_live_source(args.source) and not args.keep_buffered_frames)
    capture.start()
    last_capture_id = -1

    robot_id, joint_indices, link_indices, lower, upper, ranges = setup_ik(args)
    current = {name: getattr(args, f"current_{name}") for name in JOINT_COMMANDS}
    serial_port = open_serial(args) if args.execute else None
    print(f"mode: {'EXECUTE' if args.execute else 'DRY_RUN'}")
    print(f"fixed_x_mm: {args.fixed_x_mm:.3f}")
    print("seed_deg: " + ", ".join(f"{name}={current[name]:.2f}" for name in JOINT_COMMANDS))

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    smoothed_yz: tuple[float, float] | None = None
    last_sent_at = 0.0
    last_target_yz: tuple[float, float] | None = None
    last_line: str | None = None

    try:
        with mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.55, min_tracking_confidence=0.55) as hands:
            while True:
                ok, frame, last_capture_id = capture.read(last_capture_id)
                if not ok or frame is None:
                    break
                height, width = frame.shape[:2]
                proc_frame = processing_frame(frame, args.process_width)
                result = hands.process(cv2.cvtColor(proc_frame, cv2.COLOR_BGR2RGB))
                px = py = None
                if result.multi_hand_landmarks:
                    hand = result.multi_hand_landmarks[0]
                    lm = hand.landmark[LANDMARKS[args.landmark]]
                    px = int(lm.x * width)
                    py = int(lm.y * height)
                    smoothed_yz = blend(smoothed_yz, pixel_to_robot_yz(args, px, py, width, height), args.ema_alpha)
                    if not args.no_display and not args.no_landmark_draw:
                        mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

                    now = time.monotonic()
                    enough_time = now - last_sent_at >= 1.0 / args.rate_hz
                    enough_motion = (
                        last_target_yz is None
                        or abs(smoothed_yz[0] - last_target_yz[0]) >= args.deadband_mm
                        or abs(smoothed_yz[1] - last_target_yz[1]) >= args.deadband_mm
                    )
                    if enough_time and enough_motion:
                        target_xyz = (args.fixed_x_mm, smoothed_yz[0], smoothed_yz[1])
                        ik_target = solve_ik(robot_id, joint_indices, link_indices, lower, upper, ranges, current, target_xyz)
                        command = clamp_joint_step(current, ik_target, args.max_step_deg)
                        line = q_line(command)
                        print(f"target_mm: x={target_xyz[0]:.1f}, y={target_xyz[1]:.1f}, z={target_xyz[2]:.1f} -> {line}")
                        if serial_port is not None:
                            serial_port.write((line + "\n").encode("ascii"))
                            serial_port.flush()
                        current = command
                        last_sent_at = now
                        last_target_yz = smoothed_yz
                        last_line = line

                if not args.no_display:
                    draw_overlay(frame, args, px, py, smoothed_yz if px is not None else None, last_line)
                    cv2.imshow("hand_yz_follow_opencr", frame)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        break
    finally:
        capture.release()
        if serial_port is not None:
            serial_port.close()
        p.disconnect()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
