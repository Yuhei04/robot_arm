#!/usr/bin/env python3
"""Track a hand from a side-view video/camera and report robot Y/Z targets."""

import argparse
import csv
import json
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import mediapipe as mp


LANDMARKS = {
    "wrist": 0,
    "thumb_tip": 4,
    "index_tip": 8,
    "middle_tip": 12,
    "ring_tip": 16,
    "pinky_tip": 20,
}


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
    parser = argparse.ArgumentParser(description="Track hand landmark Y/Z from a side-view camera.")
    parser.add_argument("--source", default="0", help="Camera index, video path, HTTP MJPEG URL, or RTSP URL")
    parser.add_argument("--landmark", choices=sorted(LANDMARKS), default="wrist", help="Hand landmark to track")
    parser.add_argument("--origin-px-x", type=float, help="Image pixel X that maps to origin Y")
    parser.add_argument("--origin-px-y", type=float, help="Image pixel Y that maps to origin Z")
    parser.add_argument("--origin-y-mm", type=float, default=-160.0, help="Robot Y at origin pixel [mm]")
    parser.add_argument("--origin-z-mm", type=float, default=190.0, help="Robot Z at origin pixel [mm]")
    parser.add_argument("--mm-per-px-y", type=float, default=1.0, help="Robot Y millimeters per image pixel X")
    parser.add_argument("--mm-per-px-z", type=float, default=1.0, help="Robot Z millimeters per image pixel Y upward")
    parser.add_argument("--flip-y", action="store_true", help="Invert image X to robot Y mapping")
    parser.add_argument("--flip-z", action="store_true", help="Invert image Y to robot Z mapping")
    parser.add_argument("--ema-alpha", type=float, default=0.25, help="Smoothing factor, 1.0 disables smoothing")
    parser.add_argument("--csv", type=Path, default=Path("outputs/hand_yz_targets.csv"), help="CSV output path")
    parser.add_argument("--json", type=Path, default=Path("outputs/latest_hand_yz.json"), help="Latest target JSON path")
    parser.add_argument("--no-display", action="store_true", help="Disable OpenCV display window")
    parser.add_argument("--no-landmark-draw", action="store_true", help="Skip drawing MediaPipe hand skeleton to reduce CPU load")
    parser.add_argument("--process-width", type=int, default=320, help="Resize frames to this width for MediaPipe; 0 disables resizing")
    parser.add_argument("--keep-buffered-frames", action="store_true", help="Process buffered frames instead of dropping stale live frames")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames, 0 means unlimited")
    return parser.parse_args()


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


def draw_overlay(
    frame: Any,
    args: argparse.Namespace,
    px: int | None,
    py: int | None,
    yz: tuple[float, float] | None,
) -> None:
    height, width = frame.shape[:2]
    ox = int(args.origin_px_x if args.origin_px_x is not None else width / 2)
    oy = int(args.origin_px_y if args.origin_px_y is not None else height / 2)
    cv2.line(frame, (ox - 20, oy), (ox + 20, oy), (0, 255, 255), 1)
    cv2.line(frame, (ox, oy - 20), (ox, oy + 20), (0, 255, 255), 1)
    cv2.putText(frame, "origin", (ox + 8, oy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    if px is not None and py is not None and yz is not None:
        cv2.circle(frame, (px, py), 8, (0, 255, 0), 2)
        cv2.putText(
            frame,
            f"Y={yz[0]:.1f}mm Z={yz[1]:.1f}mm",
            (20, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
    else:
        cv2.putText(frame, "hand not found", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


def main() -> None:
    args = parse_args()
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    args.json.parent.mkdir(parents=True, exist_ok=True)

    cap = open_capture(args.source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open source: {args.source}")
    capture = LatestFrameCapture(cap, threaded=is_live_source(args.source) and not args.keep_buffered_frames)
    capture.start()
    last_capture_id = -1

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    smoothed: tuple[float, float] | None = None
    frame_index = 0

    with args.csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["time_s", "frame", "found", "landmark", "pixel_x", "pixel_y", "target_y_mm", "target_z_mm"],
        )
        writer.writeheader()

        with mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.55, min_tracking_confidence=0.55) as hands:
            start = time.monotonic()
            while True:
                ok, frame, last_capture_id = capture.read(last_capture_id)
                if not ok or frame is None:
                    break
                frame_index += 1
                height, width = frame.shape[:2]
                proc_frame = processing_frame(frame, args.process_width)
                result = hands.process(cv2.cvtColor(proc_frame, cv2.COLOR_BGR2RGB))

                found = False
                px = py = None
                yz = None
                if result.multi_hand_landmarks:
                    hand = result.multi_hand_landmarks[0]
                    lm = hand.landmark[LANDMARKS[args.landmark]]
                    px = int(lm.x * width)
                    py = int(lm.y * height)
                    raw_yz = pixel_to_robot_yz(args, px, py, width, height)
                    smoothed = blend(smoothed, raw_yz, max(0.0, min(args.ema_alpha, 1.0)))
                    yz = smoothed
                    found = True
                    if not args.no_display and not args.no_landmark_draw:
                        mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

                now = time.monotonic() - start
                writer.writerow(
                    {
                        "time_s": f"{now:.3f}",
                        "frame": frame_index,
                        "found": int(found),
                        "landmark": args.landmark,
                        "pixel_x": "" if px is None else px,
                        "pixel_y": "" if py is None else py,
                        "target_y_mm": "" if yz is None else f"{yz[0]:.3f}",
                        "target_z_mm": "" if yz is None else f"{yz[1]:.3f}",
                    }
                )
                if yz is not None:
                    args.json.write_text(
                        json.dumps(
                            {"time_s": now, "frame": frame_index, "landmark": args.landmark, "target_y_mm": yz[0], "target_z_mm": yz[1]},
                            indent=2,
                        )
                        + "\n"
                    )

                if not args.no_display:
                    draw_overlay(frame, args, px, py, yz)
                    cv2.imshow("hand_yz_tracker", frame)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        break
                if args.max_frames and frame_index >= args.max_frames:
                    break

    capture.release()
    if not args.no_display:
        cv2.destroyAllWindows()
    print(f"wrote: {args.csv}")
    print(f"latest: {args.json}")


if __name__ == "__main__":
    main()
