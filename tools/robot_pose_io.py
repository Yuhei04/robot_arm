"""Small helpers for taught joint poses and OpenCR angle reads."""

import re
import time


JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")
ANGLE_RE = re.compile(r"ID\s+(\d+)\s*:\s*([-+0-9.]+)\s*deg")
ID_TO_JOINT = {1: "j1", 2: "j2", 3: "j3", 4: "j4", 5: "j5", 6: "j6"}


def q_line(joints: dict[str, float]) -> str:
    return "q " + " ".join(f"{joints[name]:.3f}" for name in JOINT_NAMES)


def max_joint_delta(a: dict[str, float], b: dict[str, float]) -> float:
    return max(abs(a[name] - b[name]) for name in JOINT_NAMES)


def read_current_angles(port: str, baud: int) -> dict[str, float]:
    import serial  # type: ignore

    with serial.Serial(port, baudrate=baud, timeout=0.1) as ser:
        time.sleep(2.5)
        ser.reset_input_buffer()
        ser.write(b"a\n")
        ser.flush()
        deadline = time.time() + 3.0
        chunks = []
        while time.time() < deadline:
            data = ser.read(4096)
            if data:
                chunks.append(data)
            else:
                time.sleep(0.05)
    text = b"".join(chunks).decode("utf-8", errors="replace")
    angles = {}
    for match in ANGLE_RE.finditer(text):
        joint = ID_TO_JOINT.get(int(match.group(1)))
        if joint:
            angles[joint] = float(match.group(2))
    missing = [name for name in JOINT_NAMES if name not in angles]
    if missing:
        raise RuntimeError("Could not read all joint angles from OpenCR output:\n" + text)
    return angles
