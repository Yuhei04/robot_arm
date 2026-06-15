"""Shared robot arm joint limits in motor-angle degrees."""

from __future__ import annotations

import csv
import math
from pathlib import Path


DEFAULT_LIMITS_CSV = Path(__file__).resolve().parents[1] / "cad" / "exports" / "fusion_tables" / "joint_limits.csv"
JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")
MOTOR_ID_TO_JOINT = {"1": "j1", "2": "j2", "3": "j3", "4": "j4", "5": "j5", "6": "j6"}
FALLBACK_LIMITS_DEG = {name: (-90.0, 90.0) for name in JOINT_NAMES}


def read_joint_limits(path: Path = DEFAULT_LIMITS_CSV) -> dict[str, tuple[float, float]]:
    if not path.exists():
        return FALLBACK_LIMITS_DEG.copy()
    limits: dict[str, tuple[float, float]] = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            joint = (row.get("joint") or "").strip()
            if not joint:
                motor_id = (row.get("motor_id") or "").strip()
                joint = MOTOR_ID_TO_JOINT.get(motor_id, "")
            if joint not in JOINT_NAMES:
                continue
            lo = float(row["min_deg"])
            hi = float(row["max_deg"])
            limits[joint] = (min(lo, hi), max(lo, hi))
    missing = [name for name in JOINT_NAMES if name not in limits]
    if missing:
        raise ValueError(f"Missing joint limits for: {', '.join(missing)} in {path}")
    return limits


def motor_limit_for_joint(joint: str, path: Path = DEFAULT_LIMITS_CSV) -> tuple[float, float]:
    return read_joint_limits(path)[joint]


def motor_limit_for_id(motor_id: str, path: Path = DEFAULT_LIMITS_CSV) -> tuple[float, float]:
    joint = MOTOR_ID_TO_JOINT.get(str(motor_id))
    if joint is None:
        raise ValueError(f"Unknown motor_id: {motor_id}")
    return motor_limit_for_joint(joint, path)


def urdf_joint_limit_rad(motor_id: str, sign: float, path: Path = DEFAULT_LIMITS_CSV) -> tuple[float, float]:
    lo_deg, hi_deg = motor_limit_for_id(motor_id, path)
    a = math.radians(sign * lo_deg)
    b = math.radians(sign * hi_deg)
    return min(a, b), max(a, b)
