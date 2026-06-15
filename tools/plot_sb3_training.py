#!/usr/bin/env python3
"""Plot Stable-Baselines3 Monitor CSV training curves."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

DEFAULT_RUN_DIR = Path(__file__).resolve().parents[1] / "outputs" / "sb3_reacher"


def moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if len(values) == 0:
        return values
    window = max(1, min(window, len(values)))
    kernel = np.ones(window) / window
    return np.convolve(values, kernel, mode="valid")


def read_monitor_csv(path: Path) -> dict[str, list[float]]:
    rows: list[dict[str, str]] = []
    with path.open(newline="", encoding="utf-8") as f:
        filtered = (line for line in f if not line.startswith("#"))
        reader = csv.DictReader(filtered)
        rows.extend(reader)
    data: dict[str, list[float]] = {"episode": [], "reward": [], "length": [], "time": [], "distance_mm": [], "success": []}
    for index, row in enumerate(rows, start=1):
        data["episode"].append(float(index))
        data["reward"].append(float(row["r"]))
        data["length"].append(float(row["l"]))
        data["time"].append(float(row["t"]))
        if row.get("distance_mm") not in (None, ""):
            data["distance_mm"].append(float(row["distance_mm"]))
        if row.get("is_success") not in (None, ""):
            data["success"].append(1.0 if row["is_success"] in ("True", "1", "1.0") else 0.0)
    return data


def plot_monitor_log(monitor_csv: Path, output_png: Path, window: int = 20) -> None:
    data = read_monitor_csv(monitor_csv)
    episodes = np.array(data["episode"], dtype=np.float64)
    rewards = np.array(data["reward"], dtype=np.float64)
    lengths = np.array(data["length"], dtype=np.float64)
    distances = np.array(data["distance_mm"], dtype=np.float64)
    successes = np.array(data["success"], dtype=np.float64)
    if len(episodes) == 0:
        raise ValueError(f"No episodes found in {monitor_csv}")

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle("Stable-Baselines3 PyBullet reach training")

    axes[0].plot(episodes, rewards, color="#6a7fdb", alpha=0.35, label="episode")
    avg = moving_average(rewards, window)
    axes[0].plot(episodes[-len(avg):], avg, color="#253a9b", label=f"{window}-episode avg")
    axes[0].set_ylabel("Reward")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best")

    if len(distances) == len(episodes):
        axes[1].plot(episodes, distances, color="#d06b36", alpha=0.35, label="episode")
        avg = moving_average(distances, window)
        axes[1].plot(episodes[-len(avg):], avg, color="#913f16", label=f"{window}-episode avg")
        axes[1].set_ylabel("Final distance [mm]")
        axes[1].legend(loc="best")
    else:
        axes[1].plot(episodes, lengths, color="#d06b36", label="episode length")
        axes[1].set_ylabel("Episode length")
    axes[1].grid(True, alpha=0.25)

    if len(successes) == len(episodes):
        avg = moving_average(successes, window)
        axes[2].plot(episodes[-len(avg):], avg * 100.0, color="#237b53", label=f"{window}-episode avg")
        axes[2].set_ylabel("Success [%]")
        axes[2].set_ylim(-2, 102)
        axes[2].legend(loc="best")
    else:
        axes[2].plot(episodes, lengths, color="#237b53")
        axes[2].set_ylabel("Episode length")
    axes[2].set_xlabel("Episode")
    axes[2].grid(True, alpha=0.25)

    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_png, dpi=150)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot SB3 Monitor CSV curves.")
    parser.add_argument("--monitor", type=Path, default=DEFAULT_RUN_DIR / "monitor.csv", help="Monitor CSV")
    parser.add_argument("--output", type=Path, default=DEFAULT_RUN_DIR / "training_curves.png", help="Output PNG")
    parser.add_argument("--window", type=int, default=20, help="Moving average window")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    plot_monitor_log(args.monitor, args.output, args.window)
    print(f"training_plot={args.output}")


if __name__ == "__main__":
    main()
