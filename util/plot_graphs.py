"""
Plot recorded Arduino teleplot data from CSV.

Reads columns:
  - `t_sec`
  - `pendulum_angle_deg`
  - `estimated_velocity` (cart speed)
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from typing import List, Tuple

import matplotlib.pyplot as plt


def _default_csv_path() -> str:
    # Default to the recording that the user referenced.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(script_dir, "recordings", "eval_b_lqr_20.csv")


def load_csv(csv_path: str) -> Tuple[List[float], List[float], List[float]]:
    t_list: List[float] = []
    angle_list: List[float] = []
    velocity_list: List[float] = []

    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {"t_sec", "pendulum_angle_deg", "estimated_velocity"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV missing required columns: {sorted(missing)}")

        for row in reader:
            try:
                t = float(row["t_sec"])
                ang = float(row["pendulum_angle_deg"])
                vel = float(row["estimated_velocity"])
            except (TypeError, ValueError):
                continue
            t_list.append(t)
            angle_list.append(ang)
            velocity_list.append(vel)

    if not t_list:
        raise ValueError("No valid rows parsed from CSV.")

    return t_list, angle_list, velocity_list


def plot(t: List[float], angle_deg: List[float], velocity: List[float], save_dir: str | None) -> None:
    # Pendulum angle plot
    fig1, ax1 = plt.subplots(1, 1, figsize=(10, 5))
    ax1.plot(t, angle_deg, linewidth=1.2, color="C1")
    ax1.set_title("Pendulum Angle vs Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Angle (deg)")
    ax1.grid(True, alpha=0.3)

    # Cart velocity plot
    fig2, ax2 = plt.subplots(1, 1, figsize=(10, 5))
    ax2.plot(t, velocity, linewidth=1.2, color="C0")
    ax2.set_title("Cart Velocity vs Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.grid(True, alpha=0.3)

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        fig1.savefig(os.path.join(save_dir, "pendulum_angle_vs_time.png"), dpi=200)
        fig2.savefig(os.path.join(save_dir, "cart_velocity_vs_time.png"), dpi=200)

    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot pendulum angle and cart velocity from a recorded CSV.")
    parser.add_argument("--csv", default=_default_csv_path(), help="Path to arduino_live_log CSV.")
    parser.add_argument(
        "--save-dir",
        default=None,
        help="Optional directory to save PNGs (pendulum_angle_vs_time.png, cart_velocity_vs_time.png).",
    )
    args = parser.parse_args()

    csv_path = args.csv
    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    t, angles, v = load_csv(csv_path)
    plot(t, angles, v, save_dir=args.save_dir)


if __name__ == "__main__":
    main()

