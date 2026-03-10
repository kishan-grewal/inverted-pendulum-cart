"""
Calibrate pendulum encoder so 0° is at the physical middle between the two stoppers.

Expects a dataset.csv (time_s, angle_deg) recorded with this sequence:
  1. Start at front (rest), 2. Move to back (rest), 3. Move to front (rest).
Plateaus are detected in time order; segment 1 = front, segment 2 = back, segment 3 = front.
Computes the offset to add to the encoder angle in the Arduino so 0° = physical middle.
"""

import argparse
import os
import sys

import numpy as np


RECORDINGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
CALIBRATION_OFFSET_FILE = os.path.join(RECORDINGS_DIR, "calibration_offset.txt")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compute pendulum encoder calibration offset from a stopper-sweep CSV."
    )
    parser.add_argument(
        "file",
        help="Path to dataset CSV (time_s, angle_deg). Record: start at front, move to back, then to front.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=4.0,
        metavar="DEG_PER_S",
        help="Max angular rate (deg/s) to count as plateau (default: 4.0).",
    )
    parser.add_argument(
        "--min-plateau-samples",
        type=int,
        default=5,
        metavar="N",
        help="Min consecutive samples in plateau (default: 5).",
    )
    parser.add_argument(
        "--write-offset-file",
        action="store_true",
        help="Write offset to util/recordings/calibration_offset.txt.",
    )
    return parser.parse_args()


def load_dataset(path: str):
    if not os.path.isfile(path):
        # If path is a bare filename, try util/recordings/ (next to this script)
        if os.path.basename(path) == path:
            fallback = os.path.join(RECORDINGS_DIR, path)
            if os.path.isfile(fallback):
                path = fallback
        if not os.path.isfile(path):
            print(f"File not found: {path}", file=sys.stderr)
            sys.exit(1)
    try:
        data = np.loadtxt(path, delimiter=",", skiprows=1)
    except Exception as e:
        print(f"Failed to load CSV: {e}", file=sys.stderr)
        sys.exit(1)
    if data.ndim != 2 or data.shape[1] < 2:
        print("CSV must have at least two columns: time_s, angle_deg", file=sys.stderr)
        sys.exit(1)
    time_s = data[:, 0]
    angle_deg = data[:, 1]
    return time_s, angle_deg


def find_plateau_segments(time_s, angle_deg, rate_threshold_deg_s, min_samples):
    """Return list of (start_idx, end_idx) for each plateau segment."""
    omega = np.gradient(angle_deg, time_s)
    is_plateau = np.abs(omega) < rate_threshold_deg_s
    n = len(is_plateau)
    segments = []
    i = 0
    while i < n:
        if not is_plateau[i]:
            i += 1
            continue
        start = i
        while i < n and is_plateau[i]:
            i += 1
        end = i
        if end - start >= min_samples:
            segments.append((start, end))
        i += 1
    return segments


def segment_means(time_s, angle_deg, segments):
    """Return (means, list of angle arrays per segment)."""
    means = []
    angle_arrays = []
    for start, end in segments:
        angles = angle_deg[start:end]
        means.append(float(np.mean(angles)))
        angle_arrays.append(angles)
    return np.array(means), angle_arrays


def unwrap_angles_180(angles_deg):
    """Convert to [-180, 180] so averaging gives the correct geometric middle."""
    out = np.array(angles_deg, dtype=float)
    out[out > 180] -= 360
    return out


def compute_offset_from_order(angle_arrays):
    """
    Use temporal order: segment 0 = front, segment 1 = back, segment 2 = front.
    Unwrap both plateaus to [-180, 180] so the midpoint is vertical (0°).
    Returns (offset, front_center, back_center, middle, n_front, n_back).
    """
    if len(angle_arrays) < 3:
        print(
            "Need at least 3 plateau segments (front, back, front). "
            "Record: start at front, move to back, then move to front.",
            file=sys.stderr,
        )
        sys.exit(1)

    # Segment 0 and 2 = front, segment 1 = back
    front_angles = np.concatenate([angle_arrays[0], angle_arrays[2]])
    back_angles = angle_arrays[1]

    # Unwrap both to [-180, 180] so midpoint is correct (e.g. front 5° and back 355° → -5°)
    front_unwrapped = unwrap_angles_180(front_angles)
    back_unwrapped = unwrap_angles_180(back_angles)
    front_center = float(np.mean(front_unwrapped))
    back_center = float(np.mean(back_unwrapped))
    middle = (front_center + back_center) / 2.0
    offset = -middle
    return offset, front_center, back_center, middle, len(front_angles), len(back_angles)


def main():
    args = parse_args()
    time_s, angle_deg = load_dataset(args.file)
    segments = find_plateau_segments(
        time_s,
        angle_deg,
        args.threshold,
        args.min_plateau_samples,
    )
    if len(segments) < 3:
        print(
            "Need at least 3 plateau segments (front, back, front). "
            "Record: start at front, move to back, then move to front. "
            "Try --threshold or --min-plateau-samples if needed.",
            file=sys.stderr,
        )
        sys.exit(1)

    _, angle_arrays = segment_means(time_s, angle_deg, segments)
    offset, front_center, back_center, middle, n_front, n_back = compute_offset_from_order(
        angle_arrays
    )

    print("Calibration results (order: front -> back -> front)")
    print("---------------------------------------------------")
    print(f"  Front plateau center (deg): {front_center:.3f}  (n = {n_front} samples)")
    print(f"  Back  plateau center (deg):  {back_center:.3f}  (n = {n_back} samples)")
    print(f"  Physical middle (deg):      {middle:.3f}")
    print(f"  Offset to add (deg):        {offset:.3f}")
    print()
    print("Arduino: add the following.")
    print("  1. Near the top of arduino_test.ino (e.g. after pendulum_pulses_per_revolution), add:")
    print(f"     #define CALIBRATION_OFFSET_DEG  ({offset:.3f}f)")
    print("  2. After the block that wraps pendulum_encoder_angle to [0, 360), add:")
    print("     pendulum_encoder_angle += CALIBRATION_OFFSET_DEG;")
    print("     // Wrap to [-180, 180] so 0 = vertical")
    print("     if (pendulum_encoder_angle > 180.0f) pendulum_encoder_angle -= 360.0f;")
    print("     else if (pendulum_encoder_angle < -180.0f) pendulum_encoder_angle += 360.0f;")
    print()

    if args.write_offset_file:
        os.makedirs(RECORDINGS_DIR, exist_ok=True)
        with open(CALIBRATION_OFFSET_FILE, "w") as f:
            f.write(f"{offset:.6f}\n")
        print(f"Wrote offset to {CALIBRATION_OFFSET_FILE}")


if __name__ == "__main__":
    main()
