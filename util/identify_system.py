"""
Offline system identification for the pendulum using a recorded CSV dataset.

The CSV is expected to have the format produced by plot_pendulum.py:
    time_s,angle_deg

Here the experiment is the pendulum freely oscillating with the pendulum facing
DOWNWARDS at 0 degrees. We therefore linearise about the downward equilibrium
(theta = 0 at hanging down), which for small angles gives:

    theta_ddot + 2*zeta*omega_n*theta_dot + omega_n^2*theta ≈ 0
    => alpha ≈ -2*zeta*omega_n*omega - omega_n^2*theta

We perform a least-squares fit on a user-selected time window [t_start, t_end].
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Identify pendulum dynamics from a CSV recording. "
            "Assumes pendulum is hanging down at 0 deg and freely oscillating."
        )
    )
    parser.add_argument(
        "file",
        help="Path to dataset CSV (e.g. util/recordings/pendulum_YYYY-MM-DD_HH-MM-SS.csv).",
    )
    parser.add_argument(
        "--t-start",
        type=float,
        default=None,
        metavar="SECONDS",
        help="Start time (s) of the window to use for identification (default: first sample).",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        metavar="SECONDS",
        help="End time (s) of the window to use for identification (default: last sample).",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable plots (only print identified parameters).",
    )
    return parser.parse_args()


def load_dataset(path: str):
    if not os.path.isfile(path):
        print(f"CSV file not found: {path}", file=sys.stderr)
        sys.exit(1)
    try:
        data = np.loadtxt(path, delimiter=",", skiprows=1)
    except Exception as exc:  # noqa: BLE001
        print(f"Failed to load CSV '{path}': {exc}", file=sys.stderr)
        sys.exit(1)
    if data.ndim != 2 or data.shape[1] < 2:
        print("CSV must have at least two columns: time_s,angle_deg", file=sys.stderr)
        sys.exit(1)
    times = data[:, 0]
    angles_deg = data[:, 1]
    return times, angles_deg


def subset_window(times: np.ndarray, angles_deg: np.ndarray, t_start: float | None, t_end: float | None):
    if t_start is None:
        t_start = float(times[0])
    if t_end is None:
        t_end = float(times[-1])
    if t_start >= t_end:
        print("t_start must be < t_end", file=sys.stderr)
        sys.exit(1)
    mask = (times >= t_start) & (times <= t_end)
    if not np.any(mask):
        print("No samples in the requested time window.", file=sys.stderr)
        sys.exit(1)
    return times[mask], angles_deg[mask], t_start, t_end


def identify_pendulum(times: np.ndarray, angles_deg: np.ndarray):
    """
    Identify omega_n and zeta for the pendulum linearised about the downward equilibrium.

    Model:
        theta_ddot + 2*zeta*omega_n*theta_dot + omega_n**2*theta ≈ 0
        alpha ≈ -2*zeta*omega_n*omega - omega_n**2*theta
    """
    n = len(times)
    if n < 10:
        print("Too few samples for system ID (need at least 10).", file=sys.stderr)
        return None, None, None, None

    # theta in radians, 0 at downward equilibrium (given experiment setup)
    theta = np.deg2rad(angles_deg)
    omega = np.gradient(theta, times)
    alpha = np.gradient(omega, times)

    # Optional light smoothing (simple moving average) before differentiation
    window = min(5, n // 2) if n >= 4 else 0
    if window >= 2:
        kernel = np.ones(window, dtype=float) / float(window)
        theta_smooth = np.convolve(theta, kernel, mode="same")
        omega = np.gradient(theta_smooth, times)
        alpha = np.gradient(omega, times)

    # Regression: alpha = c0*omega + c1*theta
    # For downward equilibrium: c0 = -2*zeta*omega_n, c1 = -omega_n**2 (expected negative)
    X = np.column_stack([omega, theta])
    try:
        coeffs, *_ = np.linalg.lstsq(X, alpha, rcond=None)
    except np.linalg.LinAlgError:
        print("Regression failed for system ID.", file=sys.stderr)
        return None, None, None, None

    c_omega, c_theta = coeffs[0], coeffs[1]
    if c_theta == 0:
        print("Estimated theta coefficient is zero; cannot determine omega_n.", file=sys.stderr)
        return None, None, None, None

    omega_n = float(np.sqrt(abs(c_theta)))
    zeta = -c_omega / (2.0 * omega_n) if omega_n != 0.0 else 0.0
    f_n = omega_n / (2.0 * np.pi)
    T = 1.0 / f_n if f_n > 0 else float("nan")
    return omega_n, zeta, f_n, T


def plot_window(times: np.ndarray, angles_deg: np.ndarray, omega_n=None, zeta=None, f_n=None, T=None):
    theta = np.deg2rad(angles_deg)
    omega = np.gradient(theta, times)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax0, ax1 = axes[0], axes[1]

    ax0.plot(times, angles_deg, color="C0", label="Angle (deg, down = 0)")
    ax0.set_ylabel("Angle (deg)")
    ax0.set_title("Pendulum angle (subset for identification)")
    ax0.grid(True)
    ax0.legend(loc="upper right")
    if omega_n is not None and f_n is not None and T is not None:
        text = (
            f"$\\omega_n$ = {omega_n:.3f} rad/s, $\\zeta$ = {zeta:.4f}\n"
            f"$f_n$ = {f_n:.3f} Hz, T = {T:.3f} s"
        )
        ax0.text(
            0.02,
            0.98,
            text,
            transform=ax0.transAxes,
            fontsize=9,
            verticalalignment="top",
            family="monospace",
        )

    ax1.plot(times, omega, color="C1", label="Angular velocity (rad/s)")
    ax1.set_ylabel("Angular velocity (rad/s)")
    ax1.set_xlabel("Time (s)")
    ax1.set_title("Angular velocity (subset)")
    ax1.grid(True)
    ax1.legend(loc="upper right")

    plt.tight_layout()

    fig2, ax2 = plt.subplots(1, 1, figsize=(7, 6))
    ax2.plot(angles_deg, omega, color="C2", alpha=0.7)
    ax2.set_xlabel("Angle (deg, down = 0)")
    ax2.set_ylabel("Angular velocity (rad/s)")
    ax2.set_title("Phase portrait (subset)")
    ax2.grid(True)
    plt.tight_layout()

    plt.show()


def main():
    args = parse_args()

    times_all, angles_deg_all = load_dataset(args.file)
    times_win, angles_deg_win, t_start, t_end = subset_window(
        times_all, angles_deg_all, args.t_start, args.t_end
    )

    print(f"Loaded {len(times_all)} samples from {args.file}")
    print(f"Using window [{t_start:.3f}, {t_end:.3f}] s with {len(times_win)} samples.")

    omega_n, zeta, f_n, T = identify_pendulum(times_win, angles_deg_win)
    if omega_n is None:
        print("System identification failed or insufficient data.")
        return

    print("System identification (pendulum about downward equilibrium):")
    print(f"  omega_n = {omega_n:.6f} rad/s")
    print(f"  zeta    = {zeta:.6f}")
    print(f"  f_n     = {f_n:.6f} Hz")
    print(f"  T       = {T:.6f} s")

    if not args.no_plot:
        plot_window(times_win, angles_deg_win, omega_n=omega_n, zeta=zeta, f_n=f_n, T=T)


if __name__ == "__main__":
    main()

