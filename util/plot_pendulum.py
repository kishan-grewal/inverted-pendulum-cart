"""
Read pendulum angle from Arduino over Serial, store samples, plot in matplotlib,
and save recordings for offline system identification of the pendulum.
Updates the plot in real time (MATLAB-style) and saves a timestamped CSV at the end.
"""

import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
import serial

# Default serial port (Windows: COM3, Linux/Mac: /dev/ttyUSB0 or /dev/ttyACM0)
DEFAULT_PORT = "COM10"
BAUD_RATE = 115200
PENDULUM_ANGLE_PREFIX = "Pendulum Angle: "

# Directory for CSV recordings (next to this script)
RECORDINGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")

# Angle convention: Arduino sends 0° = upright, 180° = down. We use the same for
# plotting (degrees). For system ID we use radians with 0 rad = upright (0°).


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record pendulum angle from Arduino Serial and run system ID."
    )
    parser.add_argument(
        "--port", "-p",
        default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--duration", "-d",
        type=float,
        default=30.0,
        metavar="SECONDS",
        help="Recording duration in seconds (default: 30). Use 0 to run until Ctrl+C.",
    )
    return parser.parse_args()


def setup_live_plot():
    """
    Create interactive figure and line artists for real-time updates (MATLAB-style).
    Returns (fig, axes, line_angle, line_omega, text_sysid).
    """
    plt.ion()
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax0, ax1 = axes[0], axes[1]
    line_angle, = ax0.plot([], [], color="C0", label="Angle (deg)")
    ax0.set_ylabel("Angle (deg)")
    ax0.set_title("Pendulum angle (0° = upright, 180° = down)")
    ax0.grid(True)
    ax0.legend(loc="upper right")
    ax0.set_xlim(0, 10)
    ax0.set_ylim(-10, 370)
    text_sysid = ax0.text(0.02, 0.98, "", transform=ax0.transAxes, fontsize=9, verticalalignment="top", family="monospace")
    line_omega, = ax1.plot([], [], color="C1", label="Angular velocity (rad/s)")
    ax1.set_ylabel("Angular velocity (rad/s)")
    ax1.set_xlabel("Time (s)")
    ax1.set_title("Angular velocity")
    ax1.grid(True)
    ax1.legend(loc="upper right")
    ax1.set_xlim(0, 10)
    ax1.set_ylim(-5, 5)
    plt.tight_layout()
    return fig, axes, line_angle, line_omega, text_sysid


def update_live_plot(fig, line_angle, line_omega, times_list, angles_list):
    """Update line data for real-time display. Call from record loop."""
    if not times_list:
        return
    times = np.array(times_list)
    angles_deg = np.array(angles_list)
    line_angle.set_data(times, angles_deg)
    if len(times) >= 2:
        theta = np.deg2rad(angles_deg)
        omega = np.gradient(theta, times)
        line_omega.set_data(times, omega)
    ax0, ax1 = fig.get_axes()[0], fig.get_axes()[1]
    if len(times) > 1:
        ax0.set_xlim(0, max(10, times[-1] * 1.05))
        ax1.set_xlim(0, max(10, times[-1] * 1.05))
        y_omega = np.gradient(np.deg2rad(angles_deg), times)
        margin = max(0.5, np.abs(y_omega).max() * 1.2)
        ax1.set_ylim(-margin, margin)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    plt.pause(0.001)


def record_angles(port: str, duration: float, on_update=None, stop_flag=None):
    """
    Open serial port, read lines, parse 'Pendulum Angle: X.XX', store (t_rel, angle_deg).
    If on_update(times_list, angles_list) is given, call it after each new sample for real-time plot.
    If stop_flag is a list (e.g. [False]), stop when stop_flag[0] is True (e.g. on key press).
    Returns (times, angles_deg) as lists. Exits on duration elapsed, stop_flag, or Ctrl+C if duration==0.
    """
    times = []
    angles_deg = []
    t_start = None
    if stop_flag is None:
        stop_flag = [False]

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Recording from {port} at {BAUD_RATE} baud. Duration: {duration}s (0 = until key press).")
    if duration > 0:
        print(f"Will stop after {duration} seconds.")
    else:
        print("Press 'q' in the plot window to stop recording.")
    print("Waiting for data...")

    try:
        while True:
            if stop_flag[0]:
                print("Stop requested (key pressed).")
                break
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if not line.startswith(PENDULUM_ANGLE_PREFIX):
                continue
            try:
                angle = float(line.split(": ")[1])
            except (IndexError, ValueError):
                continue
            now = time.perf_counter()
            if t_start is None:
                t_start = now
            t_rel = now - t_start
            times.append(t_rel)
            angles_deg.append(angle)
            if on_update:
                on_update(times, angles_deg)
            if duration > 0 and t_rel >= duration:
                break
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()

    return times, angles_deg


def write_recordings_csv(times: np.ndarray, angles_deg: np.ndarray) -> str:
    """
    Write time_s, angle_deg to util/recordings/pendulum_YYYY-MM-DD_HH-MM-SS.csv.
    Creates util/recordings if needed. Returns the path of the written file.
    """
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"pendulum_{timestamp}.csv"
    filepath = os.path.join(RECORDINGS_DIR, filename)
    with open(filepath, "w", newline="") as f:
        f.write("time_s,angle_deg\n")
        for t, a in zip(times, angles_deg):
            f.write(f"{t:.6f},{a:.6f}\n")
    return filepath


def update_final_plot(text_sysid, omega_n=None, zeta=None, f_n=None, T=None):
    """Set system ID text on the live plot after recording."""
    # System identification is now handled offline in identify_system.py.
    text_sysid.set_text("")


def main():
    args = parse_args()
    # Real-time plot (MATLAB-style)
    fig, axes, line_angle, line_omega, text_sysid = setup_live_plot()
    stop_flag = [False]

    def on_key(event):
        if event.key == "q":
            stop_flag[0] = True

    fig.canvas.mpl_connect("key_press_event", on_key)

    def on_update(times_list, angles_list):
        update_live_plot(fig, line_angle, line_omega, times_list, angles_list)

    times_list, angles_list = record_angles(
        args.port, args.duration, on_update=on_update, stop_flag=stop_flag
    )
    if not times_list:
        print("No data recorded. Check port and Arduino output.")
        sys.exit(1)

    times = np.array(times_list)
    angles_deg = np.array(angles_list)
    print(f"Recorded {len(times)} samples over {times[-1] - times[0]:.2f} s.")

    # Final update of the live plot (no inline system identification).
    update_final_plot(text_sysid)
    fig.canvas.draw_idle()

    filepath = write_recordings_csv(times, angles_deg)
    print(f"Saved recording to {filepath}")

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
