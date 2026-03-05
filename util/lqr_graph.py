"""
Live plot of pendulum angle and estimated cart velocity from Arduino Serial (LQR mode).
Uses teleplot-style output from arduino_test.ino:
  >estimated_velocity:0.19
  >pendulum_angle:2.5

Streams for a 20-second window, then clears and continues from the left (MATLAB-style).
"""

import argparse
import sys
import time

import matplotlib.pyplot as plt

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# Default serial port (match arduino_test.ino upload port)
DEFAULT_PORT = "COM10"
BAUD_RATE = 115200
WINDOW_SECONDS = 20.0

# Prefixes sent by arduino_test.ino (throttled ~100 ms)
PREFIX_ESTIMATED_VELOCITY = ">estimated_velocity:"
PREFIX_PENDULUM_ANGLE = ">pendulum_angle:"
PREFIX_PWM = ">pwm:"  # used only to trigger new point (same sample as above)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Live plot pendulum angle and Kalman estimated cart velocity from Arduino (20s rolling window)."
    )
    parser.add_argument(
        "--port", "-p",
        default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=WINDOW_SECONDS,
        metavar="SECONDS",
        help=f"Plot window length in seconds (default: {WINDOW_SECONDS}).",
    )
    return parser.parse_args()


def setup_plot(window_sec):
    """Two subplots: pendulum angle (deg), estimated cart velocity (m/s)."""
    plt.ion()
    fig, (ax_angle, ax_velocity) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    line_angle, = ax_angle.plot([], [], color="C0", linestyle="-", linewidth=0.75, label="Pendulum angle (deg)")
    line_velocity, = ax_velocity.plot([], [], color="C1", linestyle="-", linewidth=0.75, label="Est. cart velocity (m/s)")

    ax_angle.set_xlim(0, window_sec)
    ax_angle.set_ylim(-180, 180)
    ax_angle.set_ylabel("Angle (deg)")
    ax_angle.grid(True, alpha=0.3)
    ax_angle.legend(loc="upper right")
    ax_angle.axhline(0, color="k", linewidth=0.5)

    ax_velocity.set_xlim(0, window_sec)
    ax_velocity.set_ylim(-0.75, 0.75)
    ax_velocity.set_xlabel("Time (s)")
    ax_velocity.set_ylabel("Velocity (m/s)")
    ax_velocity.grid(True, alpha=0.3)
    ax_velocity.legend(loc="upper right")
    ax_velocity.axhline(0, color="k", linewidth=0.5)

    fig.suptitle(f"LQR: Pendulum angle & estimated cart velocity ({window_sec:.0f} s window)")
    plt.tight_layout()
    return fig, ax_angle, ax_velocity, line_angle, line_velocity


def parse_line(line):
    """Return (key, value) for teleplot lines, or (None, None)."""
    line = line.strip()
    if line.startswith(PREFIX_ESTIMATED_VELOCITY):
        try:
            return "estimated_velocity", float(line[len(PREFIX_ESTIMATED_VELOCITY):])
        except ValueError:
            return None, None
    if line.startswith(PREFIX_PENDULUM_ANGLE):
        try:
            return "pendulum_angle", float(line[len(PREFIX_PENDULUM_ANGLE):])
        except ValueError:
            return None, None
    if line.startswith(PREFIX_PWM):
        try:
            return "pwm", float(line[len(PREFIX_PWM):])
        except ValueError:
            return None, None
    return None, None


def run(port, window_sec):
    ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
    time.sleep(0.5)

    fig, ax_angle, ax_velocity, line_angle, line_velocity = setup_plot(window_sec)

    t_list = []
    pendulum_angle_list = []
    estimated_velocity_list = []

    window_start = time.perf_counter()
    last_pendulum_angle = 0.0
    last_estimated_velocity = 0.0

    try:
        while True:
            if not ser.in_waiting:
                if t_list:
                    line_angle.set_data(t_list, pendulum_angle_list)
                    line_velocity.set_data(t_list, estimated_velocity_list)
                    ax_angle.relim()
                    ax_angle.autoscale_view(scalex=False, scaley=True)
                    ax_velocity.relim()
                    ax_velocity.autoscale_view(scalex=False, scaley=True)
                fig.canvas.draw_idle()
                plt.pause(0.02)
                continue

            raw = ser.readline()
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue

            key, value = parse_line(line)
            if key is None:
                continue

            now = time.perf_counter()
            t_elapsed = now - window_start

            if key == "pendulum_angle":
                last_pendulum_angle = value
            elif key == "estimated_velocity":
                last_estimated_velocity = value
            elif key == "pwm":
                # One point per print block (desired, actual, estimated_velocity, pendulum_angle, pwm)
                t_list.append(t_elapsed)
                pendulum_angle_list.append(last_pendulum_angle)
                estimated_velocity_list.append(last_estimated_velocity)

            if t_elapsed >= window_sec:
                window_start = now
                t_list.clear()
                pendulum_angle_list.clear()
                estimated_velocity_list.clear()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        plt.ioff()
        plt.close()


def main():
    args = parse_args()
    run(args.port, args.window)


if __name__ == "__main__":
    main()
