"""
Live plot of desired speed, actual speed, and PWM from Arduino Serial.
Uses the same teleplot-style output as arduino_test.ino:
  >desired:0.2
  >actual:0.123
  >pwm:0.456

Streams for a 20-second window, then clears and continues from the left (MATLAB-style).
"""

import argparse
import re
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# Default serial port (match arduino_test.ino upload port)
DEFAULT_PORT = "COM10"
BAUD_RATE = 250000
WINDOW_SECONDS = 20.0

# Prefixes sent by arduino_test.ino (throttled ~100 ms)
PREFIX_DESIRED = ">desired:"
PREFIX_ACTUAL = ">actual:"
PREFIX_PWM = ">pwm:"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Live plot desired, actual speed and PWM from Arduino (20s rolling window)."
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
    """Create MATLAB-style figure with left axis for speed (m/s), right for PWM."""
    plt.ion()
    fig, ax_left = plt.subplots(figsize=(10, 5))
    ax_right = ax_left.twinx()

    line_desired, = ax_left.plot([], [], color="C0", linestyle="-", linewidth=0.75, label="Desired (m/s)", zorder=3)
    line_actual, = ax_left.plot([], [], color="C1", linestyle="-", linewidth=0.75, label="Actual (m/s)", zorder=2)
    # line_pwm, = ax_right.plot([], [], color="C2", linestyle="-", linewidth=0.75, label="PWM", zorder=1)
    line_pwm = None  # PWM plot commented out

    ax_left.set_xlim(0, window_sec)
    ax_left.set_ylim(-0.75, 0.75)
    ax_right.set_ylim(-0.75, 0.75)
    ax_left.set_xlabel("Time (s)")
    ax_left.set_ylabel("Speed (m/s)", color="C0")
    # ax_right.set_ylabel("PWM", color="C2")
    ax_left.tick_params(axis="y", labelcolor="C0")
    # ax_right.tick_params(axis="y", labelcolor="C2")
    ax_left.grid(True, alpha=0.3)
    ax_left.legend(loc="upper left")
    # ax_right.legend(loc="upper right")
    fig.suptitle(f"Desired / Actual speed and PWM ({window_sec:.0f} s window)")
    plt.tight_layout()
    return fig, ax_left, line_desired, line_actual, line_pwm


def parse_line(line):
    """Return (key, value) for teleplot lines, or (None, None)."""
    line = line.strip()
    if line.startswith(PREFIX_DESIRED):
        try:
            return "desired", float(line[len(PREFIX_DESIRED):])
        except ValueError:
            return None, None
    if line.startswith(PREFIX_ACTUAL):
        try:
            return "actual", float(line[len(PREFIX_ACTUAL):])
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

    fig, ax_left, line_desired, line_actual, line_pwm = setup_plot(window_sec)

    t_list = []
    desired_list = []
    actual_list = []
    pwm_list = []

    window_start = time.perf_counter()
    last_desired = 0.0
    last_actual = 0.0
    last_pwm = 0.0

    try:
        while True:
            if not ser.in_waiting:
                # Update plot with current buffers
                if t_list:
                    line_actual.set_data(t_list, actual_list)
                    # line_pwm.set_data(t_list, pwm_list)
                    line_desired.set_data(t_list, desired_list)
                    ax_left.relim()
                    ax_left.autoscale_view(scalex=False, scaley=True)
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

            if key == "desired":
                last_desired = value
            elif key == "actual":
                last_actual = value
            elif key == "pwm":
                last_pwm = value
                # One point per triple (desired, actual, pwm) from Arduino
                t_list.append(t_elapsed)
                desired_list.append(last_desired)
                actual_list.append(last_actual)
                pwm_list.append(last_pwm)

            # Rolling window: after window_sec, clear and continue from the left
            if t_elapsed >= window_sec:
                window_start = now
                t_list.clear()
                desired_list.clear()
                actual_list.clear()
                pwm_list.clear()

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
