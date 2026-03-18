"""
Live plot of desired speed, actual speed, PWM, and Kalman estimated_velocity from Arduino Serial.
Uses the same teleplot-style output as arduino_test.ino:
  >desired:0.2
  >actual:0.123
  >raw_v1:0.130
  >motor_speeds(FL, FR, BL, BR):0.10,0.11,0.09,0.10
  >pwm:0.456
  >estimated_velocity:0.19
  >kalman_v_pred:0.18

Streams for a 20-second window, then clears and continues from the left (MATLAB-style).
"""

import argparse
import sys
import time
from typing import List, Optional, Tuple, Union

import matplotlib.pyplot as plt

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
PREFIX_ESTIMATED_VELOCITY = ">estimated_velocity:"
PREFIX_RAW_V1 = ">raw_v1:"
PREFIX_KALMAN_V_PRED = ">kalman_v_pred:"
PREFIX_FRAME_END = "---"
PREFIX_MOTOR_SPEEDS = ">motor_speeds(FL, FR, BL, BR):"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Live plot desired, actual, PWM and Kalman estimated_velocity from Arduino (20s rolling window)."
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
    """Create 2x2 figure: each motor speed (blue) vs Kalman predicted velocity (red)."""
    plt.ion()
    fig, axs = plt.subplots(2, 2, figsize=(12, 7), sharex=True, sharey=True)
    axes = [axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]]
    titles = ["Front Left", "Front Right", "Back Left", "Back Right"]

    motor_lines = []
    pred_lines = []
    for ax, title in zip(axes, titles):
        line_motor, = ax.plot([], [], color="C0", linestyle="-", linewidth=0.9, label="Motor speed (m/s)", zorder=2)
        line_pred, = ax.plot([], [], color="red", linestyle="-", linewidth=0.9, label="Kalman v_pred (m/s)", zorder=1)
        motor_lines.append(line_motor)
        pred_lines.append(line_pred)

        ax.set_xlim(0, window_sec)
        ax.grid(True, alpha=0.3)
        ax.set_title(title)

    axes[0].legend(loc="upper left")
    for ax in axes[2:]:
        ax.set_xlabel("Time (s)")
    for ax in (axes[0], axes[2]):
        ax.set_ylabel("Speed (m/s)")

    fig.suptitle(f"Motor speeds vs Kalman predicted velocity ({window_sec:.0f} s window)")
    plt.tight_layout()
    return fig, axes, motor_lines, pred_lines


ParseValue = Union[float, List[float], None]


def parse_line(line: str) -> Tuple[Optional[str], ParseValue]:
    """Return (key, value) for teleplot lines, or (None, None)."""
    line = line.strip()
    if line == PREFIX_FRAME_END:
        return "frame_end", None
    if line.startswith(PREFIX_MOTOR_SPEEDS):
        payload = line[len(PREFIX_MOTOR_SPEEDS):].strip()
        try:
            parts = [p.strip() for p in payload.split(",")]
            if len(parts) != 4:
                return None, None
            vals = [float(p) for p in parts]
            return "motor_speeds", vals
        except ValueError:
            return None, None
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
    if line.startswith(PREFIX_RAW_V1):
        try:
            return "raw_v1", float(line[len(PREFIX_RAW_V1):])
        except ValueError:
            return None, None
    if line.startswith(PREFIX_PWM):
        try:
            return "pwm", float(line[len(PREFIX_PWM):])
        except ValueError:
            return None, None
    if line.startswith(PREFIX_ESTIMATED_VELOCITY):
        try:
            return "estimated_velocity", float(line[len(PREFIX_ESTIMATED_VELOCITY):])
        except ValueError:
            return None, None
    if line.startswith(PREFIX_KALMAN_V_PRED):
        try:
            return "kalman_v_pred", float(line[len(PREFIX_KALMAN_V_PRED):])
        except ValueError:
            return None, None
    return None, None


def run(port, window_sec):
    ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
    time.sleep(0.5)

    fig, axes, motor_lines, pred_lines = setup_plot(window_sec)

    t_list = []
    motor_lists = [[], [], [], []]  # FL, FR, BL, BR
    pred_list = []

    window_start = time.perf_counter()
    last_kalman_v_pred: float = 0.0
    last_motor_speeds: List[float] = [0.0, 0.0, 0.0, 0.0]

    try:
        while True:
            if not ser.in_waiting:
                # Update plot with current buffers
                if t_list:
                    for i in range(4):
                        motor_lines[i].set_data(t_list, motor_lists[i])
                        pred_lines[i].set_data(t_list, pred_list)

                    for ax in axes:
                        ax.relim()
                        ax.autoscale_view(scalex=False, scaley=True)
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
                # kept for backward compatibility; not plotted in 2x2 view
                pass
            elif key == "motor_speeds":
                if isinstance(value, list) and len(value) == 4:
                    last_motor_speeds = value
            elif key == "estimated_velocity":
                # kept for backward compatibility; not plotted in 2x2 view
                pass
            elif key == "kalman_v_pred":
                if isinstance(value, float):
                    last_kalman_v_pred = value
            elif key == "frame_end":
                # One point per frame (Arduino prints a '---' separator each frame)
                t_list.append(t_elapsed)
                for i in range(4):
                    motor_lists[i].append(last_motor_speeds[i])
                pred_list.append(last_kalman_v_pred)

            # Rolling window: after window_sec, clear and continue from the left
            if t_elapsed >= window_sec:
                window_start = now
                t_list.clear()
                for lst in motor_lists:
                    lst.clear()
                pred_list.clear()

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
