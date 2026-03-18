"""
Live 2x2 plot of the 4 motor PID outputs from Arduino Serial.

Expected teleplot-style lines from arduino_test.ino (throttled ~100 ms):
  >pid_outputs(FL, FR, BL, BR):123,120,130,128
  ---

The plot appends one sample per frame on the '---' separator to keep values aligned.
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


DEFAULT_PORT = "COM10"
BAUD_RATE = 250000
WINDOW_SECONDS = 20.0

PREFIX_PID_OUTPUTS = ">pid_outputs(FL, FR, BL, BR):"
PREFIX_FRAME_END = "---"

ParseValue = Union[float, List[float], None]


def parse_args():
    parser = argparse.ArgumentParser(description="Live plot 4 PID outputs (FL/FR/BL/BR) from Arduino Serial.")
    parser.add_argument(
        "--port",
        "-p",
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


def parse_line(line: str) -> Tuple[Optional[str], ParseValue]:
    line = line.strip()
    if not line:
        return None, None
    if line == PREFIX_FRAME_END:
        return "frame_end", None
    if line.startswith(PREFIX_PID_OUTPUTS):
        payload = line[len(PREFIX_PID_OUTPUTS) :].strip()
        try:
            parts = [p.strip() for p in payload.split(",")]
            if len(parts) != 4:
                return None, None
            vals = [float(p) for p in parts]
            return "pid_outputs", vals
        except ValueError:
            return None, None
    return None, None


def setup_plot(window_sec: float):
    plt.ion()
    fig, axs = plt.subplots(2, 2, figsize=(12, 7), sharex=True, sharey=True)
    axes = [axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]]
    titles = ["Front Left", "Front Right", "Back Left", "Back Right"]

    lines = []
    for ax, title in zip(axes, titles):
        ln, = ax.plot([], [], color="C0", linewidth=0.9, label="PID output")
        lines.append(ln)
        ax.set_xlim(0, window_sec)
        ax.grid(True, alpha=0.3)
        ax.set_title(title)

    axes[0].legend(loc="upper left")
    for ax in axes[2:]:
        ax.set_xlabel("Time (s)")
    for ax in (axes[0], axes[2]):
        ax.set_ylabel("PID output (motor cmd units)")

    fig.suptitle(f"Motor PID outputs ({window_sec:.0f} s window)")
    plt.tight_layout()
    return fig, axes, lines


def run(port: str, window_sec: float):
    ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
    time.sleep(0.5)

    fig, axes, lines = setup_plot(window_sec)

    t_list: List[float] = []
    pid_lists: List[List[float]] = [[], [], [], []]

    window_start = time.perf_counter()
    last_pid: List[float] = [0.0, 0.0, 0.0, 0.0]

    try:
        while True:
            if not ser.in_waiting:
                if t_list:
                    for i in range(4):
                        lines[i].set_data(t_list, pid_lists[i])
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

            key, value = parse_line(line)
            if key is None:
                continue

            now = time.perf_counter()
            t_elapsed = now - window_start

            if key == "pid_outputs":
                if isinstance(value, list) and len(value) == 4:
                    last_pid = value
            elif key == "frame_end":
                t_list.append(t_elapsed)
                for i in range(4):
                    pid_lists[i].append(last_pid[i])

            if t_elapsed >= window_sec:
                window_start = now
                t_list.clear()
                for lst in pid_lists:
                    lst.clear()

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

