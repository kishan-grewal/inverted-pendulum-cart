"""
Log Arduino teleplot serial output to CSV.

Matches the output format in `arduino_test/arduino_test.ino`:
  - Multiple `Serial.println(...)` lines per frame
  - Each frame ends with a line containing exactly `---`

One CSV row is written per frame.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Union

try:
    import serial  # type: ignore
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)


DEFAULT_PORT = "COM10"  # match arduino_test.ino upload port (see kalman_graph.py)
BAUD_RATE = 250000

# Prefixes sent by arduino_test.ino (throttled ~100 ms)
PREFIX_FRAME_END = "---"

PREFIX_MOTOR_ENCODERS = ">motor_encoders(FL, FR, BL, BR):"
PREFIX_MOTOR_SPEEDS = ">motor_speeds(FL, FR, BL, BR):"
PREFIX_PID_OUTPUTS = ">pid_outputs(FL, FR, BL, BR):"

PREFIX_DESIRED = ">desired:"
PREFIX_ACTUAL = ">actual:"
PREFIX_RAW_V1 = ">raw_v1:"
PREFIX_ESTIMATED_VELOCITY = ">estimated_velocity:"
PREFIX_KALMAN_V_PRED = ">kalman_v_pred:"
PREFIX_PENDULUM_ANGLE = ">pendulum_angle:"
PREFIX_LQR_FORCE = ">lqr_force:"
PREFIX_KALMAN_THETA = ">kalman_theta:"
PREFIX_KALMAN_V = ">kalman_v:"
PREFIX_KALMAN_X = ">kalman_x:"
PREFIX_X_REF = ">x_ref:"
PREFIX_THETA_REF = ">theta_ref:"
PREFIX_SPRINT_ACTIVE = ">sprint_active:"
PREFIX_PWM = ">pwm:"
PREFIX_DT_CUMAVG_MS = ">dt_cumavg_ms:"


ParseValue = Union[float, int, List[float], None]


def _parse_float_payload(line: str, prefix: str) -> Optional[float]:
    payload = line[len(prefix) :].strip()
    try:
        return float(payload)
    except ValueError:
        return None


def _parse_4_floats_payload(line: str, prefix: str) -> Optional[List[float]]:
    payload = line[len(prefix) :].strip()
    parts = [p.strip() for p in payload.split(",")]
    if len(parts) != 4:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def parse_line(line: str) -> Tuple[Optional[str], ParseValue]:
    """
    Return (key, value) for teleplot lines, or (None, None).

    Keys are internal CSV fields or groups (motor/pid arrays).
    """
    line = line.strip()
    if not line:
        return None, None

    if line == PREFIX_FRAME_END:
        return "frame_end", None

    if line.startswith(PREFIX_MOTOR_ENCODERS):
        vals = _parse_4_floats_payload(line, PREFIX_MOTOR_ENCODERS)
        if vals is None:
            return None, None
        return "motor_encoders", vals

    if line.startswith(PREFIX_MOTOR_SPEEDS):
        vals = _parse_4_floats_payload(line, PREFIX_MOTOR_SPEEDS)
        if vals is None:
            return None, None
        return "motor_speeds", vals

    if line.startswith(PREFIX_PID_OUTPUTS):
        vals = _parse_4_floats_payload(line, PREFIX_PID_OUTPUTS)
        if vals is None:
            return None, None
        return "pid_outputs", vals

    if line.startswith(PREFIX_DESIRED):
        v = _parse_float_payload(line, PREFIX_DESIRED)
        return ("desired_speed", v) if v is not None else (None, None)

    if line.startswith(PREFIX_ACTUAL):
        v = _parse_float_payload(line, PREFIX_ACTUAL)
        return ("actual_speed_fl", v) if v is not None else (None, None)

    if line.startswith(PREFIX_RAW_V1):
        v = _parse_float_payload(line, PREFIX_RAW_V1)
        return ("raw_v1_fl", v) if v is not None else (None, None)

    if line.startswith(PREFIX_ESTIMATED_VELOCITY):
        v = _parse_float_payload(line, PREFIX_ESTIMATED_VELOCITY)
        return ("estimated_velocity", v) if v is not None else (None, None)

    if line.startswith(PREFIX_KALMAN_V_PRED):
        v = _parse_float_payload(line, PREFIX_KALMAN_V_PRED)
        return ("kalman_v_pred", v) if v is not None else (None, None)

    if line.startswith(PREFIX_PENDULUM_ANGLE):
        v = _parse_float_payload(line, PREFIX_PENDULUM_ANGLE)
        return ("pendulum_angle_deg", v) if v is not None else (None, None)

    if line.startswith(PREFIX_LQR_FORCE):
        v = _parse_float_payload(line, PREFIX_LQR_FORCE)
        return ("lqr_force", v) if v is not None else (None, None)

    if line.startswith(PREFIX_KALMAN_THETA):
        v = _parse_float_payload(line, PREFIX_KALMAN_THETA)
        return ("kalman_theta_rad", v) if v is not None else (None, None)

    if line.startswith(PREFIX_KALMAN_V):
        v = _parse_float_payload(line, PREFIX_KALMAN_V)
        return ("kalman_v", v) if v is not None else (None, None)

    if line.startswith(PREFIX_KALMAN_X):
        v = _parse_float_payload(line, PREFIX_KALMAN_X)
        return ("kalman_x", v) if v is not None else (None, None)

    if line.startswith(PREFIX_X_REF):
        v = _parse_float_payload(line, PREFIX_X_REF)
        return ("x_ref", v) if v is not None else (None, None)

    if line.startswith(PREFIX_THETA_REF):
        v = _parse_float_payload(line, PREFIX_THETA_REF)
        return ("theta_ref_deg", v) if v is not None else (None, None)

    if line.startswith(PREFIX_SPRINT_ACTIVE):
        # Printed as 1/0
        v = _parse_float_payload(line, PREFIX_SPRINT_ACTIVE)
        if v is None:
            return None, None
        return ("sprint_active", int(v)) if v.is_integer() else ("sprint_active", v)

    if line.startswith(PREFIX_PWM):
        v = _parse_float_payload(line, PREFIX_PWM)
        return ("pwm", v) if v is not None else (None, None)

    if line.startswith(PREFIX_DT_CUMAVG_MS):
        v = _parse_float_payload(line, PREFIX_DT_CUMAVG_MS)
        return ("dt_cumavg_ms", v) if v is not None else (None, None)

    return None, None


def build_default_out_csv(out_dir: str) -> str:
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(out_dir, f"arduino_live_log_{ts}.csv")


def run(
    port: str,
    baud_rate: int,
    out_csv: str,
    max_frames: Optional[int] = None,
) -> None:
    ser = serial.Serial(port, baud_rate, timeout=0.05)
    # Allow Arduino time to start streaming
    time.sleep(0.5)

    fieldnames = [
        "frame_idx",
        "t_sec",
        "dt_cumavg_ms",
        "motor_encoders_FL",
        "motor_encoders_FR",
        "motor_encoders_BL",
        "motor_encoders_BR",
        "motor_speeds_FL",
        "motor_speeds_FR",
        "motor_speeds_BL",
        "motor_speeds_BR",
        "pid_outputs_FL",
        "pid_outputs_FR",
        "pid_outputs_BL",
        "pid_outputs_BR",
        "desired_speed",
        "actual_speed_fl",
        "raw_v1_fl",
        "estimated_velocity",
        "kalman_v_pred",
        "pendulum_angle_deg",
        "lqr_force",
        "kalman_theta_rad",
        "kalman_v",
        "kalman_x",
        "x_ref",
        "theta_ref_deg",
        "sprint_active",
        "pwm",
    ]

    # Track the latest values within the current frame.
    value_fields: List[str] = [fn for fn in fieldnames if fn not in ("frame_idx", "t_sec")]
    current: Dict[str, Optional[Union[float, int]]] = {k: None for k in value_fields}

    frame_idx = 0
    window_start = time.perf_counter()

    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        try:
            while True:
                if not ser.in_waiting:
                    time.sleep(0.01)
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

                if key == "frame_end":
                    frame_idx += 1
                    t_sec = time.perf_counter() - window_start

                    writer.writerow(
                        {
                            "frame_idx": frame_idx,
                            "t_sec": t_sec,
                            **current,
                        }
                    )
                    f.flush()

                    if max_frames is not None and frame_idx >= max_frames:
                        break

                    # Reset for next frame
                    current = {k: None for k in value_fields}
                    continue

                # Populate fields for this frame.
                if key == "motor_encoders":
                    if isinstance(value, list) and len(value) == 4:
                        current["motor_encoders_FL"] = value[0]
                        current["motor_encoders_FR"] = value[1]
                        current["motor_encoders_BL"] = value[2]
                        current["motor_encoders_BR"] = value[3]
                elif key == "motor_speeds":
                    if isinstance(value, list) and len(value) == 4:
                        current["motor_speeds_FL"] = value[0]
                        current["motor_speeds_FR"] = value[1]
                        current["motor_speeds_BL"] = value[2]
                        current["motor_speeds_BR"] = value[3]
                elif key == "pid_outputs":
                    if isinstance(value, list) and len(value) == 4:
                        current["pid_outputs_FL"] = value[0]
                        current["pid_outputs_FR"] = value[1]
                        current["pid_outputs_BL"] = value[2]
                        current["pid_outputs_BR"] = value[3]
                else:
                    # scalar fields
                    if isinstance(value, (float, int)):
                        current[key] = value
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Log arduino_test serial teleplot output to CSV.")
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help=f"Baud rate (default: {BAUD_RATE})")
    parser.add_argument(
        "--out",
        default=None,
        help="Output CSV path. If omitted, writes into util/recordings/ as arduino_live_log_<timestamp>.csv",
    )
    parser.add_argument("--max-frames", type=int, default=None, help="Stop after N frames (for quick capture).")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    if args.out:
        out_csv = args.out
    else:
        recordings_dir = os.path.join(script_dir, "recordings")
        out_csv = build_default_out_csv(recordings_dir)

    run(port=args.port, baud_rate=args.baud, out_csv=out_csv, max_frames=args.max_frames)


if __name__ == "__main__":
    main()

