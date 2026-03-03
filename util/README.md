# Util – pendulum recording, calibration, and system ID

This folder contains Python tools for the inverted-pendulum cart: live angle recording, encoder calibration, and offline system identification. Run scripts from the **project root** (or ensure paths to CSVs are correct).

**Dependencies:** `pyserial`, `matplotlib`, `numpy`. Install with:

```bash
pip install pyserial matplotlib numpy
```

---

## 1. `plot_pendulum.py` – live recording and plotting

Records pendulum angle from the Arduino over Serial, updates a plot in real time (MATLAB-style), and at the end saves a timestamped CSV under `util/recordings/`.

**Requirements:** Arduino running and printing lines like `Pendulum Angle: 123.45` at 115200 baud.

| Argument | Short | Default | Description |
|----------|--------|---------|-------------|
| `--port` | `-p` | `COM10` | Serial port (e.g. `COM3` on Windows, `/dev/ttyUSB0` on Linux). |
| `--duration` | `-d` | `30.0` | Recording length in seconds. Use `0` to run until you press `q` in the plot window. |

**Examples:**

```bash
# Record for 30 s on default port (COM10)
python util/plot_pendulum.py

# Specify port and 20 s duration
python util/plot_pendulum.py --port COM3 --duration 20

# Short form
python util/plot_pendulum.py -p COM5 -d 45

# Run until you press 'q' in the plot window (duration 0)
python util/plot_pendulum.py -d 0
```

**Output:** CSV in `util/recordings/pendulum_YYYY-MM-DD_HH-MM-SS.csv` with columns `time_s`, `angle_deg`.

---

## 2. `calibrate_pendulum.py` – encoder calibration (0° = middle)

Computes the calibration offset so that **0° is at the physical middle** between the two stoppers. Use a CSV where you: **start at front (rest) → move to back (rest) → move to front (rest)**. The script detects the three plateaus in time order and prints the offset to add in the Arduino code.

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `file` | *(required)* | Path to dataset CSV. If only a filename is given, `util/recordings/` is tried. |
| `--threshold` | `4.0` | Max angular rate (deg/s) to treat as a plateau. |
| `--min-plateau-samples` | `5` | Minimum consecutive samples in a plateau. |
| `--write-offset-file` | off | Write the numeric offset to `util/recordings/calibration_offset.txt`. |

**Examples:**

```bash
# Use a recording in util/recordings (full path)
python util/calibrate_pendulum.py util/recordings/pendulum_2026-03-03_11-15-25.csv

# Bare filename: script looks in util/recordings/
python util/calibrate_pendulum.py pendulum_2026-03-03_11-15-25.csv

# Looser plateau detection and save offset to file
python util/calibrate_pendulum.py pendulum_2026-03-03_11-15-25.csv --threshold 6 --min-plateau-samples 3 --write-offset-file
```

**Output:** Prints front/back plateau centers, physical middle, and **offset (deg)**. Also prints the exact `#define` and line to add in `arduino_test.ino`. With `--write-offset-file`, the offset is written to `util/recordings/calibration_offset.txt`.

---

## 3. `identify_system.py` – system identification from CSV

Estimates pendulum dynamics (natural frequency, damping) from a **freely oscillating** recording, with the pendulum **hanging down at 0°**. Uses only a time window `[t_start, t_end]` of the CSV. CSV format must be `time_s`, `angle_deg` (as produced by `plot_pendulum.py`).

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `file` | *(required)* | Path to dataset CSV. |
| `--t-start` | first sample | Start time (s) of the window used for identification. |
| `--t-end` | last sample | End time (s) of the window. |
| `--no-plot` | off | Only print parameters; do not show plots. |

**Examples:**

```bash
# Use full recording
python util/identify_system.py util/recordings/pendulum_2026-03-03_11-15-25.csv

# Restrict to 3 s–12 s for system ID
python util/identify_system.py util/recordings/pendulum_2026-03-03_11-15-25.csv --t-start 3 --t-end 12

# Short form and no plot
python util/identify_system.py util/recordings/pendulum_2026-03-03_11-15-25.csv --t-start 3 --t-end 10 --no-plot
```

**Output:** Prints `omega_n` (rad/s), `zeta`, `f_n` (Hz), and period `T` (s). Without `--no-plot`, shows angle and angular velocity vs time plus a phase portrait for the selected window.

---

## Typical workflow

1. **Record:** Run `plot_pendulum.py` with the Arduino connected; move the pendulum as needed. CSVs go to `util/recordings/`.
2. **Calibrate:** Record a run with the sequence front → back → front; run `calibrate_pendulum.py` on that CSV; add the printed offset to the Arduino code.
3. **System ID:** Record free oscillation (pendulum down at 0°); run `identify_system.py` on the CSV with `--t-start` / `--t-end` over a clean swinging interval.
