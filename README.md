# Inverted Pendulum Cart

Firmware and analysis tools for a four-wheel cart balancing an inverted pendulum on an Arduino Giga R1.

The repository contains:

- embedded code in `arduino_test/` for sensing, estimation, control, and motor actuation
- Python utilities in `util/` for calibration, logging, plotting, system identification, and controller gain generation
- supporting notes and hardware PDFs in `docs/`

## Project structure

```text
.
|-- arduino_test/   Main robot firmware and controller modules
|-- util/           Offline Python tools and recorded CSV logs
|-- docs/           Hardware datasheets and modelling notes
|-- examples/       Extra experimental C++ code
|-- lib/            Placeholder Arduino library folder
|-- test/           Placeholder test folder
```

Key files inside `arduino_test/`:

- `arduino_test.ino`: top-level control loop, button handling, task selection, telemetry
- `drive.cpp/.h`: Motoron motor-controller setup and four-motor commands
- `motor_encoder.cpp/.h`: wheel encoder interrupts and distance conversion
- `pendulum_encoder.cpp/.h`: pendulum quadrature decoder with index-based zeroing
- `motor_pid.cpp/.h`: per-wheel speed PID loops
- `lqr.h`: shared state-feedback controller class used for both LQR and pole-placement gains
- `localisation_kalman.h`: 4-state Kalman filter for cart position/velocity and pendulum angle/angular velocity

## Robot architecture

The robot uses a cascaded control structure:

1. Wheel encoders provide cart displacement for all four wheels.
2. The pendulum encoder provides an unwrapped pendulum angle in radians, with the index pulse resetting the encoder count to zero.
3. A Kalman filter estimates the full state:
   `x`, `x_dot`, `theta`, `theta_dot`
4. A state-feedback controller computes a cart force:
   `F = -K (state - target)`
5. The force is integrated into a target cart velocity:
   `v_target += (F / M_TOTAL) * dt`
6. Four independent PID loops drive each wheel to that target velocity.
7. A deadband compensation offset is added before sending motor commands to the Motoron drivers.

Important runtime protections in `arduino_test/arduino_test.ino`:

- control is disabled and all motors are stopped if `|pendulum angle| > 50 deg`
- pausing the robot resets the wheel PID integrators and velocity target before resuming
- controller outputs are saturated to configured force limits before being converted into velocity demand

## Hardware assumptions from the code

- MCU: Arduino Giga R1
- Motor drivers: two Motoron I2C controllers on `Wire1`
  - front controller I2C address `17`
  - back controller I2C address `16`
- Pendulum encoder pins:
  - `A = 9`
  - `B = 11`
  - `index = 8`
- Buttons:
  - start/pause/resume button (red wired) on pin `10` 
  - controller select button (purple wired) on pin `12`
  - task-mode select button (white wired) on pin `13`
- Wheel encoder pins:
  - front left: `27`, `28`
  - front right: `22`, `23`
  - back left: `29`, `30`
  - back right: `50`, `49`

## Control modes and task modes

The firmware exposes two controller families and three task modes.

### Controller families

- `LQR` (`control_mode = 0`)
  - uses gains stored in `lqr_stabilise`, `lqr_recovery`, and `lqr_sprint`
- `POLE` (`control_mode = 1`)
  - uses pole-placement gains stored in `pole_stabilise`, `pole_recovery`, and `pole_sprint`

Both controller families use the same `LQRController` class. The only difference is the gain vector `K`.

### Task modes

- `Stabilisation` (`task_mode = 0`)
  - holds the cart at `x_ref = 0`
  - target state is upright and stationary
- `Sprint` (`task_mode = 1`)
  - commands a straight-line cart translation using a time-stretched reference ramp
  - current code uses `SPRINT_DISTANCE_M = 2.0f`
  - the pendulum reference stays upright throughout the move
- `Recovery` (`task_mode = 2`)
  - uses dedicated recovery gains while `|theta| > 3 deg`
  - automatically switches back to stabilisation gains once the pendulum is close enough to upright

## How to operate the robot

### Before powering the control loop

1. Upload the firmware in `arduino_test/` to the Arduino Giga R1.
2. Open a serial monitor or one of the Python live-plot tools at `250000` baud if you want telemetry.
3. Place the robot in a safe area with enough room for the selected task.
4. Ensure the pendulum encoder is calibrated by holding the pendulum vertically.

### Selecting modes before start

On power-up, the sketch waits in `setup()` until the start button is pressed.

While waiting:

- hold the control-select button on pin `12` for about `200 ms` to cycle:
  - `LQR -> POLE -> LQR`
- hold the task-select button on pin `13` for about `200 ms` to cycle:
  - `Stabilisation -> Sprint -> Recovery -> Stabilisation`

The active selection is printed to Serial.

### Starting

- press the red wired start button on pin `10`
- the sketch waits about `1 s`
- the main control loop begins

If sprint mode is selected, the sprint reference is initialised on the first loop iteration using the current estimated cart position as the starting point.

### Pausing and resuming during operation

The start button is also used as a pause/resume control:

- hold for at least `20 ms` while running to enter pause
- release, then hold again for about `200 ms` to resume

On resume, the firmware:

- resets all wheel PID integrators
- resets timing
- zeros the integrated target velocity
- resets sprint state flags

This helps avoid windup and discontinuities when the robot restarts.

### What the robot is doing in each mode

`Stabilisation`

- best for standing the pendulum upright around the origin
- controller target is `[x_ref, xdot_ref, theta_ref, theta_dot_ref] = [0, 0, 0, 0]`

`Sprint`

- moves the cart by a fixed distance while keeping the pendulum reference upright
- the code computes a trapezoidal or triangular move duration internally, then applies a slower linear ramp using `SPRINT_LINEAR_V_SCALE`
- once the move completes, the target position is held

`Recovery`

- meant for catching larger upright-angle disturbances than the normal stabiliser handles comfortably
- when the estimated pendulum angle is outside the `3 deg` threshold, recovery gains are used
- inside that threshold, the firmware falls back to the normal stabilisation gains

## Serial telemetry format

The firmware streams teleplot-style lines, followed by `---` as an end-of-frame marker. Example topics include:

- `>motor_encoders(FL, FR, BL, BR):...`
- `>motor_speeds(FL, FR, BL, BR):...`
- `>pid_outputs(FL, FR, BL, BR):...`
- `>desired:...`
- `>estimated_velocity:...`
- `>pendulum_angle:...`
- `>lqr_force:...`
- `>kalman_x:...`
- `>theta_ref:...`
- `>sprint_active:...`

The Python tools in `util/` parse this exact stream format.

## Offline Python utilities

Run scripts from the repository root unless noted otherwise.

### Python dependencies

Core logging and plotting scripts use:

```bash
pip install pyserial matplotlib numpy
```

Gain-generation scripts also need:

```bash
pip install control scipy
```

### Main workflow

1. Record live robot telemetry or pendulum-only angle data.
2. Calibrate the pendulum angle offset if needed.
3. Run offline plots or system identification on the recorded CSVs.
4. Recompute controller gains if the physical parameters or design targets change.

### `util/plot_pendulum.py`

Purpose:

- records only the pendulum angle from Serial
- shows a live angle and angular-velocity plot
- saves a CSV to `util/recordings/`

Expected serial input:

- `>pendulum_angle:<value>`
- also accepts the legacy format `Pendulum Angle: <value>`

Examples:

```bash
python util/plot_pendulum.py
python util/plot_pendulum.py --port COM3 --duration 20
python util/plot_pendulum.py -p COM5 -d 0
```

Notes:

- default port is `COM10`
- default baud is `250000`
- `--duration 0` runs until you press `q` in the plot window
- output CSV format is `time_s,angle_deg`

### `util/calibrate_pendulum.py`

Purpose:

- estimates the pendulum encoder offset so `0 deg` corresponds to the physical midpoint between the stoppers

Required recording procedure:

1. hold the pendulum at the front stopper
2. move it to the back stopper and hold
3. move it back to the front stopper and hold

Example:

```bash
python util/calibrate_pendulum.py pendulum_2026-03-03_11-15-25.csv
python util/calibrate_pendulum.py pendulum_2026-03-03_11-15-25.csv --threshold 6 --min-plateau-samples 3 --write-offset-file
```

What it does:

- detects plateau segments from low angular-rate intervals
- treats segment order as `front -> back -> front`
- computes the midpoint and prints the offset to apply in firmware

Current firmware integration:

- the live code already applies the offset directly in radians:
  `angle_rad = get_pendulum_angle_rad() + CALIBRATION_OFFSET_DEG * pi/180`
- the offset constant is:
  `#define CALIBRATION_OFFSET_DEG (0.0f)`

### `util/identify_system.py`

Purpose:

- identifies pendulum natural frequency and damping from a freely oscillating recording

Input expectations:

- CSV with `time_s,angle_deg`
- use a clean interval where the pendulum oscillates freely about its equilibrium

Examples:

```bash
python util/identify_system.py util/recordings/pendulum_2026-03-03_11-15-25.csv
python util/identify_system.py util/recordings/pendulum_2026-03-03_11-15-25.csv --t-start 3 --t-end 12
python util/identify_system.py util/recordings/pendulum_2026-03-03_11-15-25.csv --t-start 3 --t-end 12 --no-plot
```

Output:

- `omega_n`
- `zeta`
- `f_n`
- `T`

The script recenters the selected window around its mean angle before fitting the second-order model.

### `util/live_logs.py`

Purpose:

- logs the full telemetry frame stream from `arduino_test.ino` into a single CSV row per frame

Examples:

```bash
python util/live_logs.py
python util/live_logs.py --port COM3 --max-frames 200
python util/live_logs.py --out util/recordings/my_run.csv
```

Output:

- CSV saved to `util/recordings/arduino_live_log_<timestamp>.csv` unless `--out` is provided

Useful columns include:

- wheel encoder distances
- wheel speeds
- wheel PID outputs
- desired speed
- estimated velocity
- pendulum angle
- LQR force
- Kalman position and angle
- sprint activity flag

### Live plotting tools for telemetry

`util/kalman_graph.py`

- live plot of pendulum angle and Kalman estimated cart velocity

`util/lqr_graph.py`

- live 2x2 plot of each wheel speed against the Kalman predicted velocity

`util/pid_outputs_graph.py`

- live 2x2 plot of the four wheel PID outputs

`util/pwm_graph.py`

- live plot of desired speed and actual speed
- the code still parses PWM, but the PWM line is currently commented out in the plot itself

Examples:

```bash
python util/kalman_graph.py --port COM10
python util/lqr_graph.py --port COM10 --window 20
python util/pid_outputs_graph.py --port COM10
python util/pwm_graph.py --port COM10
```

### `util/plot_graphs.py`

Purpose:

- plots logged telemetry from a CSV produced by `util/live_logs.py`

Default behavior:

- if `--csv` is not given, it defaults to `util/recordings/eval_c_pole.csv`

Example:

```bash
python util/plot_graphs.py --csv util/recordings/arduino_live_log_20260324_101546.csv
python util/plot_graphs.py --csv util/recordings/eval_c_pole.csv --save-dir util/plots
```

It plots:

- pendulum angle vs time
- cart velocity vs time
- cart position vs time

### Gain and modelling scripts

`util/dynamics.py`

- nonlinear pendulum-cart dynamics and linearisation helper
- central reference for the physical parameters used in analysis

`util/stable_gains.py`

- generates LQR and pole-placement gains for the default stabilisation task

`util/recovery_gains.py`

- generates gains tuned for the recovery task

`util/sprint_gains.py`

- generates gains tuned for the sprint task

`util/lqr_gains_apk.py`

- older or alternate gain-generation script
- note that its `M_cart_total` is `1.515`, while the current firmware and the other gain scripts use `1.593`

Each gain script prints four gains in the format expected by the firmware:

```cpp
LQRController controller(k1, k2, k3, k4);
```

## Notes on consistency

A few details are worth keeping in mind when working on the project:

- the root repository currently does not include a `platformio.ini`, even though the earlier stub README mentioned PlatformIO
- the authoritative runtime behavior is the source in `arduino_test/arduino_test.ino`
- `util/calibrate_pendulum.py` prints instructions for a slightly older insertion method, but the current firmware already applies `CALIBRATION_OFFSET_DEG` directly to `angle_rad`
- some comments in the Python tools describe older experiment setups; use the actual code and current firmware constants when in doubt

## Supporting material

- `docs/state_space.tex`: state-space derivation notes
- `docs/accelerations_for_sim.md`: simulation notes
- `docs/*.pdf`: hardware datasheets for the Arduino, motor driver, and motor
- `util/system_parameters.txt`: example identified pendulum parameters

## Typical end-to-end workflow

1. Tune or regenerate gains in `util/stable_gains.py`, `util/recovery_gains.py`, or `util/sprint_gains.py`.
2. Copy the selected gain values into `arduino_test/arduino_test.ino`.
3. Set `CALIBRATION_OFFSET_DEG` if needed.
4. Upload the firmware to the Arduino Giga R1.
5. Select the controller family and task mode using the hardware buttons before starting.
6. Run the robot.
7. Capture telemetry with `util/live_logs.py` or pendulum-only data with `util/plot_pendulum.py`.
8. Review performance with the plotting scripts and repeat.
