# CubeSat SITL/HITL simulator

Basilisk propagates the CubeSat orbit and attitude. The simulator exchanges compact ASCII lines with an ADCS subprocess over stdin/stdout in SITL or with an STM32 over UART in HITL. One YAML file configures the spacecraft, sensors, environment, timing, and interface.

## Quick start

Use the existing virtual environment containing the dependencies in `requirements.txt` (including `tqdm` for terminal progress). On macOS or Linux:

```sh
cc -O2 -Wall -Wextra examples/adcs_example.c -o adcs_example
cp config.example.yaml config.yaml
.venv/bin/python -m cubesat_sim config.yaml
```

The example executable uses basic B-dot control and keeps the boom retracted. Replace it with your ADCS implementation in any language. The output CSV is written beside the config file.

## SITL subprocess

Set `simulation.mode: sitl` and configure an executable and its arguments as a YAML list:

```yaml
sitl:
  command: [./adcs_example]
  startup_delay_s: 5.0
  timeout_s: 1.0
```

The simulator starts one process per run, sends sensor and control messages to its stdin, and reads actuator replies from its stdout using the same protocol as HITL below. Controller state persists between messages. The process runs in the config file's directory, so relative executable and argument paths are relative to that directory; bare executable names are searched on `PATH`. Commands run without a shell. For example, a Python controller can use `command: [python3, -u, controller.py]`.

`sitl.startup_delay_s` gives the process a wall-clock boot grace period before any sensor or control messages are sent. Set it to the platform’s boot time plus a margin; the example configuration allows five seconds. It must be finite and nonnegative and defaults to zero when omitted. Simulation time and realtime pacing start after this delay. A process that exits during startup stops the run immediately. This is a fixed delay, not a readiness handshake; stdout must remain protocol-only during boot as well.

Reserve stdout for actuator replies, flush each reply immediately, and send diagnostics to stderr (inherited by the simulator). `sitl.timeout_s` is a positive wall-clock deadline for each write and actuator response after startup, including during simulated-clock runs. A closed pipe, missing reply, or invalid reply stops the simulation. On completion or failure the simulator closes the process's stdin, waits briefly for exit, then terminates or kills it if needed and reaps it.

Existing shared-library configurations must replace `sitl.library` and the symbol settings with `sitl.command` and `sitl.timeout_s`. C and Rust controllers should now build as executables with a message loop; no shared library or C ABI is required.

## Independent timing

SITL and HITL display a terminal progress bar with completed steps, elapsed wall time, ETA, and current/total simulated seconds. HITL also prints received USB/UART data above the bar as `USB RX: b'A ...\n'`. Each entry is a raw received chunk, so partial replies and non-ASCII bytes remain visible before protocol validation. Progress and USB output go to stderr; CSV telemetry is unchanged.

`step_s` is the Basilisk dynamics interval. `magnetometer_period_s`, `sun_period_s`, and `control_period_s` independently schedule the three ADCS calls; the current actuator command is held between control calls. `field_period_s` schedules WMM evaluations, and `telemetry_period_s` controls CSV size for long runs. All periods and `duration_s` must be integer multiples of `step_s`. Callbacks at the same time run in this order: magnetometer, Sun, control, then propagation. `clock: simulated` runs without wall clock pacing; `clock: realtime` aligns steps with wall time. HITL requires real time.

## SITL/HITL line protocol

Both modes use ASCII lines with space separated decimal integers and `\n` termination. SITL uses the child process's stdin/stdout. For HITL, set `simulation.mode: hitl`, `simulation.clock: realtime`, and `hitl.port`; UART uses 8N1. No JSON parser is needed on the MCU.

The magnetometer vectors are in their configured sensor frames. The Sun vector is a unit vector in the body frame when valid, otherwise zero. Both transports use the integer units below, so SITL now has the same quantization as HITL.

| Direction | Line | Units |
| --- | --- | --- |
| Simulator → Controller | `M time_ms m1x m1y m1z m2x m2y m2z` | magnetic readings, nT |
| Simulator → Controller | `S time_ms valid sx sy sz` | Sun body vector, signed Q15 (`32767` = 1) |
| Simulator → Controller | `T tick time_ms period_ms` | run one "tick" of the firmware |
| Controller → Simulator | `A tick mx my mz boom` | dipoles, µA·m²; boom target, mm |

Example exchange:

```text
M 1000 10000 0 -10000 0 20000 0
S 1000 1 32767 0 0
T 5 1000 1000
A 5 100000 0 -100000 500
```

The `A` tick must match the preceding `T` tick. The controller sends one `A` line after each `T`; no response is needed for `M` or `S`. `S` has zero vector components when `valid=0`. Missing, stale, or malformed actuator lines stop the simulation. `sitl.timeout_s` or `hitl.timeout_s` is the response deadline for the selected mode. Dipoles are clipped to the configured per-axis limits; boom targets are clipped to travel limits and rate limited. At 115200 baud, the sample `M` line is about 4 ms on the wire, so sensor periods can be set independently of a faster dynamics step.

## Environment and physical model

- The simulator evaluates Basilisk's [World Magnetic Model](https://hanspeterschaub.info/basilisk/Documentation/simulation/environment/magneticFieldWMM/magneticFieldWMM.html) from spacecraft position and simulation UTC. The included [NOAA WMM2025 coefficients](https://www.ncei.noaa.gov/products/world-magnetic-model/wmm-coefficients) cover 2025–2029. A custom coefficient file can be selected with `environment.wmm_coefficients`. Runs outside its five-year model interval are rejected.
- Earth orientation is initialized from `simulation.start_utc` and updated before every WMM evaluation at `field_period_s`, including time zero. The planet-state message supplies the J2000-to-Earth-fixed rotation and its derivative, with Earth at the inertial origin. WMM transforms spacecraft position into Earth-fixed coordinates to evaluate the geographic field and returns the field in J2000 coordinates. CSV `Bx_T`, `By_T`, and `Bz_T` remain inertial components; sensors and magnetic torque use the subsequent body-frame rotation.
- The dependency-free Earth-orientation approximation combines [IAU 1982 Greenwich mean sidereal time](https://github.com/liberfa/erfa/blob/master/src/gmst82.c) with [IAU 1976 precession from J2000](https://github.com/liberfa/erfa/blob/master/src/pmat76.c). UTC approximates UT1 for sidereal rotation and TT for precession. Nutation, polar motion, Earth-orientation corrections, and leap-second crossings are omitted. This includes the dominant daily rotation and date-dependent precession, but is not a precision ITRF transformation. No extra configuration is required. Telemetry generated before this change must be rerun to include the rotating geographic magnetic field.
- A date-dependent solar ephemeris computes Earth-to-Sun direction. Subtracting the spacecraft position gives the local Sun vector; cylindrical Earth eclipse and sensor field of view determine visibility. This is a low precision solar position model intended for attitude sensing across long detumbling runs.
- Magnetorquer torque is **m × B** using the WMM field in the body frame. Two magnetometers have independent rotation, bias, and Gaussian noise.
- The boom is a massless rod with a point tip mass. Extension changes Basilisk's spacecraft inertia and the gravity gradient torque. The inertia rate adds a reaction torque. `tip_mass_kg`, `axis_body`, initial and allowed lengths, and maximum extension speed are in the config. Flexible boom dynamics are not modeled.
- The initial orbit state uses Earth centered inertial coordinates with J2000 mean equator/equinox axes. The template's orbit is near circular LEO. Central Earth gravity is used for propagation.

`simulation.seed` makes sensor noise repeatable. CSV telemetry includes position, angular rate, applied dipoles, boom length, the three diagonal moments of inertia, and the WMM field. The default six-hour simulated-clock run has 108,000 dynamics steps and writes telemetry once per second.

Run the focused tests with:

```sh
.venv/bin/python -m unittest discover -s tests -v
```

## Telemetry plots and orbit animation

Install the visualization dependencies in the virtual environment:

```sh
.venv/bin/python -m pip install matplotlib Pillow
```

Run these utilities from the repository root after a simulation:

```sh
.venv/bin/python -m utilities.plot_magnetorquers telemetry.csv
.venv/bin/python -m utilities.plot_ggb telemetry.csv
.venv/bin/python -m utilities.plot_angular_velocity telemetry.csv
.venv/bin/python -m utilities.animate_orbit telemetry.csv --config config.yaml
```

The plots are saved to `plots/magnetorquers.png`, `plots/ggb_extension.png`,
and `plots/angular_velocity.png`. They show applied X/Y/Z magnetic dipoles in
A·m², actual gravity gradient boom extension in meters, and X/Y/Z body angular
velocity plus its Euclidean magnitude in rad/s. X is roll, Y is pitch, and Z is
yaw. All time axes use simulation seconds. Magnetorquer plots hold each sampled
value until the next sample; changes between telemetry samples cannot be recovered.

The animation defaults to `plots/orbit.gif` and displays the recorded orbit in
Earth centered inertial coordinates, with Earth drawn to scale. The Sun direction
comes from the simulator's solar model using the supplied config's `start_utc`;
use the config that produced the telemetry. Sun distance and marker sizes are
reduced for visibility. The animation does not depict spacecraft attitude.
Up to 300 frames span the complete recording at 20 frames per second, using
linear position interpolation at evenly spaced simulation times. Sparse telemetry
can therefore make the path between samples approximate.

All scripts accept an optional CSV path (default `telemetry.csv`), `-o/--output`,
and `--show` to also open a window. Static plots support PNG, PDF, and SVG output.
The animation supports GIF and interactive HTML with playback controls:

```sh
.venv/bin/python -m utilities.animate_orbit telemetry.csv --config config.yaml --frames 180 --fps 15 -o plots/orbit.html
.venv/bin/python -m utilities.plot_angular_velocity telemetry.csv -o plots/angular_velocity.svg --show
```

The utilities require NumPy, Matplotlib, and Pillow, plus PyYAML for animation;
they can process existing telemetry without loading Basilisk.
