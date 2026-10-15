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

Press **Ctrl+C** to stop a run. The CLI exits without a KeyboardInterrupt traceback
(exit status 130) and reports the telemetry path. Buffered CSV rows are flushed
and the file is closed **before** controller cleanup, so a controller shutdown
failure does not discard the recorded data. If interruption occurs between
dynamics steps, the last completed state is also saved when it is newer than the
last scheduled telemetry row; this final row may fall between the usual sampling
times. A partially executed dynamics step is not written. Interruption during
controller startup leaves a valid CSV header if output was opened, and no rows
until dynamics have completed.

`simulation.checkpoint_steps: 1000` saves a telemetry checkpoint every 1000
completed dynamics steps (the default when omitted). Set it to `0` to disable
periodic checkpoints; otherwise it must be a positive integer. The interval in
simulated seconds is `checkpoint_steps * step_s`, independent of control and
telemetry periods. Each checkpoint writes the latest completed state if it is
not already a scheduled CSV row, then flushes the CSV and calls `fsync` to sync
it to disk. Coincident sampling and checkpoints produce only one row. Normal
exit and Ctrl+C also flush and sync remaining rows before controller cleanup.
Checkpoints append to `output_csv` (normally `telemetry.csv`); they do not create
separate files or save the controller/dynamics state needed to resume a run.
They have no effect when CSV output is disabled. A forced termination can lose
data acquired since the latest checkpoint; smaller intervals reduce that window
at the cost of more disk I/O and potentially more CSV rows.

## Orbit and solar production only

Run the independent `orbit_solar` mode and its plotter:

```sh
.venv/bin/python -m cubesat_sim config.orbit_solar.yaml
.venv/bin/python -m utilities.plot_solar orbit_solar.csv -o plots/solar.png
```

This mode propagates a bound Keplerian orbit in the J2000 frame under spherical
Earth point-mass gravity. It solves the two-body ellipse directly; it does not
advance attitude or instantiate Basilisk dynamics, sensors, WMM, SITL, or HITL.
Only the `simulation`, `orbit` (position and velocity), `environment` (Earth
radius, gravitational parameter, Sun radius), and `solar` sections are needed.
Unbound, radial, and Earth-intersecting orbits are rejected. J2, drag, and
third-body perturbations are not included.

Solar generation uses the existing time-varying solar ephemeris, inverse-square
irradiance, and finite-disk Earth eclipse model (including penumbra).
`solar.effective_area_m2` is an **assumed projected collecting area**, before
cell efficiency, that stays constant regardless of orientation. Choose it for
your scenario: a Sun-facing panel's active area, or total one-sided panel area
divided by four for an ideal isotropic orientation average. The example uses
0.015 m², 30% cell efficiency, and 95% conversion efficiency. It does not predict
actual attitude-dependent panel illumination. Power is:

```text
irradiance_at_1au × (AU / spacecraft_to_sun_distance)²
× visible_sun_fraction × effective_area × efficiency × conversion_efficiency
```

Energy means available electrical energy at the bus, before battery charge
limits, storage losses, curtailment, or loads. None of those are simulated.
Zero effective area disables generation without disabling orbit/eclipse output.

The CSV includes time, position, velocity, instantaneous `solar_power_W`,
`sunlight_fraction`, cumulative `solar_energy_Wh`, and `orbit_period_s`.
Energy is trapezoidally integrated at every `step_s`, independently of telemetry
spacing. Initial and final samples are always saved, with extra samples on both
sides of transitions into/out of full umbra. Checkpoint flushing and Ctrl+C
saving are supported. Decrease `step_s` to check convergence near eclipse
boundaries; a coarse step can miss short eclipses altogether.

The plotter shows cumulative energy, interval-average power with visible Sun
fraction, energy per orbital period measured from the simulation epoch, and
energy per sunrise-to-sunrise cycle. Sunrise is the first integration sample
with positive visible Sun after full umbra, so its timing has step-size
precision. Orbit-boundary energies are interpolated between cumulative samples;
reduce `telemetry_period_s` for finer allocation. Incomplete first/last cycles
and the last partial orbit are marked separately and excluded from printed
complete-interval averages. Continuous sunlight or eclipses without two recorded
sunrises produce no complete sun cycle. All segment energies still sum to the
energy produced over the recorded interval. Add `--show` for an interactive plot.

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

`step_s` is the Basilisk dynamics interval. `magnetometer_period_s`, `accelerometer_period_s`, `gyroscope_period_s`, `sun_period_s`, and `control_period_s` independently schedule the five ADCS calls; the current actuator command is held between control calls. `field_period_s` schedules WMM evaluations, and `telemetry_period_s` controls CSV size for long runs. All periods and `duration_s` must be integer multiples of `step_s`. Callbacks at the same time run in this order: magnetometer, accelerometer, gyroscope, Sun, battery status, control, then propagation. Battery status is sent at `control_period_s` immediately before each control request, including time zero. `clock: simulated` runs without wall clock pacing; `clock: realtime` aligns steps with wall time. HITL requires real time.

## SITL/HITL line protocol

Both modes use ASCII lines with space separated decimal integers and `\n` termination. SITL uses the child process's stdin/stdout. For HITL, set `simulation.mode: hitl`, `simulation.clock: realtime`, and `hitl.port`; UART uses 8N1. No JSON parser is needed on the MCU.

Each magnetometer sends its raw simulated reading in its configured sensor frame, including bias and noise, without calibration or sensor fusion. Readings are quantized to integer nT, not hardware ADC counts. One `M` line is sent per sensor at each magnetometer update, with the same timestamp and a zero-based `sensor_id` matching its position in `sensors.magnetometers`. Any nonempty number of magnetometers is supported. The Sun vector is a unit vector in the body frame when valid, otherwise zero. Both transports use the integer units below, so SITL now has the same quantization as HITL.

Two accelerometers are configured in the supplied SITL/HITL YAML files. Each sends one `C` line per `accelerometer_period_s` update, including time zero. IDs are zero-based indices into `sensors.accelerometers`, independent of magnetometer IDs; all accelerometers in an update share a timestamp. Any nonempty accelerometer list is supported. Each reading is `body_to_sensor × specific_force_body + bias_m_s2 + noise`, with independent zero-mean Gaussian noise per axis and sensor, specified by `noise_std_m_s2`. Values are rounded to the nearest integer µm/s² (ties to even), with identical encoding on both transports. These are raw sensor-frame values without calibration or fusion, not ADC counts or units of g.

One three-axis gyroscope is configured as the single mapping `sensors.gyroscope` (not a list). It sends one `G time_ms x y z` line, without a sensor ID, at time zero and every `simulation.gyroscope_period_s`. It measures spacecraft angular velocity relative to the inertial frame: `body_to_sensor × omega_BN_B + bias_rad_s + noise`. The rotation is a proper 3×3 body-to-sensor matrix, bias is a finite three-vector in rad/s, and `noise_std_rad_s` is a finite nonnegative Gaussian standard deviation per sample per axis in rad/s. Noise has zero mean and samples are independent across axes and updates; zero disables noise. The supplied configurations use body alignment, zero bias, and an illustrative standard deviation of `1e-4` rad/s. Bias drift, scale-factor error, and saturation are not modeled. Both transports round to integer µrad/s (rad/s × 1,000,000, ties to even); these are raw sensor-frame readings, not degrees/s or ADC counts.

Existing SITL/HITL configurations must add `simulation.gyroscope_period_s` (positive, finite, and an integer multiple of `step_s`) and `sensors.gyroscope`, as in `config.example.yaml`. Controllers must parse or explicitly ignore `G` messages; rebuild the C example, which decodes them into `adcs_gyroscope(time_s, rate_rad_s)` while retaining B-dot control. Include one additional line per gyroscope update when budgeting UART bandwidth. Orbit-only solar mode does not instantiate the gyroscope or require these keys.

**Before each SITL/HITL run, verify that every magnetometer, accelerometer, and gyroscope's `body_to_sensor` rotation matches the firmware calibration for that sensor (and ID where applicable).** The firmware's sensor-to-body rotation must be the inverse (transpose) of the configured `body_to_sensor` matrix before readings are combined or used for body-frame control. Identity firmware calibration requires body-aligned sensors (`body_to_sensor: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]`), as configured for both sensor pairs and the gyroscope in `config.blast.yaml`. The other example configurations may use rotated sensors; check their calibration explicitly.

Accelerometers measure non-gravitational specific force at Basilisk's body reference point, using `nonConservativeAccelpntB_B`. Both sensors are co-located there; sensor offsets and rotational lever-arm acceleration are not modeled. With the current central-gravity and torque-only dynamics, ideal readings are zero in free fall, with configured bias and noise added. Orbital gravitational acceleration is not an accelerometer reading.

Existing configurations must add `simulation.accelerometer_period_s` (a positive integer multiple of `step_s`) and `sensors.accelerometers`, as in `config.example.yaml`. Controllers must parse or explicitly ignore `C` messages before running this version. Rebuild the C example using the Quick start command; it decodes `C` into `adcs_accelerometer(time_s, sensor_id, accel_m_s2)` while retaining its B-dot control law. `A` remains the actuator response message. Include both accelerometer lines when budgeting UART bandwidth.

| Direction | Line | Units |
| --- | --- | --- |
| Simulator → Controller | `M time_ms sensor_id x y z` | magnetic readings, nT |
| Simulator → Controller | `C time_ms sensor_id x y z` | specific force, µm/s² (integer = m/s² × 1,000,000) |
| Simulator → Controller | `G time_ms x y z` | angular velocity, µrad/s (integer = rad/s × 1,000,000) |
| Simulator → Controller | `S time_ms valid sx sy sz` | Sun body vector, signed Q15 (`32767` = 1) |
| Simulator → Controller | `B time_ms soc_permille power_mW` | battery state of charge, 0–1000; signed net battery power, mW |
| Simulator → Controller | `T tick time_ms period_ms` | run one "tick" of the firmware |
| Controller → Simulator | `A tick mx my mz boom` | dipoles, µA·m²; boom target, mm |

Example exchange:

```text
M 1000 0 10000 0 -10000
M 1000 1 0 20000 0
C 1000 0 1000 -500 250
C 1000 1 -400 900 -100
G 1000 10000 -20000 15000
S 1000 1 32767 0 0
B 1000 800 -750
T 5 1000 1000
A 5 100000 0 -100000 500
```

The `A` tick must match the preceding `T` tick. The controller sends one `A` line after each `T`; no response is needed for `M`, `C`, `G`, `S`, or `B`. `S` has zero vector components when `valid=0`. Missing, stale, or malformed actuator lines stop the simulation. `sitl.timeout_s` or `hitl.timeout_s` is the response deadline for the selected mode. Dipoles are clipped to the configured per-axis limits; boom targets are clipped to travel limits and rate limited. This replaces the previous two-sensor `M` format; controller firmware must update its parser. The example controller receives every sensor but uses only sensor 0 for B-dot control. At 115200 baud, the two sample `M` lines together take about 4 ms on the wire, so sensor periods can be set independently of a faster dynamics step.

`B time_ms soc_permille power_mW` is sent immediately before every `T`, using the
same simulation timestamp, in both SITL and HITL. `soc_permille` is the current
stored energy divided by capacity, multiplied by 1000 and rounded to the nearest
integer (800 means 80.0%). `power_mW` is signed net **battery** power, rounded to
the nearest milliwatt: positive means charging, negative means discharging, and
zero means no stored-energy change. It includes charging losses and full/empty
battery clamping; it is not gross panel output or actuator demand. For example,
`B 1000 800 -750` reports 80.0% charge and 0.750 W discharge at one second.

Power describes the last completed dynamics step, matching CSV `battery_power_W`,
not the average since the previous control request. At time zero the message is
`B 0 <round(battery_initial_soc * 1000)> 0` (for example, `B 0 800 0`
with the supplied 0.8 setting): no dynamics interval has elapsed yet. Battery updates are independent
of CSV output and `telemetry_period_s`. Controllers must add a `B` parser (or
explicitly ignore it) before using this version; rebuild the example executable
with the Quick start command. The C example decodes it into `adcs_battery(time_s,
soc, power_W)` without changing its B-dot control law.

## Environment and physical model

- The simulator evaluates Basilisk's [World Magnetic Model](https://hanspeterschaub.info/basilisk/Documentation/simulation/environment/magneticFieldWMM/magneticFieldWMM.html) from spacecraft position and simulation UTC. The included [NOAA WMM2025 coefficients](https://www.ncei.noaa.gov/products/world-magnetic-model/wmm-coefficients) cover 2025–2029. A custom coefficient file can be selected with `environment.wmm_coefficients`. Runs outside its five-year model interval are rejected.
- Earth orientation is initialized from `simulation.start_utc` and updated before every WMM evaluation at `field_period_s`, including time zero. The planet-state message supplies the J2000-to-Earth-fixed rotation and its derivative, with Earth at the inertial origin. WMM transforms spacecraft position into Earth-fixed coordinates to evaluate the geographic field and returns the field in J2000 coordinates. CSV `Bx_T`, `By_T`, and `Bz_T` remain inertial components; sensors and magnetic torque use the subsequent body-frame rotation.
- The dependency-free Earth-orientation approximation combines [IAU 1982 Greenwich mean sidereal time](https://github.com/liberfa/erfa/blob/master/src/gmst82.c) with [IAU 1976 precession from J2000](https://github.com/liberfa/erfa/blob/master/src/pmat76.c). UTC approximates UT1 for sidereal rotation and TT for precession. Nutation, polar motion, Earth-orientation corrections, and leap-second crossings are omitted. This includes the dominant daily rotation and date-dependent precession, but is not a precision ITRF transformation. No extra configuration is required. Telemetry generated before this change must be rerun to include the rotating geographic magnetic field.
- A date-dependent solar ephemeris computes Earth-to-Sun direction and distance. Subtracting the spacecraft position gives the local Sun vector. A finite solar disk and spherical Earth determine full and partial eclipse for both charging and sensor visibility. The Sun sensor reports a direction whenever some sunlight remains and the Sun center is within its field of view; sensor noise does not depend on illumination. This is a low precision solar position model intended for long detumbling runs.
- Magnetorquer torque is **m × B** using the WMM field in the body frame. Each configured magnetometer has independent rotation, bias, and Gaussian noise.
- The boom is a massless rod with a point tip mass. Extension changes Basilisk's spacecraft inertia and the gravity gradient torque. The inertia rate adds a reaction torque. `tip_mass_kg`, `axis_body`, initial and allowed lengths, and maximum extension speed are in the config. Flexible boom dynamics are not modeled.
- The initial orbit state uses Earth centered inertial coordinates with J2000 mean equator/equinox axes. The template's orbit is near circular LEO. Central Earth gravity is used for propagation.

`simulation.seed` makes sensor noise repeatable. Accelerometers and the gyroscope each use a separate random stream, so changing either update rate does not change the other sensors’ noise sequences. CSV telemetry includes position, true body-frame angular rate (the `wx_rad_s`, `wy_rad_s`, and `wz_rad_s` columns, not noisy gyroscope readings), applied dipoles, boom length, the three diagonal moments of inertia, the WMM field, and battery/power values described below. The default six-hour simulated-clock run has 108,000 dynamics steps and writes telemetry once per second.

## Battery and solar power

Every run initializes stored energy to
**`spacecraft.battery_initial_soc * spacecraft.battery_capacity_Wh`**.
`battery_initial_soc` is a fraction from 0 to 1 inclusive: 0 is empty, 0.8 is 80%,
and 1 is full. The supplied configs set 0.8; change this field to choose the starting charge.
Capacity is usable energy in watt-hours, not amp-hours. Stored energy stays between
zero and capacity. `config.example.yaml` includes the required power fields;
copy these fields into older configs. Hardware values (10 Wh battery, 1 W nominal load, 0.5 W per
magnetorquer, six 0.01 m² solar faces, and efficiencies) are illustrative and must
be replaced with your spacecraft's values.

| Configuration field under `spacecraft` | Meaning |
| --- | --- |
| `battery_capacity_Wh` | Positive finite usable battery energy capacity. |
| `battery_initial_soc` | Finite initial state-of-charge fraction in [0, 1], including empty and full. |
| `nominal_power_W` | Constant nonnegative finite bus demand in W, added to magnetorquer demand every dynamics step; set to zero to disable. |
| `magnetorquer_max_power_W` | Three nonnegative finite electrical powers, X/Y/Z, at each axis's maximum absolute dipole. |
| `solar_charging.irradiance_W_m2_at_1au` | Positive solar irradiance at one astronomical unit; example 1361 W/m². |
| `solar_charging.conversion_efficiency` | Panel-to-bus wiring, MPPT and regulator efficiency, in (0, 1]. |
| `solar_charging.battery_charge_efficiency` | Fraction of charger input power stored as battery energy, in (0, 1]. |
| `solar_charging.max_charge_power_W` | Positive maximum charger **input** power, after solar power supplies nominal and actuator demand. |
| `solar_charging.panels` | List of independently illuminated, fixed body-mounted panels; `[]` disables generation. |
| `solar_charging.panels[].normal_body` | Outward-facing body-frame normal; a finite nonzero vector, normalized on loading. |
| `solar_charging.panels[].area_m2` | Positive active solar-cell area, excluding unused face area. |
| `solar_charging.panels[].efficiency` | Electrical cell conversion efficiency at the assumed operating condition, in (0, 1]. |

`environment.sun_radius_m` sets the positive finite solar radius used by charging
and Sun sensor eclipse geometry (example: 6.957 × 10⁸ m). Move the old
`spacecraft.solar_charging.sun_radius_m` field into `environment` when updating
older configs; the old location is no longer used.

`spacecraft.nominal_power_W` represents continuous background consumption. It is
independent of orientation, position, sunlight, and component activity. It applies
even with all actuators idle, and continues to count as demand when the battery is
empty. All supplied configs use an illustrative 1 W; replace it with the desired
constant load. Existing battery plots and the `B` protocol message automatically
include its effect on net battery power and stored energy after rerunning.

Magnetorquer load is linear in the magnitude of the **applied, clipped** dipole:

```text
P_mtq = sum_i(P_max_i * abs(m_i) / m_max_i)
```

A zero maximum dipole disables that axis and contributes zero power. Reversing
polarity leaves consumption unchanged. The linear law is the requested actuator
approximation, not a coil resistance/current-squared model.

Solar generation uses the actual spacecraft attitude and local Sun direction,
independently of Sun sensor noise, field of view, or callback period:

```text
P_solar = irradiance_at_1au * (AU / distance_to_Sun)^2 * sunlight_fraction
          * conversion_efficiency
          * sum_panels(area * efficiency * max(0, normal_body dot sun_direction_body))
```

Panels collect direct sunlight on their front face only. The visible fraction of
the Sun is calculated from the overlap of the apparent Earth and Sun disks,
including full light, penumbra, umbra, and annular occultation. The geometry follows
the [Basilisk eclipse disk-overlap model](https://hanspeterschaub.info/bskOlderDocs/bsk_1_7_5/Documentation/simulation/environment/eclipse/eclipse.html).
The example solar irradiance and radius use
[IAU 2015 Resolution B3 nominal values](https://iauarchive.eso.org/static/resolutions/IAU2015_English.pdf).
The approximate solar elements refer to the equator/equinox of date; an inverse
IAU 1976 precession rotation converts them to the spacecraft's J2000 frame before
computing incidence and eclipse. This corrects a former frame mismatch of about
0.373° at the example September 2026 epoch. Rerun older telemetry to include it.

At each dynamics step, solar power is averaged between the start and end states
(trapezoidal integration); magnetorquer demand is held over the step. For average
surplus `S = P_solar - P_mtq - P_nominal`, the battery energy rate in watts is
`min(S, max_charge_power_W) * battery_charge_efficiency` when `S > 0`, or `S`
otherwise. Multiply by `step_s / 3600` to obtain the change in Wh and clamp to
capacity. Discharge is ideal (100% efficiency). Generation beyond the charge
limit or remaining capacity is discarded. Reduce `step_s` to resolve rapid
tumbling, eclipse transitions, or battery saturation; events within a step are
approximated by that step's average power.

This is **power tracking only**: at empty, unmet demand is accumulated while
commanded actuator dynamics continue. There is no brownout, controller reset, or
automatic actuator shutdown. Battery state is available as `plant.power` and in
CSV telemetry; the SITL/HITL `B` message sends state of charge and net battery power before each control request.

Other components can be represented collectively by the constant nominal load.
Individual activity-dependent draws from avionics, radio, payload, sensors,
heaters, boom motor, and idle drivers/regulators are not separately modeled;
choose the nominal value to cover the intended background consumption without
double-counting the separately modeled magnetorquers.
The model also omits self-discharge, aging, temperature evolution, voltage/current
curves, charge taper, panel degradation, spacecraft/boom self-shadowing, Earth
albedo, atmospheric refraction, solar limb darkening, and lunar eclipses. Panel
efficiencies are fixed operating-point values. The spherical-Earth disk model and
existing low precision ephemeris provide a geometric engineering approximation,
not a thermal/electrical battery or high precision eclipse model.

The following columns are appended to the existing CSV:

| Column | Meaning |
| --- | --- |
| `battery_energy_Wh` | Stored energy at the row timestamp. |
| `battery_soc` | Stored energy / capacity, from 0 to 1. |
| `solar_power_W` | Available solar bus power, after conversion losses. |
| `magnetorquer_power_W` | Total actuator electrical demand. |
| `nominal_power_W` | Constant configured background electrical demand. |
| `battery_power_W` | Actual stored-energy change rate; positive charging, negative discharging, includes capacity clamping. |
| `sunlight_fraction` | Visible solar disk fraction at the row timestamp, from 0 to 1. |
| `unserved_power_W` | Demand that the empty battery and solar generation could not supply. |
| `unserved_energy_Wh` | Cumulative unmet demand since simulation start. |
| `solar_energy_Wh` | Cumulative solar energy available at the bus, after panel/converter losses. |
| `magnetorquer_energy_Wh` | Cumulative requested actuator energy, including unmet demand. |
| `nominal_energy_Wh` | Cumulative nominal demand, including unmet demand; nominal power × elapsed seconds / 3600. |
| `charging_loss_Wh` | Cumulative loss converting accepted charger input into stored energy. |
| `curtailed_energy_Wh` | Cumulative surplus rejected by the charge-power limit or full battery. |

Power columns average the **last dynamics step**, not the whole telemetry interval.
Energy columns incorporate every dynamics step even when CSV sampling is sparse.
The first row is written at the first telemetry sample or enabled checkpoint,
whichever comes first (or earlier on a graceful interrupt);
the configured initial state is available on `Plant` construction, not as a time-zero row.

Use the cumulative counters to audit energy, rather than integrating sparsely
sampled power columns. Every row satisfies this balance, up to floating-point
roundoff (all terms in Wh):

```text
battery_initial_soc * capacity + solar_energy_Wh + unserved_energy_Wh
  = battery_energy_Wh + magnetorquer_energy_Wh + nominal_energy_Wh
    + charging_loss_Wh + curtailed_energy_Wh
```

The counters start at zero and update every dynamics step. Curtailed energy
includes rejected solar surplus; charging loss includes only energy accepted by
the charger. Older CSVs lack these counters, and the intervening power history
cannot be recovered exactly. See [the power audit](docs/power-audit.md) for checks,
findings from the September 2026 recording, and hardware-calibration limitations.

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
.venv/bin/python -m utilities.plot_battery telemetry.csv
.venv/bin/python -m utilities.plot_angular_velocity telemetry.csv
.venv/bin/python -m utilities.plot_y_thomson telemetry.csv
.venv/bin/python -m utilities.animate_orbit telemetry.csv --config config.yaml
```

The plots are saved to `plots/magnetorquers.png`, `plots/ggb_extension.png`,
and `plots/angular_velocity_body.png` (or `plots/angular_velocity_world.png`
with `--frame world`). An explicit `-o/--output` overrides the default filename. They show applied X/Y/Z magnetic dipoles in
A·m², actual gravity gradient boom extension in meters, and X/Y/Z body angular
velocity plus its Euclidean magnitude in rad/s. X is roll, Y is pitch, and Z is
yaw. Use `--frame world` to express the same angular velocity in the fixed Earth
centered inertial (ECI) axes, or `--frame body` for the default. World-frame plots
use the recorded attitude (`sigma_BN_x`, `sigma_BN_y`, `sigma_BN_z`); rerun the
simulation for older CSVs that lack these columns. The magnitude is unchanged.
For example:

```sh
.venv/bin/python -m utilities.plot_angular_velocity telemetry.csv --frame world
```

For **Y-Thomson nadir pointing**, use `plot_y_thomson`, which saves
`plots/y_thomson_pointing.png`. This evaluates the recorded true attitude, not
the firmware's estimated attitude or deployment decision. Its right-handed
target frame is **+X along-track, +Y opposite orbital angular momentum, +Z
toward Earth**. This assumes the configured +Z boom is intended to point nadir;
verify this axis/sign convention against your ADCS target. The orbit normal is
`h = r × v`; along-track is the velocity direction with its radial component
removed. The pitch/orbit-normal, yaw/local-vertical, roll/along-track convention
and one-revolution-per-orbit motion are described by
[Lewis and Zajac (1964)](https://content-animation.org.uk/htmls/lewis.htm).

Five rows separate the +Z nadir error from the X/Y alignment errors, followed
by full attitude rotation error (0–180 degrees), orbit-relative body angular
rates and their magnitude (degrees/second), and boom extension (meters). The
left column shows the selected window; the right shows its final orbit, with
an enlarged Z-error scale to expose wobble. Z error is directed: 0 degrees
means toward Earth, 90 sideways, and 180 away from Earth. Successful pointing
keeps both attitude errors and orbit-relative rates near zero over multiple
orbits. A 180-degree flip remains a large error, and nadir alignment alone
does not hide an incorrect Y-axis direction. No mission-specific pass/fail
tolerance is assumed.

The rate residual is `omega_BN_B - C_BN * (r × v / |r|²)`, using the simulator's
fixed orbit plane. Thus successful pointing has nonzero inertial angular rate:
approximately `[0, -n, 0]` in the target body frame, with `n` the orbital rate
(about 0.065 degrees/second for the nominal ISS orbit). A low inertial spin
magnitude alone does not establish Y-Thomson pointing.

Existing CSVs are supported: velocity is estimated from recorded positions with
second-order finite differences on the actual time grid, before applying the
display window. At least three samples, position, attitude MRPs, body rates,
and boom length are required. Use densely sampled telemetry (e.g. 1 second);
sparse samples can alias tumbling and degrade the orbital-rate estimate,
especially at the endpoints. The utility does not model orbit-plane precession.
Time is displayed in hours for long runs; select acquisition or steady-state
windows in seconds, for example:

```sh
.venv/bin/python -m utilities.plot_y_thomson telemetry.csv --start-s 0 --end-s 600 -o plots/y_thomson_acquisition.png
```

The battery utility saves `plots/battery.png`, with three panels sharing a time axis:

- Battery level as a single curve, with **stored energy (Wh) on the left** and
  **SoC (%) on the right**. Capacity is inferred from recorded energy/SoC to align
  the two scales; no config is needed. An entirely empty recording only labels
  zero on both axes because its capacity cannot be inferred.
- Unsmoothed solar bus power and net battery power in W.
- Smoothed charging power (positive, gold), discharging power (negative, red),
  and their sum (net power, green).

The bottom panel uses interval power derived from stored-energy changes, regardless
of `--power-mode`. It splits that power into `max(P, 0)` and `min(P, 0)` before
smoothing; these are battery charging/discharging contributions, not gross solar
generation and electrical load. Opposite flows within a CSV interval cannot be
separated from its net energy change. The same centered, time-weighted moving
average is applied to each component, so their sum is the smoothed net power.
The default window is **300 seconds**; change it with `--smooth-window-s 60`, for
example. Irregular sample spacing is weighted by elapsed time, and recording-edge
windows use only the available duration without padding with zeros. Smoothing uses
the full recording before applying a display time window, so zooming does not
change the filtered values. It is a display filter and does not alter simulation
energy, telemetry, or the unsmoothed power panel.

The power panel matches the orbit animation's colors: **yellow/gold solar bus power**
and **green net battery power**, with a legend. Positive net power means charging;
negative means discharging, with a dashed zero line.
It reads `battery_soc`, `battery_energy_Wh`, and `solar_energy_Wh` directly from telemetry; no
configuration file is needed. By default, power is the average over each interval
between CSV rows: `3600 * (energy_next - energy_previous) / (time_next - time_previous)`.
Solar power uses the change in `solar_energy_Wh` over the same interval.
Each horizontal segment spans that exact interval. This includes all intermediate
dynamics steps and matches the battery-energy curve, even when sampled power misses
actuator pulses. No power is inferred before the first recorded energy sample;
at least two rows are needed. This is an energy-derived interval mean, not a
moving-average filter, and real rotation/eclipse variations remain visible.

Use `--power-mode sampled` to inspect the original `solar_power_W` and `battery_power_W` samples,
which describe only the last dynamics step at each CSV timestamp. With a 0.01 s
dynamics step and 1 s telemetry, that is one step out of 100; it can miss actuator
duty cycles and differ from the interval average. Generation and consumption are
both integrated at `simulation.step_s`: solar is averaged across each step's
endpoints and actuator commands are held over that same step. For supply/load
comparisons over CSV intervals, likewise difference `solar_energy_Wh`,
`magnetorquer_energy_Wh`, and `nominal_energy_Wh`, rather than comparing isolated
power samples. Charge losses and curtailed/unserved energy complete the balance.

During full eclipse, solar power is zero but magnetorquer demand can still vary.
While energy remains in the battery, net power is exactly
`-(nominal_power_W + magnetorquer_power_W)`. Pulsed actuation and changes in the
commanded dipole therefore produce discharge fluctuations without sunlight.
Measurement noise can also influence the controller's commands; the simulator
does not add noise directly to power. With zero torquer demand, constant nominal
load and no sunlight, discharge is constant until the battery empties.

Use `--start-s` and `--end-s` to inspect individual rotation cycles instead of
compressing hours of variation onto one plot, for example:

```sh
.venv/bin/python -m utilities.plot_battery telemetry.csv --start-s 2000 --end-s 2200 -o plots/battery_detail.png
```

Rerun the simulation if an older CSV lacks these columns. Use `-o/--output` to
choose another output path or `--show` to also display the plot interactively.

All time axes use simulation seconds. Magnetorquer plots hold each sampled
value until the next sample; changes between telemetry samples cannot be recovered.

The animation defaults to `plots/orbit.gif` and selects its layout from the CSV
columns. With complete attitude and power telemetry, it synchronizes four views:

- Recorded orbit position in J2000 Earth-centered inertial (ECI) coordinates,
  with Earth drawn to scale and enlarged body axes at the spacecraft.
- Spacecraft attitude in a fixed ECI close-up: a schematic cube, colored body
  X/Y/Z axes, configured outward panel normals P1/P2/etc., and the Sun direction.
  Body vectors use the transpose of the recorded inertial-to-body MRP rotation.
  The cube and normal lengths are illustrative; physical panel mounting locations
  and spacecraft dimensions are not defined by the configuration.
- Recorded solar bus and net battery power, with a cursor at the frame timestamp.
- Per-panel bus power estimated from orientation, configured active area and
  efficiencies, inverse-square Sun distance, and recorded `sunlight_fraction`.
  Eclipse state and recorded versus estimated total solar power are shown.

Use the config that produced the telemetry. The Sun direction uses the actual
spacecraft-to-Sun vector from the simulator's ephemeris and the config's epoch;
only its drawn length is shortened. Gold panel normals receive sunlight; gray
normals face away or are eclipsed. Panel estimates use endpoint geometry, while
recorded power averages the last dynamics step, so small differences are expected.
Larger differences can indicate a mismatched config or telemetry from an older
solar model. No Basilisk installation is needed for the animation.

Every frame uses position, attitude, illumination and power from the **same actual
CSV row**. MRPs are never linearly interpolated, avoiding artifacts when equivalent
MRP representations switch. By default up to 300 rows nearest uniformly spaced
target times span the recording at 20 playback frames per second. Actual sample
times and epoch timestamps appear above the figure; irregular telemetry yields
approximate playback timing. Sampling cannot reconstruct rotations between rows,
and a long run compressed into few frames can make rotation appear to jump or
reverse. Increase `--frames` or select a shorter interval with `--start-s` and
`--end-s` to inspect illumination changes. The time window is inclusive and uses
only existing rows.

For `orbit_solar` runs or older CSVs without the complete attitude and power
columns, the utility displays only the orbit animation: Earth, the recorded
satellite path and position, the Sun direction, and timestamps. No spacecraft
panel configuration or controller data is required. The minimum CSV columns are
`time_s`, `x_m`, `y_m`, and `z_m`; the config needs `simulation.start_utc` and
`environment.earth_radius_m`. The four-view dashboard additionally requires all
three `sigma_BN_*` columns, `solar_power_W`, `battery_power_W`, and
`sunlight_fraction`, plus `spacecraft.solar_charging` in the config.

```sh
.venv/bin/python -m utilities.animate_orbit orbit_solar.csv --config config.orbit_solar.yaml -o plots/orbit_solar.html
```

Use `utilities.plot_solar` for the separate energy-production plots from an
orbit-only run.

All scripts accept an optional CSV path (default `telemetry.csv`), `-o/--output`,
and `--show` to also open a window. Static plots support PNG, PDF, and SVG output.
The animation supports GIF and interactive HTML with playback controls:

```sh
.venv/bin/python -m utilities.animate_orbit telemetry.csv --config config.yaml --frames 180 --fps 15 -o plots/orbit.html
.venv/bin/python -m utilities.animate_orbit telemetry.csv --config software_config.yaml --start-s 900 --end-s 1200 --frames 301 -o plots/eclipse_exit.html
.venv/bin/python -m utilities.plot_angular_velocity telemetry.csv -o plots/angular_velocity.svg --show
```

The utilities require NumPy, Matplotlib, and Pillow, plus PyYAML for animation;
they can process existing telemetry without loading Basilisk.

## Firmware estimator comparison

With `simulation.output_csv: telemetry.csv`, SITL/HITL also writes
`telemetry_estimator.csv` (the output stem plus `_estimator.csv`). It records
pre-step ground truth at **every dynamics step**, independently of the main
telemetry period: body-frame `omega_BN_B` in rad/s and attitude MRPs `sigma_BN`
(inertial-to-body). Each control response is recorded at that same pre-step
instant, with `estimate_present`, `estimate_valid`, its sensor epoch
`estimate_time_s`, and `estimate_w{x,y,z}_rad_s`. No response on an intervening
step, or a legacy controller, gives `estimate_present=0`. Invalid/missing values
are zero placeholders, not measurements. Both CSVs are flushed at checkpoints
and closed on exit, including Ctrl+C. Full-cadence logging increases disk usage.

From the simulator directory:

```sh
.venv/bin/python -m utilities.plot_estimator telemetry_estimator.csv -o plots/estimator.png
```

Add `--show` to open the plot. It overlays body rates, plots estimate-minus-truth
errors, reports per-axis RMSE, and shows truth attitude. Truth rates are linearly
interpolated to the **sensor epoch**, not the response arrival time. Held epochs
are scored only once; invalid estimates and epochs outside recorded truth are
excluded. Use a dynamics step fine enough to resolve the motion. These are
diagnostic errors, not a pass/fail accuracy specification.

The YUAA firmware currently estimates rates only during pointing measurement
windows, after detumbling and boom deployment. An early/short run can therefore
contain no valid estimates. It has no attitude estimator; only attitude truth is
recorded/plotted. Rates are the absolute fused inertial angular velocity expressed
in body axes, not the high-pass libration residual.

The optional versioned response extension is:

```text
A tick mx my mz boom E1 rate_time_ms rate_valid wx wy wz
```

The first six fields retain their existing units. `rate_time_ms` is the firmware
sensor timestamp (uint32 milliseconds); `rate_valid` is 0 or 1; `wx wy wz` are
signed int32 body-frame **microradians/second**. Invalid rates serialize as zero.
The simulator accepts both this 12-field form and legacy six-field responses.
Old simulator versions need updating to consume the extended YUAA response.
Comparison requires firmware and simulator to share the simulated sensor clock
(the usual T-driven mode); autonomous firmware-clock streaming is not supported
by this lockstep comparison. Runs spanning uint32 timestamp rollover must be
split; the plotting tool does not unwrap firmware timestamps.
