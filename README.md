# CubeSat SITL/HITL simulator

Basilisk propagates the CubeSat orbit and attitude. The simulator exchanges compact binary frames with an ADCS subprocess over stdin/stdout in SITL or with an STM32 over UART in HITL. One YAML file configures the spacecraft, sensors, environment, timing, and interface.

## Install Python and Basilisk

Run these commands from the simulator repository directory. Create a separate
virtual environment on each machine; do not copy `.venv` between operating
systems. The instructions use Python 3.13 on Windows/macOS; Ubuntu 22.04/24.04's
Python 3.10/3.12 also works with the pinned package.

Basilisk's Python import is `Basilisk`, but its official PyPI package is **`bsk`**.
`requirements.txt` pins `bsk==2.11.1`, matching the version used for local simulator
tests. The core package contains the dynamics and WMM modules used here; optical
navigation extras are unnecessary. See the [official installation guide](https://avslab.github.io/basilisk/Install.html).

### Windows 10/11 (64-bit x86, PowerShell)

Install **64-bit Python 3.13** from [python.org](https://www.python.org/downloads/windows/),
including the Python launcher (`py`). Then:

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install --only-binary=bsk -r requirements.txt
.\.venv\Scripts\python.exe -c "from importlib.metadata import version; from cubesat_sim.physics import Plant; print('Basilisk', version('bsk'), 'imports OK')"
```

These commands use the environment's interpreter directly, so PowerShell script
activation is unnecessary. For the Windows HITL commands below, use
`.\.venv\Scripts\python.exe` wherever `python` is shown. A prebuilt Basilisk
wheel does not require Visual Studio, CMake, Bash, or a compiler for HITL.

### macOS (Apple Silicon, macOS 11 or newer)

Install Python 3.13 from [python.org](https://www.python.org/downloads/macos/).
Use a native ARM64 terminal/Python, rather than an Intel process under Rosetta.

```sh
python3.13 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install --only-binary=bsk -r requirements.txt
.venv/bin/python -c "from importlib.metadata import version; from cubesat_sim.physics import Plant; print('Basilisk', version('bsk'), 'imports OK')"
```

An **Intel Mac** needs a Basilisk source build for this pinned release; follow the
[official source-build guide](https://avslab.github.io/basilisk/Build.html)
and its macOS prerequisites in the same virtual environment. The commands above
intentionally require a wheel instead of silently starting a source build.

### Linux (Ubuntu 22.04/24.04, x86-64 or ARM64)

Install Python and virtual-environment support, then install the prebuilt wheel:

```sh
sudo apt update
sudo apt install python3 python3-venv python3-pip
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install --only-binary=bsk -r requirements.txt
.venv/bin/python -c "from importlib.metadata import version; from cubesat_sim.physics import Plant; print('Basilisk', version('bsk'), 'imports OK')"
```

For other distributions, install equivalent Python/venv packages. The 2.11.1
Linux wheels require glibc 2.27 or newer; Alpine/musl requires a different setup.
Available architectures and Python constraints are listed in the
[Basilisk 2.11.1 release files](https://pypi.org/project/bsk/2.11.1/#files).

The import check exercises the simulator's physics imports without opening a
serial port or starting a simulation. If pip reports no matching `bsk` wheel,
check Python version and architecture against those release files, or use the
source-build guide. Keep Python, pip, and the simulator in the same environment.
The simulator includes its WMM coefficients in `data/WMM2025.COF`.

## Quick start

After completing the installation above, run the example SITL controller on
macOS or Linux (a C compiler is required for this example):

```sh
cc -O2 -Wall -Wextra examples/adcs_example.c -o adcs_example
cp config.example.yaml config.yaml
.venv/bin/python -m cubesat_sim config.yaml
```

The example executable uses basic B-dot control and keeps the boom retracted. Replace it with your ADCS implementation in any language. The output CSV is written beside the config file.

### Windows HITL

HITL uses `pyserial` on Windows, macOS, and Linux (115200 baud by default,
8 data bits, no parity, 1 stop bit, no flow control). In your Windows Python
environment with Basilisk installed, install the dependencies and list ports:

```powershell
python -m pip install -r requirements.txt
python -m serial.tools.list_ports
```

In your existing simulation YAML, set `simulation.mode: hitl` and
`simulation.clock: realtime`, and update the serial section:

```yaml
hitl:
  port: COM3  # replace with the STM32 port shown above (COM10+ also supported)
  baudrate: 115200
  timeout_s: 0.15
```

Run `python -m cubesat_sim config.yaml` using your configuration's filename.
Close any serial monitor holding that COM port before starting. The same
`timeout_s` bounds each outbound write and fresh actuator stream silence (or each lockstep reply),
including replies arriving in fragments. A disconnect ends the run with an
I/O error. Windows HITL does not use the SITL command or its build tools;
SITL subprocess transport remains Unix-only.

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

### FTDI latency for OBC HITL on Windows 11 / WSL2

For the OBC's USB-C FTDI/UART5 connection, set the FTDI receive latency timer
to **1 ms**. In the tested setup, 2 ms still allowed lag to accumulate; 1 ms
resolved the delay. This is a USB buffering setting, separate from baud rate
and `hitl.timeout_s`. Keep the firmware and simulator at matching 921600 baud
for the full 10 ms sensor/control exchange.

When the USB device is attached to WSL2 through `usbipd`, change the **Linux
driver setting inside WSL**, rather than relying on Windows Device Manager's
COM-port setting. These instructions assume USB forwarding already works.
See [Microsoft's WSL USB instructions](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
for attachment setup.

1. Stop the simulator, leaving the OBC connected and attached to WSL. No
   firmware rebuild or reflash is required.

2. From the simulator directory in WSL, identify the OBC's FTDI port:

   ```bash
   .venv/bin/python -m serial.tools.list_ports -v
   ```

   Select the FTDI adapter, not ST-LINK. The following commands assume
   `/dev/ttyUSB0`; replace `ttyUSB0` with the device used by `hitl.port`.

3. Check the driver and record the original latency value:

   ```bash
   ADCS_TTY=ttyUSB0
   readlink -f "/sys/bus/usb-serial/devices/$ADCS_TTY/driver"
   cat "/sys/bus/usb-serial/devices/$ADCS_TTY/latency_timer"
   ```

   The driver path should end in `ftdi_sio`. The timer value is in milliseconds.
   If the file does not exist, verify the port and driver before continuing;
   this setting does not apply to native USB CDC devices.

4. Set 1 ms and verify it:

   ```bash
   printf '1\n' | sudo tee "/sys/bus/usb-serial/devices/$ADCS_TTY/latency_timer"
   cat "/sys/bus/usb-serial/devices/$ADCS_TTY/latency_timer"
   ```

   The result should be `1`. If it was already `1`, no change is needed.
   The [Linux FTDI driver](https://github.com/torvalds/linux/blob/master/drivers/usb/serial/ftdi_sio.c)
   exposes this writable timer through sysfs.

5. Restart the same simulation, using your actual configuration filename:

   ```bash
   .venv/bin/python -m cubesat_sim config.blast.yaml
   ```

   Keep `hitl.log_usb_rx: false` when measuring timing. In a second WSL terminal,
   confirm that the timer remains at 1 ms while the serial port is open:

   ```bash
   cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
   ```

   Replace `ttyUSB0` there too if necessary. Check that the speed approaches
   `1.00x` and lag no longer grows steadily. This fixed the tested setup but
   does not guarantee that every host can meet the real-time deadlines.

Recheck after unplugging the OBC, reattaching it through `usbipd`, or restarting
WSL; do not assume the setting persists. Repeat step 4 if it has reverted.
To undo the change, write the original recorded value with the same `sudo tee`
command.

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

Reserve stdout for actuator replies, flush each reply immediately, and send diagnostics to stderr (inherited by the simulator). `sitl.timeout_s` is a positive wall-clock deadline for each write and lockstep actuator response after startup, or a stream freshness watchdog with autonomous exchange. A closed pipe, missing reply, or invalid reply stops the simulation. On completion or failure the simulator closes the process's stdin, waits briefly for exit, then terminates or kills it if needed and reaps it.

Existing shared-library configurations must replace `sitl.library` and the symbol settings with `sitl.command` and `sitl.timeout_s`. C and Rust controllers should now build as executables with a message loop; no shared library or C ABI is required.

## Independent timing

SITL and HITL display a terminal progress bar with completed steps, elapsed wall time, ETA, and current/total simulated seconds. Set `hitl.log_usb_rx: true` to print decoded USB/UART messages above the bar.
The logger assembles fragmented binary frames and emits one line per complete
message, using the familiar M/C/G/S/B/T/A type letters and scaled physical values.
For example:

```text
USB RX: A tick=100 m=[0.100000, 0.000000, -0.100000] A*m^2 boom=0.500 m E1 time=1.000 s valid=1 omega=[0.010000, -0.020000, 0.015000] rad/s
```

Times/periods are seconds, dipoles are A·m², boom targets are meters, and rates
are rad/s. The formatter also supports magnetic fields in µT, accelerations in
m/s², unitless Sun vectors, battery SOC in percent, and battery power in watts.
Tick numbers, sensor IDs, and validity flags remain integers. RX normally contains
A replies; enabling logging does not add TX logging. Invalid frames show an error
and bounded hexadecimal preview rather than misleading decoded values. Incomplete
frames wait for their delimiter; timeouts still report the partial-reply byte count.
Logging does not alter protocol validation or the binary wire format. Progress and USB output go to stderr; CSV telemetry is unchanged.

`step_s` is the Basilisk dynamics interval. `magnetometer_period_s`, `accelerometer_period_s`, `gyroscope_period_s`, `sun_period_s`, and `control_period_s` independently schedule the five ADCS calls; the current actuator command is held between control calls. `field_period_s` schedules WMM evaluations, and `telemetry_period_s` controls CSV size for long runs. All periods and `duration_s` must be integer multiples of `step_s`. Callbacks at the same time run in this order: magnetometer, accelerometer, gyroscope, Sun, battery status, control, then propagation. Battery status is sent at `control_period_s`, including time zero. `clock: simulated` runs without wall clock pacing; `clock: realtime` aligns steps with wall time. HITL requires real time.

For an OBC that is booting or restarted by the serial adapter, set
`hitl.startup_delay_s: 5.0` (or the measured boot time plus margin). HITL waits
that long after opening the port, then clears buffered startup output before
sending sensor/control packets. This is separate from `hitl.timeout_s`, the
stream freshness deadline; it is not a readiness handshake. The default delay is zero.

For YUAA development-board HITL, use `-DSIM_TARGET=HITL` and select
`-DSIM_CLOCK=REALTIME` for RTOS receipt timestamps, or `SIMULATED` for host
sensor timestamps. Use `hitl.exchange: autonomous` for REALTIME or `lockstep` for SIMULATED. Choose
`-DSIM_TRANSPORT=USB` for USB CDC or `UART` for COM1/USART3; match the UART
baud configured in `Core/Src/main.c` (currently 115200). USB CDC throughput is
not determined by the configured UART baud rate.

For the production OBC, select USB-C FTDI/UART5 and build with
`YUAA_ADCS_DEVELOPMENT=ON`, `YUAA_ADCS_SIM_REALTIME=ON`, and
`YUAA_ADCS_UART_BAUDRATE=921600`. Match `hitl.baudrate: 921600`.
The VS Code realtime build task includes these settings. Physical actuator
build switches remain independent. An autonomous timeout means no fresh synchronized U command arrived within `timeout_s`; confirm matching firmware/protocol versions, baud and port.

## SITL/HITL binary protocol

Both transports use [ADCS binary protocol v1](PROTOCOL.md): 3-bit message IDs,
quantized integer values, COBS framing and CRC16. Rebuild the C example and both
firmwares; ASCII replies are no longer accepted. Real-time HITL defaults to autonomous U output: firmware runs independently, and
the simulator applies the newest fresh command without a per-step reply wait.
T is sent once per second for timestamp alignment. Simulated-clock/native SITL
retains matching T/A lockstep.
The specification includes exact layouts, units, ranges and error handling.

For the updated OBC or YUAA real-time firmware:

```yaml
hitl:
  exchange: autonomous
  sync_period_s: 1.0
  timeout_s: 0.15  # maximum age/silence of fresh synchronized commands
```

Keep your existing port and baud settings. Reflash the updated firmware and update
this simulator together. U includes a sequence counter, RTOS timestamp and echoed
T anchor; the host rejects old-session and stale buffered output. Lost occasional
T packets do not stop control. Before the first synchronized command, output is
zero. The timeout remains a stream watchdog; it no longer imposes a round-trip
deadline on every control step. Corrupt frames are counted and skipped until a
valid delimiter/frame; a sustained loss of fresh output still stops the run.
For Renode with REALTIME firmware, set the same options under `sitl`; native
firmware and the C example require `sitl.exchange: lockstep` (the SITL default).


Sensor readings remain raw in each configured sensor frame. Firmware calibration
must use the inverse of the simulator's `body_to_sensor` rotation. Accelerometers
measure non-gravitational specific force, not orbital gravitational acceleration.
Estimated rate telemetry is included in every reply; invalid rates are zero.

Real-time runs use absolute monotonic deadlines and stop if lag exceeds
`simulation.max_realtime_lag_s` (default 0.25 seconds). Reducing bytes does not
synchronize independent clocks or guarantee deadlines. Retain 921600 baud for the
OBC's 10 ms full sensor exchange; it still exceeds the 115200-baud budget.

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

The binary A/U telemetry fields are specified in [PROTOCOL.md](PROTOCOL.md).
Every command contains an estimate validity flag; invalid rates are zero.
Autonomous U reception maps RTOS estimate epochs into simulation time using the
echoed synchronization anchor, including uint32 rollover. Lockstep simulated time
already shares the simulator epoch. Autonomous estimates are held with the command
between updates; the plotting tool scores each sensor epoch only once.

### Real-time throughput and zero outputs

Real-time pacing sleeps only when the loop is ahead; it does not skip physics steps or
invent missed sensor samples when behind. The progress bar reports simulation/wall-time
`speed` (1.00x is real time) and accumulated `lag`. At 115200 baud, 8N1 permits only
115.2 bytes per 10 ms in each direction. Two magnetometers, two accelerometers, a gyro,
battery and control request exceed that input budget at typical values. The reply and
USB/RTOS scheduling add further latency to the sequential exchange. For UART at 100 Hz, increase baud on both the board and simulator (for example,
921600 if supported), or use USB CDC. Leave RX logging disabled when measuring throughput.
This is additional transport headroom, not a guarantee that the host and USB path meet deadlines.

The OBC development estimator requires magnetic samples no more than 20 ms apart
and at least three observations. Slower exchanges can repeatedly reset the derivative
estimate and keep magnetic commands zero. Do not increase the control period to work
around throughput without revisiting estimator timing. Invalid rate telemetry
is expected during detumbling/deployment: the preserved algorithm computes rates only
in pointing mode. Zero rate validity alone therefore does not diagnose a sensor failure.
