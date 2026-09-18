# CubeSat SITL/HITL simulator

Basilisk propagates the CubeSat orbit and attitude. The simulator calls a C/Rust ADCS library in SITL or exchanges compact ASCII lines with an STM32 over UART in HITL. One YAML file configures the spacecraft, sensors, environment, timing, and interface.

## Quick start

Use the existing virtual environment containing Basilisk, NumPy, and PyYAML. On macOS:

```sh
cc -dynamiclib -O2 examples/adcs_example.c -o libadcs.dylib
cp config.example.yaml config.yaml
# Set sitl.library to ./libadcs.dylib in config.yaml
.venv/bin/python -m cubesat_sim config.yaml
```

On Linux, build with `cc -shared -fPIC -O2 examples/adcs_example.c -o libadcs.so`; the template path already names that file. The example uses basic B-dot control and requests 0.5 m boom deployment. Replace it with your ADCS implementation. The output CSV is written beside the config file.

## Independent timing

`step_s` is the Basilisk dynamics interval. `magnetometer_period_s`, `sun_period_s`, and `control_period_s` independently schedule the three ADCS calls; the current actuator command is held between control calls. `field_period_s` schedules WMM evaluations, and `telemetry_period_s` controls CSV size for long runs. All periods and `duration_s` must be integer multiples of `step_s`. Callbacks at the same time run in this order: magnetometer, Sun, control, then propagation. `clock: simulated` runs without wall clock pacing; `clock: realtime` aligns steps with wall time. HITL requires real time.

For SITL, export these C ABI functions from a shared C or Rust library:

```c
void adcs_magnetometer(double time_s, const double mag1_T[3], const double mag2_T[3]);
void adcs_sun(double time_s, const double sun_body[3], int valid);
int adcs_step(double time_s, double control_period_s, double dipole_Am2_out[3], double *boom_length_m_out);
```

The functions may retain controller state between calls. `adcs_step` returns zero on success. The magnetometer vectors are in their configured sensor frames. The Sun vector is a unit vector in the body frame when `valid=1`; it is zero when `valid=0`. A Rust library can export the same ABI with `extern "C"` and `#[no_mangle]` (or `#[unsafe(no_mangle)]` in Rust 2024). Symbol names are configurable in `sitl`.

## HITL UART protocol

Set `simulation.mode: hitl`, `simulation.clock: realtime`, and `hitl.port`. The UART format is ASCII, 8N1, one line per message, with space separated decimal integers and `\n` termination. No JSON parser is needed on the MCU.

| Direction | Line | Units |
| --- | --- | --- |
| Simulator → MCU | `M time_ms m1x m1y m1z m2x m2y m2z` | magnetic readings, nT |
| Simulator → MCU | `S time_ms valid sx sy sz` | Sun body vector, signed Q15 (`32767` = 1) |
| Simulator → MCU | `T tick time_ms period_ms` | run one ADCS control step |
| MCU → Simulator | `A tick mx my mz boom` | dipoles, µA·m²; boom target, mm |

Example exchange:

```text
M 1000 10000 0 -10000 0 20000 0
S 1000 1 32767 0 0
T 5 1000 1000
A 5 100000 0 -100000 500
```

The `A` tick must match the preceding `T` tick. The MCU sends one `A` line after each `T`; no response is needed for `M` or `S`. `S` has zero vector components when `valid=0`. Missing, stale, or malformed actuator lines stop the simulation. `hitl.timeout_s` is the response deadline. Dipoles are clipped to the configured per-axis limits; boom targets are clipped to travel limits and rate limited. At 115200 baud, the sample `M` line is about 4 ms on the wire, so sensor periods can be set independently of a faster dynamics step.

## Environment and physical model

- The simulator evaluates Basilisk's [World Magnetic Model](https://hanspeterschaub.info/basilisk/Documentation/simulation/environment/magneticFieldWMM/magneticFieldWMM.html) from spacecraft position and simulation UTC. The included [NOAA WMM2025 coefficients](https://www.ncei.noaa.gov/products/world-magnetic-model/wmm-coefficients) cover 2025–2029. A custom coefficient file can be selected with `environment.wmm_coefficients`. Runs outside its five-year model interval are rejected.
- A date-dependent solar ephemeris computes Earth-to-Sun direction. Subtracting the spacecraft position gives the local Sun vector; cylindrical Earth eclipse and sensor field of view determine visibility. This is a low precision solar position model intended for attitude sensing across long detumbling runs.
- Magnetorquer torque is **m × B** using the WMM field in the body frame. Two magnetometers have independent rotation, bias, and Gaussian noise.
- The boom is a massless rod with a point tip mass. Extension changes Basilisk's spacecraft inertia and the gravity gradient torque. The inertia rate adds a reaction torque. `tip_mass_kg`, `axis_body`, initial and allowed lengths, and maximum extension speed are in the config. Flexible boom dynamics are not modeled.
- The initial orbit state uses Earth centered inertial coordinates. The template's orbit is near circular LEO. Central Earth gravity is used for propagation.

`simulation.seed` makes sensor noise repeatable. CSV telemetry includes position, angular rate, applied dipoles, boom length, the three diagonal moments of inertia, and the WMM field. The default six-hour simulated-clock run has 108,000 dynamics steps and writes telemetry once per second.

Run the focused tests with:

```sh
.venv/bin/python -m unittest discover -s tests -v
```
