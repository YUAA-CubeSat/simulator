from __future__ import annotations

# Allow direct execution from any working directory as well as package execution.
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "cubesat_sim"

import argparse
import csv
import os
import sys
import time
from pathlib import Path

import numpy as np
from tqdm import tqdm

from .config import load_config
from .io import HitlController, SitlController
from .estimator_log import EstimatorLog
from .pacing import pace
from .protocol_log import ProtocolRxLogger


def telemetry_row(plant, applied):
    """Snapshot a completed dynamics step for CSV output."""
    state = plant.state()
    inertia = np.asarray(plant.sc.scMassOutMsg.read().ISC_PntB_B)
    power = plant.power
    return [plant.time_s, *state["position_m"], *state["angular_rate_rad_s"], plant.boom_length,
            inertia[0, 0], inertia[1, 1], inertia[2, 2], *applied, *plant.magnetic_field_n(), *state["attitude_mrp"],
            power.battery_energy_Wh, power.battery_soc, power.solar_power_W, power.magnetorquer_power_W,
            power.battery_power_W, power.sunlight_fraction, power.unserved_power_W, power.unserved_energy_Wh,
            power.solar_energy_Wh, power.magnetorquer_energy_Wh, power.charging_loss_Wh, power.curtailed_energy_Wh,
            power.nominal_power_W, power.nominal_energy_Wh]


def run(config, on_output_ready=None):
    if config["simulation"]["mode"] == "orbit_solar":
        from .orbit_solar import run as run_orbit_solar
        return run_orbit_solar(config, on_output_ready=on_output_ready)
    from .physics import Plant, Sensors

    plant = Plant(config)
    sensors = Sensors(config)
    sim = config["simulation"]
    estimator_log = None
    file = None
    progress = None
    controller = None
    interrupted = False
    step_in_progress = False
    last_completed_time = 0.0
    last_recorded_time = 0.0
    last_applied = np.zeros(3)
    try:
        output = sim.get("output_csv")
        if output:
            Path(output).parent.mkdir(parents=True, exist_ok=True)
            file = open(output, "w", newline="", encoding="utf-8")
            writer = csv.writer(file)
            writer.writerow(["time_s", "x_m", "y_m", "z_m", "wx_rad_s", "wy_rad_s", "wz_rad_s", "boom_length_m", "Ixx_kg_m2", "Iyy_kg_m2", "Izz_kg_m2", "mx_Am2", "my_Am2", "mz_Am2", "Bx_T", "By_T", "Bz_T", "sigma_BN_x", "sigma_BN_y", "sigma_BN_z",
                             "battery_energy_Wh", "battery_soc", "solar_power_W", "magnetorquer_power_W", "battery_power_W", "sunlight_fraction", "unserved_power_W", "unserved_energy_Wh",
                             "solar_energy_Wh", "magnetorquer_energy_Wh", "charging_loss_Wh", "curtailed_energy_Wh",
                             "nominal_power_W", "nominal_energy_Wh"])
        if output:
            estimator_log = EstimatorLog(output)
        if on_output_ready is not None:
            if file:
                file.flush()
                estimator_log.file.flush()
            on_output_ready()
        controller = SitlController(config["sitl"]) if sim["mode"] == "sitl" else HitlController(
            config["hitl"], on_receive=ProtocolRxLogger(lambda line: tqdm.write(line, file=sys.stderr))
            if config["hitl"].get("log_usb_rx", False) else None
        )
        periods = {key: round(sim[key] / plant.dt) for key in ("magnetometer_period_s", "accelerometer_period_s", "gyroscope_period_s", "sun_period_s", "control_period_s", "telemetry_period_s")}
        steps = round(sim["duration_s"] / plant.dt)
        checkpoint_steps = sim.get("checkpoint_steps", 1000)
        applied = np.zeros(3)
        boom_target = plant.boom_length
        progress = tqdm(total=steps, desc=sim["mode"].upper(), unit="step", dynamic_ncols=True,
                        postfix=f"sim 0/{sim['duration_s']:g} s")
        start = time.monotonic()
        last_live_flush = start
        for tick in range(steps):
            t = plant.time_s
            if tick % periods["magnetometer_period_s"] == 0:
                controller.magnetometers(t, *sensors.magnetometers(plant))
            if tick % periods["accelerometer_period_s"] == 0:
                controller.accelerometers(t, *sensors.accelerometers(plant))
            if tick % periods["gyroscope_period_s"] == 0:
                controller.gyroscopes(t, *sensors.gyroscopes(plant))
            if tick % periods["sun_period_s"] == 0:
                controller.sun(t, sensors.sun(plant))
            estimate = None
            if tick % periods["control_period_s"] == 0:
                controller.battery(t, plant.power.battery_soc, plant.power.battery_power_W)
                applied, boom_target = controller.step(tick, t, sim["control_period_s"])
                estimate = getattr(controller, "estimate", None)
            if estimator_log:
                estimator_log.record(plant, tick, estimate)
            step_in_progress = True
            applied = plant.step(applied, boom_target)
            last_completed_time = plant.time_s
            last_applied = applied.copy()
            step_in_progress = False
            checkpoint = checkpoint_steps > 0 and (tick + 1) % checkpoint_steps == 0
            if file and ((tick + 1) % periods["telemetry_period_s"] == 0 or checkpoint):
                writer.writerow(telemetry_row(plant, applied))
                last_recorded_time = plant.time_s
            now = time.monotonic()
            if file and now - last_live_flush >= 0.5:
                file.flush()
                if estimator_log:
                    estimator_log.file.flush()
                last_live_flush = now
            if file and checkpoint:
                estimator_log.flush()
                file.flush()
                os.fsync(file.fileno())
            if sim["clock"] == "realtime":
                pace(start, plant.time_s, sim.get("max_realtime_lag_s", 0.25))
            elapsed = max(time.monotonic() - start, 1e-9)
            progress.set_postfix_str(
                f"sim {plant.time_s:g}/{sim['duration_s']:g} s, "
                f"speed {plant.time_s / elapsed:.2f}x, lag {max(0.0, elapsed - plant.time_s):.2f}s",
                refresh=False)
            progress.update(1)
        return plant
    except KeyboardInterrupt:
        interrupted = True
        # Never serialize a partially advanced plant, nor a new controller
        # command that has not yet been applied to the dynamics.
        if file and not step_in_progress and last_completed_time > last_recorded_time:
            writer.writerow(telemetry_row(plant, last_applied))
        raise
    finally:
        # Save telemetry before touching the subprocess/UART or progress bar:
        # their cleanup can itself fail or receive another interrupt.
        try:
            if file:
                try:
                    file.flush()
                    os.fsync(file.fileno())
                finally:
                    file.close()
                if interrupted:
                    tqdm.write(f"Telemetry saved to {output}", file=sys.stderr)
        finally:
            try:
                if estimator_log is not None:
                    estimator_log.close()
            finally:
                try:
                    if progress is not None:
                        progress.close()
                finally:
                    if controller is not None:
                        controller.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="CubeSat SITL/HITL and orbit-only solar simulator")
    parser.add_argument("config", help="YAML configuration file")
    parser.add_argument("--live", action="store_true", help="Open the live viewer and wait for its Start simulation button")
    parser.add_argument("--live-port", type=int, default=8765, help="Live viewer localhost port (0 chooses a free port)")
    parser.add_argument("--live-max-rows", type=int, default=20000, help="Retained telemetry rows per stream")
    parser.add_argument("--no-browser", action="store_true", help="Print the viewer URL without opening a browser")
    args = parser.parse_args(argv)
    if not 0 <= args.live_port <= 65535 or args.live_max_rows < 3:
        parser.error("--live-port must be 0–65535 and --live-max-rows must be >= 3")
    try:
        config = load_config(args.config)
        if args.live:
            if not config["simulation"].get("output_csv"):
                parser.error("--live requires simulation.output_csv in the configuration")
            from utilities.live_dashboard import run_simulator_viewer
            return run_simulator_viewer(config, Path(args.config).resolve(), run,
                                        port=args.live_port, max_rows=args.live_max_rows,
                                        open_browser=not args.no_browser)
        run(config)
    except KeyboardInterrupt:
        print("Simulation stopped (Ctrl+C).", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
