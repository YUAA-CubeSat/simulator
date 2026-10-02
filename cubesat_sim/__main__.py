from __future__ import annotations

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
from .physics import Plant, Sensors


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


def run(config):
    plant = Plant(config)
    sensors = Sensors(config)
    sim = config["simulation"]
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
        controller = SitlController(config["sitl"]) if sim["mode"] == "sitl" else HitlController(
            config["hitl"], on_receive=lambda data: tqdm.write(f"USB RX: {data!r}", file=sys.stderr)
        )
        periods = {key: round(sim[key] / plant.dt) for key in ("magnetometer_period_s", "sun_period_s", "control_period_s", "telemetry_period_s")}
        steps = round(sim["duration_s"] / plant.dt)
        checkpoint_steps = sim.get("checkpoint_steps", 1000)
        applied = np.zeros(3)
        boom_target = plant.boom_length
        progress = tqdm(total=steps, desc=sim["mode"].upper(), unit="step", dynamic_ncols=True,
                        postfix=f"sim 0/{sim['duration_s']:g} s")
        start = time.monotonic()
        for tick in range(steps):
            t = plant.time_s
            if tick % periods["magnetometer_period_s"] == 0:
                controller.magnetometers(t, *sensors.magnetometers(plant))
            if tick % periods["sun_period_s"] == 0:
                controller.sun(t, sensors.sun(plant))
            if tick % periods["control_period_s"] == 0:
                controller.battery(t, plant.power.battery_soc, plant.power.battery_power_W)
                applied, boom_target = controller.step(tick, t, sim["control_period_s"])
            step_in_progress = True
            applied = plant.step(applied, boom_target)
            last_completed_time = plant.time_s
            last_applied = applied.copy()
            step_in_progress = False
            checkpoint = checkpoint_steps > 0 and (tick + 1) % checkpoint_steps == 0
            if file and ((tick + 1) % periods["telemetry_period_s"] == 0 or checkpoint):
                writer.writerow(telemetry_row(plant, applied))
                last_recorded_time = plant.time_s
            if file and checkpoint:
                file.flush()
                os.fsync(file.fileno())
            if sim["clock"] == "realtime":
                remaining = start + (tick + 1) * plant.dt - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
            progress.set_postfix_str(f"sim {plant.time_s:g}/{sim['duration_s']:g} s", refresh=False)
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
                if progress is not None:
                    progress.close()
            finally:
                if controller is not None:
                    controller.close()


def main():
    parser = argparse.ArgumentParser(description="Basilisk CubeSat SITL/HITL simulator")
    parser.add_argument("config", help="YAML configuration file")
    args = parser.parse_args()
    try:
        run(load_config(args.config))
    except KeyboardInterrupt:
        print("Simulation stopped (Ctrl+C).", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
