"""Attitude-free elliptic two-body propagation and available solar bus energy."""

import csv
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
from tqdm import tqdm

from .config import _vector
from .power import sunlight_fraction
from .sun import AU_M, sun_position_eci


def validate_config(config, path):
    """Validate only inputs used by this mode; ADCS and battery are irrelevant."""
    sim, orbit, env, solar = (config[k] for k in ("simulation", "orbit", "environment", "solar"))
    sim.setdefault("clock", "simulated")
    if sim["clock"] not in ("simulated", "realtime"):
        raise ValueError("simulation.clock is invalid")
    start = datetime.fromisoformat(sim["start_utc"].replace("Z", "+00:00"))
    if start.tzinfo is None:
        raise ValueError("simulation.start_utc must include a UTC offset")
    sim["start_utc"] = start.astimezone(timezone.utc).isoformat()
    for section, keys in ((sim, ("duration_s", "step_s", "telemetry_period_s")),
                          (env, ("earth_radius_m", "earth_mu_m3_s2", "sun_radius_m")),
                          (solar, ("irradiance_W_m2_at_1au", "efficiency", "conversion_efficiency"))):
        for key in keys:
            section[key] = float(section[key])
            if not math.isfinite(section[key]) or section[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
    solar["effective_area_m2"] = float(solar["effective_area_m2"])
    if not math.isfinite(solar["effective_area_m2"]) or solar["effective_area_m2"] < 0:
        raise ValueError("effective_area_m2 must be nonnegative and finite")
    if solar["efficiency"] > 1 or solar["conversion_efficiency"] > 1:
        raise ValueError("solar efficiencies must be at most 1")
    for key in ("duration_s", "telemetry_period_s"):
        ratio = sim[key] / sim["step_s"]
        if ratio < 1 or abs(ratio - round(ratio)) > 1e-8:
            raise ValueError(f"simulation.{key} must be a positive integer multiple of step_s")
    sim.setdefault("checkpoint_steps", 1000)
    if type(sim["checkpoint_steps"]) is not int or sim["checkpoint_steps"] < 0:
        raise ValueError("simulation.checkpoint_steps must be a nonnegative integer")
    for key in ("position_m", "velocity_m_s"):
        orbit[key] = _vector(orbit[key], f"orbit.{key}").tolist()
    KeplerOrbit(config)  # Reject unbound, radial, or Earth-intersecting trajectories.
    if sim.get("output_csv"):
        sim["output_csv"] = str((path.parent / sim["output_csv"]).resolve())
    return config


class KeplerOrbit:
    """Exact Keplerian ellipse in J2000, with no rotational state or dynamics."""

    def __init__(self, config):
        r = np.asarray(config["orbit"]["position_m"], dtype=float)
        v = np.asarray(config["orbit"]["velocity_m_s"], dtype=float)
        mu = config["environment"]["earth_mu_m3_s2"]
        radius = np.linalg.norm(r)
        h = np.cross(r, v)
        if radius <= config["environment"]["earth_radius_m"] or np.linalg.norm(h) == 0:
            raise ValueError("orbit must be nonradial and above Earth")
        energy = np.dot(v, v) / 2 - mu / radius
        if energy >= 0:
            raise ValueError("orbit_solar requires a bound elliptic orbit")
        self.a = -mu / (2 * energy)
        eccentricity = np.cross(v, h) / mu - r / radius
        self.e = float(np.linalg.norm(eccentricity))
        if self.e >= 1 or self.a * (1 - self.e) <= config["environment"]["earth_radius_m"]:
            raise ValueError("orbit must not intersect Earth")
        self.p = eccentricity / self.e if self.e > 1e-12 else r / radius
        self.q = np.cross(h / np.linalg.norm(h), self.p)
        self.beta = math.sqrt(1 - self.e ** 2)
        anomaly = math.atan2(np.dot(r, self.q) / self.beta, np.dot(r, self.p) + self.a * self.e)
        self.mean0 = anomaly - self.e * math.sin(anomaly)
        self.n = math.sqrt(mu / self.a ** 3)
        self.period_s = 2 * math.pi / self.n

    def state(self, elapsed_s):
        mean = (self.mean0 + self.n * elapsed_s) % (2 * math.pi)
        low, high = 0.0, 2 * math.pi
        for _ in range(52):
            anomaly = (low + high) / 2
            if anomaly - self.e * math.sin(anomaly) < mean:
                low = anomaly
            else:
                high = anomaly
        c, s = math.cos(anomaly), math.sin(anomaly)
        position = self.a * ((c - self.e) * self.p + self.beta * s * self.q)
        velocity = self.a * self.n / (1 - self.e * c) * (-s * self.p + self.beta * c * self.q)
        return position, velocity


def solar_sample(config, position, elapsed_s):
    sun = sun_position_eci(config["simulation"]["start_utc"], elapsed_s)
    env, solar = config["environment"], config["solar"]
    fraction = sunlight_fraction(position, sun, env["earth_radius_m"], env["sun_radius_m"])
    irradiance = solar["irradiance_W_m2_at_1au"] * (AU_M / np.linalg.norm(sun - position)) ** 2
    power = irradiance * fraction * solar["effective_area_m2"] * solar["efficiency"] * solar["conversion_efficiency"]
    return float(power), fraction


COLUMNS = ("time_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s",
           "solar_power_W", "sunlight_fraction", "solar_energy_Wh", "orbit_period_s")


def run(config, on_output_ready=None):
    sim = config["simulation"]
    orbit = KeplerOrbit(config)
    dt = sim["step_s"]
    steps = round(sim["duration_s"] / dt)
    record_every = round(sim["telemetry_period_s"] / dt)
    checkpoint_every = sim.get("checkpoint_steps", 1000)
    output = sim.get("output_csv")
    file = progress = None
    row = None
    recorded_time = None
    try:
        if output:
            Path(output).parent.mkdir(parents=True, exist_ok=True)
            file = open(output, "w", newline="", encoding="utf-8")
            writer = csv.writer(file)
            writer.writerow(COLUMNS)
        if on_output_ready is not None:
            if file:
                file.flush()
            on_output_ready()
        r, v = orbit.state(0)
        power, fraction = solar_sample(config, r, 0)
        energy = 0.0
        row = [0.0, *r, *v, power, fraction, energy, orbit.period_s]
        if file:
            writer.writerow(row)
            recorded_time = 0.0
        progress = tqdm(total=steps, desc="ORBIT_SOLAR", unit="step", dynamic_ncols=True)
        start = time.monotonic()
        last_live_flush = start
        for tick in range(1, steps + 1):
            elapsed = tick * dt
            r, v = orbit.state(elapsed)
            next_power, next_fraction = solar_sample(config, r, elapsed)
            energy += (power + next_power) * 0.5 * dt / 3600
            # Preserve both sides of eclipse transitions even with sparse output.
            transition = (fraction > 0) != (next_fraction > 0)
            if file and transition and recorded_time != row[0]:
                writer.writerow(row)
                recorded_time = row[0]
            next_row = [elapsed, *r, *v, next_power, next_fraction, energy, orbit.period_s]
            row = next_row  # Only publish a fully completed sample to interrupt cleanup.
            power, fraction = next_power, next_fraction
            checkpoint = checkpoint_every > 0 and tick % checkpoint_every == 0
            if file and (tick % record_every == 0 or transition or checkpoint or tick == steps):
                writer.writerow(row)
                recorded_time = elapsed
            now = time.monotonic()
            if file and now - last_live_flush >= 0.5:
                file.flush()
                last_live_flush = now
            if file and checkpoint:
                file.flush()
                os.fsync(file.fileno())
            progress.update(1)
            if sim["clock"] == "realtime":
                remaining = start + elapsed - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
        return dict(zip(COLUMNS, row))
    except KeyboardInterrupt:
        if file:
            tqdm.write(f"Telemetry saved to {output}", file=sys.stderr)
        raise
    finally:
        try:
            if file:
                try:
                    if row is not None and recorded_time != row[0]:
                        writer.writerow(row)
                    file.flush()
                    os.fsync(file.fileno())
                finally:
                    file.close()
        finally:
            if progress is not None:
                progress.close()
