from __future__ import annotations

from pathlib import Path
from datetime import datetime, timedelta, timezone
import math
import numpy as np
import yaml


def _vector(value, name, unit=False):
    a = np.asarray(value, dtype=float)
    if a.shape != (3,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be a finite 3-vector")
    if unit:
        norm = np.linalg.norm(a)
        if norm == 0:
            raise ValueError(f"{name} must be nonzero")
        a = a / norm
    return a


def load_config(path):
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as f:
        c = yaml.safe_load(f)
    if not isinstance(c, dict):
        raise ValueError("Configuration must be a YAML mapping")
    s, o, sc, env, sensors = (c[k] for k in ("simulation", "orbit", "spacecraft", "environment", "sensors"))
    for key in ("duration_s", "step_s", "magnetometer_period_s", "sun_period_s", "control_period_s", "field_period_s", "telemetry_period_s"):
        s[key] = float(s[key])
    start = datetime.fromisoformat(s["start_utc"].replace("Z", "+00:00"))
    if start.tzinfo is None:
        raise ValueError("simulation.start_utc must include a UTC offset")
    s["start_utc"] = start.astimezone(timezone.utc).isoformat()
    s["seed"] = int(s["seed"])
    sc["bus_mass_kg"] = float(sc["bus_mass_kg"])
    for key in ("tip_mass_kg", "initial_length_m", "min_length_m", "max_length_m", "max_speed_m_s"):
        sc["boom"][key] = float(sc["boom"][key])
    for key in ("earth_radius_m", "earth_mu_m3_s2"):
        env[key] = float(env[key])
    for mag in sensors["magnetometers"]:
        mag["noise_std_T"] = float(mag["noise_std_T"])
    for key in ("half_angle_deg", "noise_std_rad"):
        sensors["sun_sensor"][key] = float(sensors["sun_sensor"][key])
    if "hitl" in c:
        c["hitl"]["baudrate"] = int(c["hitl"]["baudrate"])
        c["hitl"]["timeout_s"] = float(c["hitl"]["timeout_s"])
    if s["mode"] not in ("sitl", "hitl") or s["clock"] not in ("simulated", "realtime"):
        raise ValueError("simulation.mode or clock is invalid")
    if s["mode"] == "hitl" and s["clock"] != "realtime":
        raise ValueError("HITL requires a realtime clock")
    for key in ("duration_s", "step_s", "magnetometer_period_s", "sun_period_s", "control_period_s", "field_period_s", "telemetry_period_s"):
        if not math.isfinite(float(s[key])) or float(s[key]) <= 0:
            raise ValueError(f"simulation.{key} must be positive")
    if s["step_s"] > s["duration_s"]:
        raise ValueError("step_s cannot exceed duration_s")
    for key in ("magnetometer_period_s", "sun_period_s", "control_period_s", "field_period_s", "telemetry_period_s", "duration_s"):
        ratio = s[key] / s["step_s"]
        if ratio < 1 or abs(ratio - round(ratio)) > 1e-8:
            raise ValueError(f"simulation.{key} must be a positive integer multiple of step_s")
    if not math.isfinite(sc["bus_mass_kg"]) or sc["bus_mass_kg"] <= 0:
        raise ValueError("bus_mass_kg must be positive")
    inertia = np.asarray(sc["bus_inertia_kg_m2"], dtype=float)
    if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)) or not np.allclose(inertia, inertia.T) or np.min(np.linalg.eigvalsh(inertia)) <= 0:
        raise ValueError("bus inertia must be a symmetric positive definite 3x3 matrix")
    sc["bus_inertia_kg_m2"] = inertia.tolist()
    for key in ("position_m", "velocity_m_s", "attitude_mrp", "angular_rate_rad_s"):
        o[key] = _vector(o[key], f"orbit.{key}").tolist()
    if np.linalg.norm(o["position_m"]) <= float(env["earth_radius_m"]):
        raise ValueError("initial position must be above Earth")
    for key in ("magnetorquer_max_dipole_Am2",):
        sc[key] = _vector(sc[key], key).tolist()
        if np.any(np.asarray(sc[key]) < 0):
            raise ValueError(f"{key} must be nonnegative")
    boom = sc["boom"]
    boom["axis_body"] = _vector(boom["axis_body"], "boom.axis_body", True).tolist()
    if not (0 <= boom["min_length_m"] <= boom["initial_length_m"] <= boom["max_length_m"]):
        raise ValueError("boom length limits are invalid")
    if any(not math.isfinite(boom[k]) for k in ("tip_mass_kg", "initial_length_m", "min_length_m", "max_length_m", "max_speed_m_s")) or boom["tip_mass_kg"] < 0 or boom["max_speed_m_s"] <= 0:
        raise ValueError("boom mass or speed is invalid")
    if any(not math.isfinite(env[k]) or env[k] <= 0 for k in ("earth_radius_m", "earth_mu_m3_s2")):
        raise ValueError("environment values must be positive")
    env["wmm_coefficients"] = str((path.parent / env["wmm_coefficients"]).resolve())
    if not Path(env["wmm_coefficients"]).is_file():
        raise FileNotFoundError(f"WMM coefficients missing: {env['wmm_coefficients']}")
    with open(env["wmm_coefficients"], encoding="ascii") as coefficients:
        model_year = int(float(coefficients.readline().split()[0]))
    model_start = datetime(model_year, 1, 1, tzinfo=timezone.utc)
    model_end = datetime(model_year + 5, 1, 1, tzinfo=timezone.utc)
    if not model_start <= start or start + timedelta(seconds=s["duration_s"]) >= model_end:
        raise ValueError(f"WMM coefficients are valid from {model_year} through {model_year + 4}")
    mags = sensors["magnetometers"]
    if len(mags) != 2:
        raise ValueError("exactly two magnetometers are required")
    for i, mag in enumerate(mags):
        dcm = np.asarray(mag["body_to_sensor"], dtype=float)
        if dcm.shape != (3, 3) or not np.allclose(dcm @ dcm.T, np.eye(3), atol=1e-5) or np.linalg.det(dcm) < 0.999:
            raise ValueError(f"magnetometer {i} body_to_sensor must be a proper rotation")
        mag["bias_T"] = _vector(mag["bias_T"], f"magnetometer {i} bias_T").tolist()
        mag["body_to_sensor"] = dcm.tolist()
        if not math.isfinite(mag["noise_std_T"]) or mag["noise_std_T"] < 0:
            raise ValueError("magnetometer noise must be nonnegative")
    sun = sensors["sun_sensor"]
    sun["boresight_body"] = _vector(sun["boresight_body"], "sun_sensor.boresight_body", True).tolist()
    if not 0 < sun["half_angle_deg"] <= 180 or not math.isfinite(sun["noise_std_rad"]) or sun["noise_std_rad"] < 0:
        raise ValueError("sun sensor angle or noise is invalid")
    if s["mode"] == "sitl":
        sitl = c["sitl"]
        command = sitl.get("command")
        if not isinstance(command, list) or not command or any(not isinstance(arg, str) or not arg for arg in command):
            raise ValueError("sitl.command must be a nonempty list of strings")
        sitl["timeout_s"] = float(sitl["timeout_s"])
        if not math.isfinite(sitl["timeout_s"]) or sitl["timeout_s"] <= 0:
            raise ValueError("SITL timeout_s must be positive and finite")
        sitl["cwd"] = str(path.parent)
    if s.get("output_csv"):
        s["output_csv"] = str((path.parent / s["output_csv"]).resolve())
    return c
