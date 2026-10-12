from __future__ import annotations

import numpy as np
from datetime import datetime
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft, extForceTorque, magneticFieldWMM
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody, RigidBodyKinematics

from .config import _vector
from .earth import EarthOrientation
from .sun import sun_position_eci
from .power import PowerSystem, sunlight_fraction


def inertia_at_length(sc, length):
    bus = np.asarray(sc["bus_inertia_kg_m2"], dtype=float)
    mass, tip = float(sc["bus_mass_kg"]), float(sc["boom"]["tip_mass_kg"])
    axis = _vector(sc["boom"]["axis_body"], "boom axis", True)
    reduced_mass = mass * tip / (mass + tip)
    return bus + reduced_mass * length ** 2 * (np.eye(3) - np.outer(axis, axis))


class Plant:
    def __init__(self, config):
        self.config = config
        self.dt = float(config["simulation"]["step_s"])
        self._step_ns = macros.sec2nano(self.dt)
        self._time_ns = 0
        self.time_s = 0.0
        self.boom_length = float(config["spacecraft"]["boom"]["initial_length_m"])
        self.sim = SimulationBaseClass.SimBaseClass()
        process = self.sim.CreateNewProcess("physics")
        process.addTask(self.sim.CreateNewTask("physicsStep", self._step_ns))
        process.addTask(self.sim.CreateNewTask("fieldStep", macros.sec2nano(config["simulation"]["field_period_s"])))
        self.sc = spacecraft.Spacecraft()
        self.sc.ModelTag = "CubeSat"
        sc_cfg, orbit = config["spacecraft"], config["orbit"]
        self.sc.hub.mHub = float(sc_cfg["bus_mass_kg"] + sc_cfg["boom"]["tip_mass_kg"])
        self.sc.hub.IHubPntBc_B = inertia_at_length(sc_cfg, self.boom_length).tolist()
        self.sc.hub.r_CN_NInit = orbit["position_m"]
        self.sc.hub.v_CN_NInit = orbit["velocity_m_s"]
        self.sc.hub.sigma_BNInit = orbit["attitude_mrp"]
        self.sc.hub.omega_BN_BInit = orbit["angular_rate_rad_s"]
        gravity = simIncludeGravBody.gravBodyFactory()
        earth = gravity.createEarth()
        earth.mu = float(config["environment"]["earth_mu_m3_s2"])
        earth.isCentralBody = True
        self.sc.gravField.gravBodies = spacecraft.GravBodyVector(list(gravity.gravBodies.values()))
        self.effect = extForceTorque.ExtForceTorque()
        self.sc.addDynamicEffector(self.effect)
        self.sim.AddModelToTask("physicsStep", self.effect)
        self.sim.AddModelToTask("physicsStep", self.sc)
        self.wmm = magneticFieldWMM.MagneticFieldWMM()
        self.wmm.ModelTag = "WMM2025"
        self.wmm.configureWMMFile(config["environment"]["wmm_coefficients"])
        self.wmm.addSpacecraftToModel(self.sc.scStateOutMsg)
        epoch = datetime.fromisoformat(config["simulation"]["start_utc"])
        payload = messaging.EpochMsgPayload()
        payload.year, payload.month, payload.day = epoch.year, epoch.month, epoch.day
        payload.hours, payload.minutes, payload.seconds = epoch.hour, epoch.minute, epoch.second + epoch.microsecond / 1e6
        self.epoch_msg = messaging.EpochMsg().write(payload)
        self.wmm.epochInMsg.subscribeTo(self.epoch_msg)
        self.earth_orientation = EarthOrientation(config["simulation"]["start_utc"])
        self.wmm.planetPosInMsg.subscribeTo(self.earth_orientation.planet_state_out)
        # Explicit priorities ensure WMM sees the orientation at its own timestamp,
        # including the first evaluation at t=0.
        self.sim.AddModelToTask("fieldStep", self.earth_orientation, ModelPriority=1)
        self.sim.AddModelToTask("fieldStep", self.wmm, ModelPriority=0)
        self.sim.InitializeSimulation()
        self.sim.ConfigureStopTime(0)
        self.sim.ExecuteSimulation()
        self.power = PowerSystem(config)
        self.power.solar_power_W, self.power.sunlight_fraction = self.power.solar_power(self.state(), 0.0)

    def magnetic_field_n(self):
        return np.asarray(self.wmm.envOutMsgs[0].read().magField_N)

    def specific_force_body(self):
        """Non-gravitational acceleration at Basilisk's body reference point, m/s²."""
        return np.asarray(self.sc.scStateOutMsg.read().nonConservativeAccelpntB_B)

    def state(self):
        p = self.sc.scStateOutMsg.read()
        return {"position_m": np.asarray(p.r_BN_N), "velocity_m_s": np.asarray(p.v_BN_N),
                "attitude_mrp": np.asarray(p.sigma_BN), "angular_rate_rad_s": np.asarray(p.omega_BN_B)}

    def step(self, dipole, target_length):
        cfg = self.config
        boom = cfg["spacecraft"]["boom"]
        target = np.clip(float(target_length), boom["min_length_m"], boom["max_length_m"])
        delta = np.clip(target - self.boom_length, -boom["max_speed_m_s"] * self.dt, boom["max_speed_m_s"] * self.dt)
        previous_inertia = inertia_at_length(cfg["spacecraft"], self.boom_length)
        self.boom_length += float(delta)
        inertia = inertia_at_length(cfg["spacecraft"], self.boom_length)
        self.sc.hub.IHubPntBc_B = inertia.tolist()
        state = self.state()
        solar_start_W, _ = self.power.solar_power(state, self.time_s)
        dcm_bn = np.asarray(RigidBodyKinematics.MRP2C(state["attitude_mrp"]))
        b_body = dcm_bn @ self.magnetic_field_n()
        rhat_body = dcm_bn @ (state["position_m"] / np.linalg.norm(state["position_m"]))
        gg = 3 * float(cfg["environment"]["earth_mu_m3_s2"]) / np.linalg.norm(state["position_m"]) ** 3 * np.cross(rhat_body, inertia @ rhat_body)
        max_dipole = np.asarray(cfg["spacecraft"]["magnetorquer_max_dipole_Am2"], dtype=float)
        dipole = np.clip(np.asarray(dipole, dtype=float), -max_dipole, max_dipole)
        boom_reaction = -((inertia - previous_inertia) / self.dt) @ state["angular_rate_rad_s"]
        self.effect.extTorquePntB_B = (np.cross(dipole, b_body) + gg + boom_reaction).tolist()
        # Stop exactly on the task boundary. Accumulated float seconds can round
        # below it on long runs, skipping a step and later advancing twice.
        self._time_ns += self._step_ns
        self.time_s = self._time_ns / 1e9
        self.sim.ConfigureStopTime(self._time_ns)
        self.sim.ExecuteSimulation()
        solar_end_W, self.power.sunlight_fraction = self.power.solar_power(self.state(), self.time_s)
        self.power.advance(dipole, self.dt, solar_start_W, solar_end_W)
        return dipole


class Sensors:
    def __init__(self, config):
        self.config = config
        self.rng = np.random.default_rng(config["simulation"]["seed"])
        # Keep existing magnetometer/Sun noise sequences independent of this cadence.
        self.accel_rng = np.random.default_rng(np.random.SeedSequence(
            config["simulation"]["seed"], spawn_key=(1,)))

    def accelerometers(self, plant):
        force_body = plant.specific_force_body()
        readings = []
        for accel in self.config["sensors"]["accelerometers"]:
            reading = np.asarray(accel["body_to_sensor"]) @ force_body + np.asarray(accel["bias_m_s2"])
            reading += self.accel_rng.normal(0, accel["noise_std_m_s2"], 3)
            readings.append(reading.tolist())
        return readings

    def magnetometers(self, plant):
        state, c = plant.state(), self.config
        dcm_bn = np.asarray(RigidBodyKinematics.MRP2C(state["attitude_mrp"]))
        b_body = dcm_bn @ plant.magnetic_field_n()
        mags = []
        for mag in c["sensors"]["magnetometers"]:
            reading = np.asarray(mag["body_to_sensor"]) @ b_body + np.asarray(mag["bias_T"])
            reading += self.rng.normal(0, mag["noise_std_T"], 3)
            mags.append(reading.tolist())
        return mags

    def sun(self, plant):
        state, c = plant.state(), self.config
        dcm_bn = np.asarray(RigidBodyKinematics.MRP2C(state["attitude_mrp"]))
        r = state["position_m"]
        sun_position = sun_position_eci(c["simulation"]["start_utc"], plant.time_s)
        sun_r = sun_position - r
        sun_n = sun_r / np.linalg.norm(sun_r)
        radius = c["environment"]["earth_radius_m"]
        eclipse = sunlight_fraction(r, sun_position, radius, c["environment"]["sun_radius_m"]) == 0
        sun_b = dcm_bn @ sun_n
        sun_cfg = c["sensors"]["sun_sensor"]
        boresight = _vector(sun_cfg["boresight_body"], "boresight", True)
        visible = not eclipse and np.dot(sun_b, boresight) >= np.cos(np.deg2rad(sun_cfg["half_angle_deg"]))
        if visible:
            sun_b = sun_b + self.rng.normal(0, sun_cfg["noise_std_rad"], 3)
            sun_b = sun_b / np.linalg.norm(sun_b)
        return sun_b.tolist() if visible else None
