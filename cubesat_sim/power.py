"""Solar generation and battery energy accounting (SI power, Wh energy)."""

import numpy as np

from .sun import AU_M, sun_position_eci


def sunlight_fraction(position_m, sun_position_m, earth_radius_m, sun_radius_m):
    """Visible solar disk fraction using apparent angular disk overlap.

    Spherical Earth and a uniformly bright Sun; includes umbra and penumbra.
    See Basilisk's eclipse module documentation for the disk-overlap model.
    """
    r = np.asarray(position_m, dtype=float)
    to_sun = np.asarray(sun_position_m, dtype=float) - r
    distance = np.linalg.norm(to_sun)
    earth_distance = np.linalg.norm(r)
    if earth_distance <= earth_radius_m:
        return 0.0
    if earth_distance >= distance:
        return 1.0
    a = np.arcsin(np.clip(sun_radius_m / distance, 0, 1))
    b = np.arcsin(earth_radius_m / earth_distance)
    c = np.arccos(np.clip(np.dot(-r / earth_distance, to_sun / distance), -1, 1))
    if c >= a + b:
        return 1.0
    if c <= abs(b - a):
        return float(max(0.0, 1.0 - (b / a) ** 2))
    x = (c * c + a * a - b * b) / (2 * c)
    y = np.sqrt(max(0.0, a * a - x * x))
    overlap = a * a * np.arccos(np.clip(x / a, -1, 1))
    overlap += b * b * np.arccos(np.clip((c - x) / b, -1, 1)) - c * y
    return float(np.clip(1.0 - overlap / (np.pi * a * a), 0, 1))


class PowerSystem:
    def __init__(self, config):
        self.config = config
        sc = config["spacecraft"]
        self.capacity_Wh = sc["battery_capacity_Wh"]
        self.battery_energy_Wh = sc["battery_initial_soc"] * self.capacity_Wh
        self.solar_power_W = 0.0
        self.sunlight_fraction = 0.0
        self.magnetorquer_power_W = 0.0
        self.nominal_power_W = sc["nominal_power_W"]
        self.battery_power_W = 0.0
        self.unserved_power_W = 0.0
        self.unserved_energy_Wh = 0.0
        # Integrate at every physics step so sparse CSV output can still be
        # audited without aliasing the actuator duty cycle or charge clipping.
        self.solar_energy_Wh = 0.0
        self.magnetorquer_energy_Wh = 0.0
        self.nominal_energy_Wh = 0.0
        self.charging_loss_Wh = 0.0
        self.curtailed_energy_Wh = 0.0

    @property
    def battery_soc(self):
        return self.battery_energy_Wh / self.capacity_Wh

    def solar_power(self, state, time_s):
        """Available bus power after panel and power-converter losses."""
        from Basilisk.utilities import RigidBodyKinematics

        cfg = self.config
        solar = cfg["spacecraft"]["solar_charging"]
        r = state["position_m"]
        sun = sun_position_eci(cfg["simulation"]["start_utc"], time_s)
        to_sun = sun - r
        distance = np.linalg.norm(to_sun)
        fraction = sunlight_fraction(r, sun, cfg["environment"]["earth_radius_m"], cfg["environment"]["sun_radius_m"])
        sun_body = np.asarray(RigidBodyKinematics.MRP2C(state["attitude_mrp"])) @ (to_sun / distance)
        effective_area = sum(panel["area_m2"] * panel["efficiency"] *
                             max(0.0, np.dot(panel["normal_body"], sun_body))
                             for panel in solar["panels"])
        power = solar["irradiance_W_m2_at_1au"] * (AU_M / distance) ** 2
        return float(power * fraction * effective_area * solar["conversion_efficiency"]), fraction

    def advance(self, dipole, dt, solar_start_W, solar_end_W):
        """Integrate one step; powers are averages over this dynamics step.

        Solar power is trapezoidally averaged; the clipped dipole is held.
        Empty batteries record unmet load without changing actuator dynamics.
        """
        sc = self.config["spacecraft"]
        maximum = np.asarray(sc["magnetorquer_max_dipole_Am2"])
        strength = np.abs(np.clip(dipole, -maximum, maximum))
        fraction = np.divide(strength, maximum, out=np.zeros(3), where=maximum > 0)
        self.magnetorquer_power_W = float(np.dot(fraction, sc["magnetorquer_max_power_W"]))
        self.solar_power_W = 0.5 * (solar_start_W + solar_end_W)
        surplus = self.solar_power_W - self.magnetorquer_power_W - self.nominal_power_W
        solar = sc["solar_charging"]
        stored_power = min(surplus, solar["max_charge_power_W"]) * solar["battery_charge_efficiency"] if surplus > 0 else surplus
        before = self.battery_energy_Wh
        proposed = before + stored_power * dt / 3600
        self.battery_energy_Wh = float(np.clip(proposed, 0, self.capacity_Wh))
        self.battery_power_W = (self.battery_energy_Wh - before) * 3600 / dt
        unserved = max(0.0, -proposed)
        self.unserved_power_W = unserved * 3600 / dt
        self.unserved_energy_Wh += unserved
        self.solar_energy_Wh += self.solar_power_W * dt / 3600
        self.magnetorquer_energy_Wh += self.magnetorquer_power_W * dt / 3600
        self.nominal_energy_Wh += self.nominal_power_W * dt / 3600
        if surplus > 0:
            charged = self.battery_energy_Wh - before
            charger_input = charged / solar["battery_charge_efficiency"]
            self.charging_loss_Wh += charger_input - charged
            self.curtailed_energy_Wh += max(0.0, surplus * dt / 3600 - charger_input)
