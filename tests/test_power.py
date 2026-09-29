import copy
import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from cubesat_sim.config import load_config
from cubesat_sim.physics import Plant
from cubesat_sim.power import PowerSystem, sunlight_fraction
from cubesat_sim.sun import AU_M
from cubesat_sim.__main__ import run

ROOT = Path(__file__).resolve().parents[1]


class PowerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / "config.example.yaml")
        self.config["spacecraft"]["nominal_power_W"] = 0
        self.power = PowerSystem(self.config)

    def test_initial_energy_and_linear_clipped_axis_load(self):
        self.assertAlmostEqual(self.power.battery_soc, 0.8)
        self.config["spacecraft"]["magnetorquer_max_dipole_Am2"] = [0.2, 0.2, 0]
        self.config["spacecraft"]["magnetorquer_max_power_W"] = [2, 4, 9]
        self.power.advance([0.1, -10, 10], 3600, 0, 0)
        self.assertEqual(self.power.magnetorquer_power_W, 5)
        self.assertAlmostEqual(self.power.battery_energy_Wh, 3)
        self.assertAlmostEqual(self.power.battery_power_W, -5)

    def test_configurable_initial_charge_including_empty_and_full(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            for soc in (0, 0.35, 1):
                with self.subTest(soc=soc):
                    self.config["spacecraft"]["battery_initial_soc"] = soc
                    self.config["spacecraft"]["battery_capacity_Wh"] = 12
                    path.write_text(yaml.safe_dump(self.config))
                    plant = Plant(load_config(path))
                    self.assertAlmostEqual(plant.power.battery_soc, soc)
                    self.assertAlmostEqual(plant.power.battery_energy_Wh, 12 * soc)
                    self.assertEqual(plant.power.battery_power_W, 0)
                    self.assertEqual(plant.power.unserved_energy_Wh, 0)

    def test_charge_efficiency_limit_capacity_and_unserved_energy(self):
        solar = self.config["spacecraft"]["solar_charging"]
        solar.update(max_charge_power_W=2, battery_charge_efficiency=0.5)
        self.power.advance([0.2, 0, 0], 3600, 3, 7)
        self.assertEqual(self.power.solar_power_W, 5)
        self.assertAlmostEqual(self.power.battery_energy_Wh, 9)
        self.power.advance([0, 0, 0], 7200, 5, 5)
        self.assertEqual(self.power.battery_energy_Wh, 10)
        self.assertAlmostEqual(self.power.battery_power_W, 0.5)
        self.power.advance([0.2, 0.2, 0.2], 36000, 0, 0)
        self.assertEqual(self.power.battery_energy_Wh, 0)
        self.assertAlmostEqual(self.power.unserved_energy_Wh, 5)
        self.assertAlmostEqual(self.power.unserved_power_W, 0.5)
        self.power.advance([0, 0, 0], 3600, 1, 1)
        self.assertEqual(self.power.battery_energy_Wh, 0.5)
        self.assertEqual(self.power.unserved_power_W, 0)

    def test_surplus_only_charges_and_solar_offsets_load(self):
        self.power.advance([0.2, 0.2, 0], 3600, 0.5, 0.5)
        self.assertAlmostEqual(self.power.battery_energy_Wh, 7.5)
        self.power.advance([0.2, 0.2, 0], 3600, 2, 2)
        self.assertAlmostEqual(self.power.battery_energy_Wh, 8.45)

    def test_nominal_load_with_idle_actuators_solar_and_depleted_battery(self):
        self.config["spacecraft"]["nominal_power_W"] = 1.0
        power = PowerSystem(self.config)
        power.advance([0, 0, 0], 3600, 0, 0)
        self.assertEqual(power.battery_energy_Wh, 7)
        self.assertEqual(power.nominal_energy_Wh, 1)
        # Solar first supplies both the nominal and magnetorquer loads.
        power.advance([0.2, 0, 0], 3600, 0.75, 0.75)
        self.assertEqual(power.battery_energy_Wh, 6.25)
        power.advance([0.2, 0, 0], 3600, 2, 2)
        self.assertAlmostEqual(power.battery_energy_Wh, 6.725)
        power.advance([0, 0, 0], 36000, 0, 0)
        self.assertEqual(power.battery_energy_Wh, 0)
        self.assertAlmostEqual(power.unserved_energy_Wh, 3.275)
        self.assertEqual(power.nominal_energy_Wh, 13)

    def test_nominal_load_independent_of_orbit_attitude_and_actuator_activity(self):
        self.config["spacecraft"]["nominal_power_W"] = 1.0
        self.config["spacecraft"]["solar_charging"]["panels"] = []
        for position, attitude, dipole, expected_load in (
                ([7e6, 0, 0], [0, 0, 0], [0, 0, 0], 1),
                ([-7e6, 0, 0], [0.2, 0.1, -0.3], [0.2, 0, 0], 1.5)):
            self.config["orbit"].update(position_m=position, attitude_mrp=attitude)
            plant = Plant(copy.deepcopy(self.config))
            plant.step(dipole, 0)
            self.assertEqual(plant.power.nominal_power_W, 1)
            self.assertAlmostEqual(plant.power.nominal_energy_Wh, plant.dt / 3600)
            self.assertAlmostEqual(plant.power.battery_power_W, -expected_load)

    def test_panel_projection_distance_efficiency_and_attitude(self):
        solar = self.config["spacecraft"]["solar_charging"]
        solar["panels"] = [{"normal_body": [1, 0, 0], "area_m2": 0.02, "efficiency": 0.3}]
        state = {"position_m": np.array([7e6, 0, 0]), "attitude_mrp": [0, 0, 0]}
        expected = 1361 * 0.02 * 0.3 * 0.95
        for angle, distance in ((0, AU_M), (np.pi / 3, AU_M), (0, 2 * AU_M), (np.pi, AU_M)):
            with self.subTest(angle=angle, distance=distance):
                state["attitude_mrp"] = [0, 0, np.tan(angle / 4)]
                with patch("cubesat_sim.power.sun_position_eci", return_value=state["position_m"] + [distance, 0, 0]):
                    power, fraction = self.power.solar_power(state, 0)
                self.assertEqual(fraction, 1)
                self.assertAlmostEqual(power, expected * max(0, np.cos(angle)) * (AU_M / distance) ** 2)
        solar["panels"] = []
        self.assertEqual(self.power.solar_power(state, 0)[0], 0)

    def test_eclipse_full_partial_annular_and_contacts(self):
        earth, sun_radius, distance = 6378137, 6.957e8, 7e6
        r = np.array([distance, 0, 0])
        self.assertEqual(sunlight_fraction(r, [AU_M, 0, 0], earth, sun_radius), 1)
        self.assertEqual(sunlight_fraction(-r, [AU_M, 0, 0], earth, sun_radius), 0)
        a, b = np.arcsin(sun_radius / AU_M), np.arcsin(earth / distance)
        fractions = []
        for separation in np.linspace(b - a - 1e-8, b + a + 1e-8, 101):
            sun = r + AU_M * np.array([-np.cos(separation), np.sin(separation), 0])
            fractions.append(sunlight_fraction(r, sun, earth, sun_radius))
        self.assertAlmostEqual(fractions[0], 0)
        self.assertAlmostEqual(fractions[-1], 1)
        self.assertAlmostEqual(fractions[50], 0.5, delta=0.002)
        self.assertTrue(np.all(np.diff(fractions) >= -1e-8))
        r = np.array([-2e9, 0, 0])
        actual = sunlight_fraction(r, r + [AU_M, 0, 0], earth, sun_radius)
        self.assertAlmostEqual(actual, 1 - (np.arcsin(earth / 2e9) / a) ** 2)

    def test_plant_tracks_energy_using_applied_dipole(self):
        self.config["spacecraft"]["solar_charging"]["panels"] = []
        plant = Plant(self.config)
        before = plant.power.battery_energy_Wh
        applied = plant.step([10, -0.1, 0], 0)
        np.testing.assert_allclose(applied, [0.2, -0.1, 0])
        self.assertAlmostEqual(plant.power.magnetorquer_power_W, 0.75)
        self.assertAlmostEqual(plant.power.battery_energy_Wh, before - 0.75 * plant.dt / 3600)

    def test_full_load_in_eclipse_exhausts_small_battery(self):
        # 0.5 Wh at 80% / 1.5 W = 960 s. This orbit starts in eclipse,
        # including the first 980 s; leave solar generation enabled.
        self.config["spacecraft"]["battery_capacity_Wh"] = 0.5
        self.config["simulation"].update(step_s=1.0, field_period_s=1.0)
        plant = Plant(self.config)
        for tick in range(980):
            plant.step([0.2, -0.2, 0.2], 0)
            self.assertEqual(plant.power.solar_power_W, 0)
            self.assertAlmostEqual(plant.power.battery_energy_Wh,
                                   max(0, 0.4 - (tick + 1) * 1.5 / 3600), places=11)
        self.assertAlmostEqual(plant.power.unserved_energy_Wh, 20 * 1.5 / 3600)

    def test_energy_ledger_conserves_energy_through_full_and_empty(self):
        self.config["spacecraft"]["battery_capacity_Wh"] = 0.01
        self.config["spacecraft"]["nominal_power_W"] = 0.7
        power = PowerSystem(self.config)
        rng = np.random.default_rng(4)
        for i in range(1000):
            # Alternating eclipse/daylight with changing loads and step sizes
            # exercises saturation, charger limits, losses and unmet demand.
            solar = 0 if i % 40 < 20 else 10
            power.advance(rng.uniform(-0.4, 0.4, 3), rng.uniform(1, 10), solar, solar)
            balance = (0.008 + power.solar_energy_Wh + power.unserved_energy_Wh
                       - power.magnetorquer_energy_Wh - power.nominal_energy_Wh - power.charging_loss_Wh
                       - power.curtailed_energy_Wh)
            self.assertAlmostEqual(power.battery_energy_Wh, balance, places=11)
        self.assertGreater(power.unserved_energy_Wh, 0)
        self.assertGreater(power.charging_loss_Wh, 0)
        self.assertGreater(power.curtailed_energy_Wh, 0)

    def test_four_side_panels_do_not_collect_on_back_faces(self):
        solar = self.config["spacecraft"]["solar_charging"]
        solar["panels"] = solar["panels"][:4]
        r = np.array([0, 0, 7e6])
        state = {"position_m": r, "attitude_mrp": [0, 0, 0]}
        # At 45 degrees, two faces each collect cos(45), not four full faces.
        for direction, projected_area in (([1, 0, 0], 0.01),
                                          ([1, 1, 0], 0.01 * np.sqrt(2)),
                                          ([0, 0, 1], 0)):
            direction = np.asarray(direction) / np.linalg.norm(direction)
            with patch("cubesat_sim.power.sun_position_eci", return_value=r + AU_M * direction):
                power, fraction = self.power.solar_power(state, 0)
            self.assertEqual(fraction, 1)
            self.assertAlmostEqual(power, 1361 * 0.3 * 0.95 * projected_area)

    def test_penumbra_against_independent_solar_disk_ray_sampling(self):
        # Uniform solar-disk samples: test each ray against Earth's angular
        # limb, independently of the production circle-intersection formula.
        r = np.array([7e6, 0, 0])
        earth, sun_radius = 6378137, 6.957e8
        a, b = np.arcsin(sun_radius / AU_M), np.arcsin(earth / np.linalg.norm(r))
        samples = np.arange(30000) + 0.5
        rho = a * np.sqrt(samples / len(samples))
        phi = samples * np.pi * (3 - np.sqrt(5))
        for offset in (-0.9, -0.5, 0, 0.5, 0.9):
            angle = b + offset * a
            direction = np.array([-np.cos(angle), np.sin(angle), 0])
            tangent = np.array([np.sin(angle), np.cos(angle), 0])
            rays = (np.cos(rho)[:, None] * direction
                    + (np.sin(rho) * np.cos(phi))[:, None] * tangent
                    + (np.sin(rho) * np.sin(phi))[:, None] * [0, 0, 1])
            visible = np.mean(-rays[:, 0] < np.cos(b))
            actual = sunlight_fraction(r, r + AU_M * direction, earth, sun_radius)
            self.assertAlmostEqual(actual, visible, delta=0.002)

    def test_config_validation_and_normalization(self):
        invalid = [
            ("battery_initial_soc", -0.01), ("battery_initial_soc", 1.01),
            ("battery_initial_soc", float("inf")), ("battery_initial_soc", float("nan")),
            ("nominal_power_W", -1), ("nominal_power_W", float("inf")),
            ("nominal_power_W", float("nan")),
            ("battery_capacity_Wh", 0), ("battery_capacity_Wh", float("nan")),
            ("magnetorquer_max_power_W", [-1, 0, 0]),
            ("magnetorquer_max_power_W", [1, 2]),
            ("magnetorquer_max_power_W", [1, float("inf"), 1]),
        ]
        with TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            for key, value in invalid:
                config = copy.deepcopy(self.config)
                config["spacecraft"][key] = value
                path.write_text(yaml.safe_dump(config))
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    load_config(path)
            for value in (0, -1, float("inf"), float("nan")):
                config = copy.deepcopy(self.config)
                config["environment"]["sun_radius_m"] = value
                path.write_text(yaml.safe_dump(config))
                with self.subTest(sun_radius_m=value), self.assertRaises(ValueError):
                    load_config(path)
            for key in ("conversion_efficiency", "battery_charge_efficiency", "max_charge_power_W", "irradiance_W_m2_at_1au"):
                config = copy.deepcopy(self.config)
                config["spacecraft"]["solar_charging"][key] = float("inf")
                path.write_text(yaml.safe_dump(config))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    load_config(path)
            for key, value in (("normal_body", [0, 0, 0]), ("area_m2", -1), ("efficiency", 1.1)):
                config = copy.deepcopy(self.config)
                config["spacecraft"]["solar_charging"]["panels"][0][key] = value
                path.write_text(yaml.safe_dump(config))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    load_config(path)
            self.config["spacecraft"]["solar_charging"]["panels"][0]["normal_body"] = [2, 0, 0]
            path.write_text(yaml.safe_dump(self.config))
            self.assertEqual(load_config(path)["spacecraft"]["solar_charging"]["panels"][0]["normal_body"], [1, 0, 0])

    def test_csv_power_columns_and_energy_balance(self):
        self.config["spacecraft"]["solar_charging"]["panels"] = []
        self.config["spacecraft"]["nominal_power_W"] = 1
        self.config["spacecraft"]["battery_initial_soc"] = 0.35
        with TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.csv"
            self.config["simulation"].update(duration_s=2, output_csv=str(path))
            with patch("cubesat_sim.__main__.SitlController") as controller:
                controller.return_value.step.return_value = ([0.2, 0, 0], 0)
                plant = run(self.config)
                updates = controller.return_value.battery.call_args_list
                self.assertEqual(len(updates), 2)
                self.assertEqual(updates[0].args, (0, 0.35, 0))
                time_s, soc, power_W = updates[1].args
                self.assertEqual(time_s, 1)
                self.assertAlmostEqual(soc, (3.5 - 1.5 / 3600) / 10)
                self.assertAlmostEqual(power_W, -1.5)
            with path.open() as file:
                rows = list(csv.DictReader(file))
        self.assertEqual(len(rows), 2)
        for row in rows:
            self.assertAlmostEqual(float(row["battery_energy_Wh"]), 3.5 - 1.5 * float(row["time_s"]) / 3600)
            self.assertAlmostEqual(float(row["magnetorquer_power_W"]), 0.5)
            self.assertAlmostEqual(float(row["battery_power_W"]), -1.5)
            self.assertEqual(float(row["nominal_power_W"]), 1)
            self.assertAlmostEqual(float(row["nominal_energy_Wh"]), float(row["time_s"]) / 3600)
            self.assertEqual(float(row["solar_power_W"]), 0)
            self.assertEqual(float(row["unserved_energy_Wh"]), 0)
            self.assertAlmostEqual(float(row["magnetorquer_energy_Wh"]), 0.5 * float(row["time_s"]) / 3600)
            self.assertEqual(float(row["solar_energy_Wh"]), 0)
            self.assertEqual(float(row["charging_loss_Wh"]), 0)
            self.assertEqual(float(row["curtailed_energy_Wh"]), 0)
        self.assertEqual(float(rows[-1]["battery_soc"]), plant.power.battery_soc)

    def test_sparse_csv_preserves_energy_when_power_samples_miss_pulses(self):
        self.config["spacecraft"]["solar_charging"]["panels"] = []
        with TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.csv"
            self.config["simulation"].update(duration_s=2, step_s=0.01,
                                             control_period_s=0.01, output_csv=str(path))
            with patch("cubesat_sim.__main__.SitlController") as controller:
                # 40% duty at 1.5 W; each one-second CSV sample lands in off time.
                controller.return_value.step.side_effect = lambda tick, *_: (
                    [0.2, 0.2, 0.2] if tick % 100 < 40 else [0, 0, 0], 0)
                run(self.config)
            with path.open() as file:
                rows = list(csv.DictReader(file))
        for row in rows:
            consumed = 0.6 * float(row["time_s"]) / 3600
            self.assertEqual(float(row["magnetorquer_power_W"]), 0)
            self.assertEqual(float(row["battery_power_W"]), 0)
            self.assertAlmostEqual(float(row["magnetorquer_energy_Wh"]), consumed)
            self.assertAlmostEqual(float(row["battery_energy_Wh"]), 8 - consumed)


if __name__ == "__main__":
    unittest.main()
