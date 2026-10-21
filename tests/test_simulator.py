from wire_fixture import frame, text_frame
import copy
import os
import pty
import threading
import unittest
from pathlib import Path
from tempfile import NamedTemporaryFile
from unittest.mock import patch

import numpy as np
import yaml

from cubesat_sim.config import load_config
from cubesat_sim.io import HitlController
from cubesat_sim.physics import Plant, Sensors
from cubesat_sim.sun import sun_position_eci
from cubesat_sim.__main__ import run
from Basilisk.utilities import RigidBodyKinematics

ROOT = Path(__file__).resolve().parents[1]


class SimulatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "config.example.yaml")

    def test_each_step_advances_dynamics_after_long_simulated_time(self):
        plant = Plant(copy.deepcopy(self.config))
        reader = plant.sc.scStateOutMsg.addSubscriber()
        # Repeated addition of 0.2 s used to miss the task at step 14796.
        for tick in range(1, 15001):
            plant.step([0, 0, 0], 0)
            self.assertEqual(reader.timeWritten(), tick * 200_000_000)
            self.assertAlmostEqual(plant.time_s, tick / 5, delta=1e-9)

    def test_body_field_for_known_positive_z_rotation(self):
        config = copy.deepcopy(self.config)
        config['orbit']['attitude_mrp'] = [0, 0, np.tan(np.pi / 8)]
        config['orbit']['angular_rate_rad_s'] = [0, 0, 0.01]
        for mag in config['sensors']['magnetometers']:
            mag['noise_std_T'] = 0
        plant = Plant(config)
        # Orbital gravity is already initialized; disable only explicit GG torque.
        config['environment']['earth_mu_m3_s2'] = 0
        plant.magnetic_field_n = lambda: np.array([25e-6, 0, 0])
        sensors = Sensors(config)
        for tick in range(501):
            theta = np.pi / 2 + 0.01 * plant.time_s
            expected = 25e-6 * np.array([np.cos(theta), -np.sin(theta), 0])
            np.testing.assert_allclose(sensors.magnetometers(plant)[0], expected, atol=1e-15, rtol=0)
            if tick < 500:
                plant.step([0, 0, 0], 0)

    def test_torque_free_body_rates_reverse_with_conserved_momentum(self):
        config = copy.deepcopy(self.config)
        config['orbit']['angular_rate_rad_s'] = [0.09, -0.09, 0.09]
        plant = Plant(config)
        config['environment']['earth_mu_m3_s2'] = 0
        inertia = np.asarray(config['spacecraft']['bus_inertia_kg_m2'])
        initial = plant.state()
        initial_w = initial['angular_rate_rad_s']
        initial_h = np.asarray(RigidBodyKinematics.MRP2C(initial['attitude_mrp'])).T @ (inertia @ initial_w)
        initial_energy = 0.5 * initial_w @ inertia @ initial_w
        max_y = initial_w[1]
        for _ in range(3000):
            plant.step([0, 0, 0], 0)
            state = plant.state()
            w = state['angular_rate_rad_s']
            h = np.asarray(RigidBodyKinematics.MRP2C(state['attitude_mrp'])).T @ (inertia @ w)
            self.assertLess(np.linalg.norm(h - initial_h) / np.linalg.norm(initial_h), 1e-8)
            self.assertLess(abs((0.5 * w @ inertia @ w) / initial_energy - 1), 1e-9)
            max_y = max(max_y, w[1])
        self.assertGreater(max_y, 0.09)

    def test_wmm_sun_and_boom_change_with_time(self):
        config = copy.deepcopy(self.config)
        plant = Plant(config)
        sensors = Sensors(config)
        initial_field = plant.magnetic_field_n().copy()
        initial_inertia = np.asarray(plant.sc.scMassOutMsg.read().ISC_PntB_B)
        self.assertEqual(len(sensors.magnetometers(plant)), 2)
        for _ in range(2):
            plant.step([0, 0, 0], 1.0)
        updated_inertia = np.asarray(plant.sc.scMassOutMsg.read().ISC_PntB_B)
        self.assertGreater(np.linalg.norm(plant.magnetic_field_n() - initial_field), 1e-10)
        self.assertAlmostEqual(plant.boom_length, 0.02)
        self.assertGreater(updated_inertia[0, 0], initial_inertia[0, 0])
        self.assertGreater(np.linalg.norm(sun_position_eci(config["simulation"]["start_utc"], 86400) - sun_position_eci(config["simulation"]["start_utc"], 0)), 1e9)

    def test_configured_magnetometer_count_and_raw_frames(self):
        for count in (1, 3):
            config = copy.deepcopy(self.config)
            mags = config['sensors']['magnetometers']
            mags[:] = [copy.deepcopy(mags[i % len(mags)]) for i in range(count)]
            for i, mag in enumerate(mags):
                mag['noise_std_T'] = 0
                mag['bias_T'] = [i * 1e-6, 0, 0]
            with NamedTemporaryFile(mode='w', suffix='.yaml') as file:
                yaml.safe_dump(config, file)
                file.flush()
                config = load_config(file.name)
            plant = Plant(config)
            readings = Sensors(config).magnetometers(plant)
            self.assertEqual(len(readings), count)
            body_field = np.asarray(RigidBodyKinematics.MRP2C(
                plant.state()['attitude_mrp'])) @ plant.magnetic_field_n()
            for mag, reading in zip(config['sensors']['magnetometers'], readings):
                expected = np.asarray(mag['body_to_sensor']) @ body_field + mag['bias_T']
                np.testing.assert_allclose(reading, expected, atol=1e-15, rtol=0)

    def test_independent_callback_rates(self):
        config = copy.deepcopy(self.config)
        config["simulation"].update(duration_s=2.0, output_csv=None, accelerometer_period_s=0.6, gyroscope_period_s=0.8)
        config["sensors"]["magnetometers"].append(copy.deepcopy(config["sensors"]["magnetometers"][0]))
        calls = {"mag": 0, "accel": 0, "gyro": 0, "sun": 0, "battery": 0, "step": 0}
        test = self

        class FakeController:
            def __init__(self, _):
                pass

            def magnetometers(self, time_s, *readings):
                test.assertEqual(len(readings), 3)
                calls["mag"] += 1

            def accelerometers(self, time_s, *readings):
                test.assertEqual(len(readings), 2)
                test.assertAlmostEqual(time_s, calls["accel"] * 0.6)
                calls["accel"] += 1

            def gyroscope(self, time_s, reading):
                test.assertEqual(len(reading), 3)
                test.assertAlmostEqual(time_s, calls["gyro"] * 0.8)
                calls["gyro"] += 1

            def sun(self, *_):
                calls["sun"] += 1

            def step(self, *_):
                test.assertEqual(calls["battery"], calls["step"] + 1)
                calls["step"] += 1
                return np.zeros(3), 0.0

            def battery(self, time_s, soc, power_W):
                test.assertEqual(time_s, calls["step"])
                if time_s == 0:
                    test.assertEqual(soc, 0.8)
                    test.assertEqual(power_W, 0)
                calls["battery"] += 1

            def close(self):
                pass

        with patch("cubesat_sim.__main__.SitlController", FakeController):
            run(config)
        self.assertEqual(calls, {"mag": 5, "accel": 4, "gyro": 3, "sun": 2, "battery": 2, "step": 2})

    def test_sun_sensor_uses_spacecraft_location_and_eclipse(self):
        config = copy.deepcopy(self.config)
        config["sensors"]["sun_sensor"]["half_angle_deg"] = 180.0
        sun_n = sun_position_eci(config["simulation"]["start_utc"], 0)
        sun_n /= np.linalg.norm(sun_n)
        radius = np.linalg.norm(config["orbit"]["position_m"])
        config["orbit"]["position_m"] = (radius * sun_n).tolist()
        lit = Sensors(config).sun(Plant(config))
        config["orbit"]["position_m"] = (-radius * sun_n).tolist()
        dark = Sensors(config).sun(Plant(config))
        self.assertIsNotNone(lit)
        self.assertIsNone(dark)

    def test_hitl_uart_round_trip(self):
        master, slave = pty.openpty()
        controller = None
        try:
            lines = []
            received = []

            def board():
                data = bytearray()
                while len(lines) < 8:
                    data.extend(os.read(master, 1024))
                    while b"\0" in data:
                        line, _, rest = data.partition(b"\0")
                        lines.append(text_frame(line))
                        data = bytearray(rest)
                os.write(master, frame("A 7 100000 0 -100000 400"))

            thread = threading.Thread(target=board)
            thread.start()
            controller = HitlController({"port": os.ttyname(slave), "baudrate": 115200, "timeout_s": 1, "exchange": "lockstep"},
                                        on_receive=received.append)
            controller.magnetometers(1.0, [1e-5, 0, -1e-5], [0, 2e-5, 0])
            controller.accelerometers(1.0, [0.001, -0.0005, 0.00025], [-0.0004, 0.0009, -0.0001])
            controller.gyroscope(1.0, [0.01, -0.02, 0.015])
            controller.sun(1.0, [1, 0, 0])
            controller.battery(1.0, 0.8, -0.75)
            dipole, boom = controller.step(7, 1.0, 1.0)
            self.assertEqual(lines[0], "M 1000 0 10000 0 -10000")
            self.assertEqual(lines[1], "M 1000 1 0 20000 0")
            self.assertEqual(lines[2], "C 1000 0 1000 -500 250")
            self.assertEqual(lines[3], "C 1000 1 -400 900 -100")
            self.assertEqual(lines[4], "G 1000 10000 -20000 15000")
            self.assertEqual(lines[5], "S 1000 1 32767 0 0")
            self.assertEqual(lines[6], "B 1000 800 -750")
            self.assertEqual(lines[7], "T 7 1000 1000")
            self.assertAlmostEqual(dipole[0], 0.1)
            self.assertAlmostEqual(dipole[2], -0.1)
            self.assertEqual(boom, 0.4)
            self.assertEqual(b"".join(received), frame("A 7 100000 0 -100000 400"))
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive())
        finally:
            if controller:
                controller.close()
            os.close(master)
            os.close(slave)

    def test_hitl_requires_realtime_and_period_multiples(self):
        config = copy.deepcopy(self.config)
        config["simulation"].update(mode="hitl", clock="simulated")
        with NamedTemporaryFile(mode="w", suffix=".yaml") as file:
            yaml.safe_dump(config, file)
            file.flush()
            with self.assertRaisesRegex(ValueError, "realtime"):
                load_config(file.name)
        config["simulation"].update(mode="sitl", clock="simulated", control_period_s=0.3)
        with NamedTemporaryFile(mode="w", suffix=".yaml") as file:
            yaml.safe_dump(config, file)
            file.flush()
            with self.assertRaisesRegex(ValueError, "multiple"):
                load_config(file.name)


if __name__ == "__main__":
    unittest.main()
