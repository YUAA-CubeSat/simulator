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

ROOT = Path(__file__).resolve().parents[1]


class SimulatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "config.example.yaml")

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

    def test_independent_callback_rates(self):
        config = copy.deepcopy(self.config)
        config["simulation"].update(duration_s=2.0, output_csv=None)
        calls = {"mag": 0, "sun": 0, "step": 0}

        class FakeController:
            def __init__(self, _):
                pass

            def magnetometers(self, *_):
                calls["mag"] += 1

            def sun(self, *_):
                calls["sun"] += 1

            def step(self, *_):
                calls["step"] += 1
                return np.zeros(3), 0.0

            def close(self):
                pass

        with patch("cubesat_sim.__main__.SitlController", FakeController):
            run(config)
        self.assertEqual(calls, {"mag": 5, "sun": 2, "step": 2})

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
                while len(lines) < 3:
                    data.extend(os.read(master, 1024))
                    while b"\n" in data:
                        line, _, rest = data.partition(b"\n")
                        lines.append(line.decode("ascii"))
                        data = bytearray(rest)
                os.write(master, b"A 7 100000 0 -100000 400\n")

            thread = threading.Thread(target=board)
            thread.start()
            controller = HitlController({"port": os.ttyname(slave), "baudrate": 115200, "timeout_s": 1},
                                        on_receive=received.append)
            controller.magnetometers(1.0, [1e-5, 0, -1e-5], [0, 2e-5, 0])
            controller.sun(1.0, [1, 0, 0])
            dipole, boom = controller.step(7, 1.0, 1.0)
            self.assertEqual(lines[0], "M 1000 10000 0 -10000 0 20000 0")
            self.assertEqual(lines[1], "S 1000 1 32767 0 0")
            self.assertEqual(lines[2], "T 7 1000 1000")
            self.assertAlmostEqual(dipole[0], 0.1)
            self.assertAlmostEqual(dipole[2], -0.1)
            self.assertEqual(boom, 0.4)
            self.assertEqual(b"".join(received), b"A 7 100000 0 -100000 400\n")
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
