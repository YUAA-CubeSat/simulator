import copy
import unittest
from pathlib import Path
from tempfile import NamedTemporaryFile
from types import SimpleNamespace

import numpy as np
import yaml

from cubesat_sim.config import load_config
from cubesat_sim.physics import Plant, Sensors


ROOT = Path(__file__).resolve().parents[1]


class AccelerometerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / 'config.example.yaml')

    def test_free_fall_and_applied_body_force(self):
        self.config['orbit']['angular_rate_rad_s'] = [0, 0, 0]
        plant = Plant(self.config)
        # Keep central orbital gravity, but disable the explicit gravity-gradient torque.
        self.config['environment']['earth_mu_m3_s2'] = 0
        np.testing.assert_allclose(plant.specific_force_body(), 0, atol=1e-12)
        plant.step([0, 0, 0], 0)
        np.testing.assert_allclose(plant.specific_force_body(), 0, atol=1e-12)
        mass = self.config['spacecraft']['bus_mass_kg'] + self.config['spacecraft']['boom']['tip_mass_kg']
        plant.effect.extForce_B = [mass * 0.1, 0, 0]
        plant.step([0, 0, 0], 0)
        np.testing.assert_allclose(plant.specific_force_body(), [0.1, 0, 0], atol=1e-10)

    def test_sensor_frames_bias_and_configurable_count(self):
        accels = self.config['sensors']['accelerometers']
        accels.append(copy.deepcopy(accels[0]))
        for i, accel in enumerate(accels):
            accel['noise_std_m_s2'] = 0
            accel['bias_m_s2'] = [i * 0.01, -0.02, 0.03]
        plant = SimpleNamespace(specific_force_body=lambda: np.array([1., 2., 3.]))
        readings = Sensors(self.config).accelerometers(plant)
        self.assertEqual(len(readings), 3)
        for accel, reading in zip(accels, readings):
            expected = np.asarray(accel['body_to_sensor']) @ [1., 2., 3.] + accel['bias_m_s2']
            np.testing.assert_allclose(reading, expected)

    def test_noise_is_repeatable_independent_and_has_configured_scale(self):
        plant = SimpleNamespace(specific_force_body=lambda: np.zeros(3))
        first, second = Sensors(self.config), Sensors(self.config)
        samples = np.array([first.accelerometers(plant) for _ in range(4000)])
        repeated = np.array([second.accelerometers(plant) for _ in range(4000)])
        np.testing.assert_array_equal(samples, repeated)
        np.testing.assert_allclose(samples.mean(axis=0), 0, atol=0.00006)
        np.testing.assert_allclose(samples.std(axis=0), 0.001, rtol=0.06)
        self.assertLess(abs(np.corrcoef(samples[:, 0, 0], samples[:, 1, 0])[0, 1]), 0.06)
        untouched = Sensors(self.config)
        np.testing.assert_array_equal(first.rng.normal(size=3), untouched.rng.normal(size=3))

    def test_invalid_sensor_configuration(self):
        cases = [
            ('noise_std_m_s2', -1), ('noise_std_m_s2', float('nan')),
            ('noise_std_m_s2', float('inf')), ('bias_m_s2', [0, 0]),
            ('bias_m_s2', [0, float('nan'), 0]),
            ('body_to_sensor', [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
            ('body_to_sensor', [[float('nan'), 0, 0], [0, 1, 0], [0, 0, 1]]),
        ]
        for key, value in cases:
            config = copy.deepcopy(self.config)
            config['sensors']['accelerometers'][0][key] = value
            with self.subTest(key=key, value=value):
                self.assert_invalid(config, 'accelerometer')
        self.config['sensors']['accelerometers'] = []
        self.assert_invalid(self.config, 'accelerometer')

    def test_invalid_period(self):
        for period in (0, -1, 0.3, float('inf')):
            self.config['simulation']['accelerometer_period_s'] = period
            with self.subTest(period=period):
                self.assert_invalid(self.config, 'accelerometer_period_s')

    def assert_invalid(self, config, message):
        with NamedTemporaryFile(mode='w', suffix='.yaml') as file:
            yaml.safe_dump(config, file)
            file.flush()
            with self.assertRaisesRegex(ValueError, message):
                load_config(file.name)
