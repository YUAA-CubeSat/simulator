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


class GyroscopeTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / 'config.example.yaml')

    def test_sensor_rotation_bias_and_propagated_rates(self):
        gyro = self.config['sensors']['gyroscope']
        gyro.update(noise_std_rad_s=0,
                    body_to_sensor=[[0, 1, 0], [-1, 0, 0], [0, 0, 1]],
                    bias_rad_s=[0.001, -0.002, 0.003])
        plant = Plant(self.config)
        sensors = Sensors(self.config)
        for _ in range(5):
            rate = plant.state()['angular_rate_rad_s']
            expected = np.array([rate[1], -rate[0], rate[2]]) + gyro['bias_rad_s']
            np.testing.assert_allclose(sensors.gyroscope(plant), expected, atol=1e-15)
            plant.step([0.1, -0.1, 0], 0)

    def test_noise_scale_repeatability_and_independent_streams(self):
        self.config['sensors']['gyroscope']['bias_rad_s'] = [0.01, -0.02, 0.03]
        plant = SimpleNamespace(state=lambda: {'angular_rate_rad_s': np.zeros(3)})
        first, second, untouched = (Sensors(self.config) for _ in range(3))
        samples = np.array([first.gyroscope(plant) for _ in range(8000)])
        repeated = np.array([second.gyroscope(plant) for _ in range(8000)])
        np.testing.assert_array_equal(samples, repeated)
        np.testing.assert_allclose(samples.mean(axis=0), [0.01, -0.02, 0.03], atol=4e-6)
        np.testing.assert_allclose(samples.std(axis=0), 1e-4, rtol=0.04)
        np.testing.assert_allclose(np.corrcoef(samples.T), np.eye(3), atol=0.04)
        np.testing.assert_array_equal(first.rng.normal(size=10), untouched.rng.normal(size=10))
        np.testing.assert_array_equal(first.accel_rng.normal(size=10), untouched.accel_rng.normal(size=10))
        first.rng.normal(size=100)
        first.accel_rng.normal(size=100)
        np.testing.assert_array_equal(first.gyroscope(plant), second.gyroscope(plant))

    def test_invalid_sensor_configuration(self):
        cases = [
            ('noise_std_rad_s', -1), ('noise_std_rad_s', float('nan')),
            ('noise_std_rad_s', float('inf')), ('bias_rad_s', [0, 0]),
            ('bias_rad_s', [0, float('nan'), 0]),
            ('body_to_sensor', [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
            ('body_to_sensor', [[2, 0, 0], [0, 1, 0], [0, 0, 1]]),
            ('body_to_sensor', [[float('nan'), 0, 0], [0, 1, 0], [0, 0, 1]]),
            ('body_to_sensor', [[1, 0], [0, 1]]),
        ]
        for key, value in cases:
            config = copy.deepcopy(self.config)
            config['sensors']['gyroscope'][key] = value
            with self.subTest(key=key, value=value):
                self.assert_invalid(config, 'gyroscope')
        for value in ([], [self.config['sensors']['gyroscope']], None):
            config = copy.deepcopy(self.config)
            config['sensors']['gyroscope'] = value
            self.assert_invalid(config, 'single sensor mapping')

    def test_invalid_period(self):
        for period in (0, -1, 0.1, 0.3, float('nan'), float('inf')):
            self.config['simulation']['gyroscope_period_s'] = period
            with self.subTest(period=period):
                self.assert_invalid(self.config, 'gyroscope_period_s')

    def assert_invalid(self, config, message):
        with NamedTemporaryFile(mode='w', suffix='.yaml') as file:
            yaml.safe_dump(config, file)
            file.flush()
            with self.assertRaisesRegex(ValueError, message):
                load_config(file.name)
