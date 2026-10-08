import copy
import csv
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from cubesat_sim.config import load_config
from cubesat_sim.__main__ import run
from cubesat_sim.orbit_solar import KeplerOrbit, solar_sample
from cubesat_sim.sun import sun_position_eci
from utilities.plot_solar import analyze

ROOT = Path(__file__).resolve().parents[1]


class OrbitSolarTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / 'config.orbit_solar.yaml')

    def test_orbit_initial_state_period_and_conservation(self):
        # Circular and eccentric, inclined ellipses, starting away from periapsis.
        for velocity in ([0, 7612.6, 0], [900, 7700, 1800]):
            self.config['orbit']['velocity_m_s'] = velocity
            orbit = KeplerOrbit(self.config)
            r0 = np.array(self.config['orbit']['position_m'])
            v0 = np.array(velocity)
            mu = self.config['environment']['earth_mu_m3_s2']
            for t in (0, orbit.period_s):
                r, v = orbit.state(t)
                np.testing.assert_allclose(r, r0, atol=1e-7)
                np.testing.assert_allclose(v, v0, atol=1e-9)
            for t in np.linspace(0, orbit.period_s, 50):
                r, v = orbit.state(t)
                np.testing.assert_allclose(np.cross(r, v), np.cross(r0, v0), atol=1e-4)
                self.assertAlmostEqual(np.dot(v, v) / 2 - mu / np.linalg.norm(r),
                                       np.dot(v0, v0) / 2 - mu / np.linalg.norm(r0), delta=1e-6)

    def test_solar_eclipse_and_area_scaling(self):
        direction = sun_position_eci(self.config['simulation']['start_utc'], 0)
        direction /= np.linalg.norm(direction)
        r = direction * 6878137
        lit, fraction = solar_sample(self.config, r, 0)
        dark, shadow = solar_sample(self.config, -r, 0)
        self.assertGreater(lit, 0)
        self.assertEqual((fraction, dark, shadow), (1, 0, 0))
        self.config['solar']['effective_area_m2'] *= 2
        self.assertAlmostEqual(solar_sample(self.config, r, 0)[0], 2 * lit)

    def test_run_no_controller_initial_final_and_step_integral(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'solar.csv'
            self.config['simulation'].update(duration_s=100, telemetry_period_s=60, output_csv=str(path))
            with patch('cubesat_sim.__main__.SitlController', side_effect=AssertionError), \
                    patch('cubesat_sim.__main__.HitlController', side_effect=AssertionError), \
                    patch('sys.stderr', io.StringIO()):
                result = run(self.config)
            with path.open() as source:
                rows = list(csv.DictReader(source))
            self.assertEqual(float(rows[0]['time_s']), 0)
            self.assertEqual(float(rows[-1]['time_s']), 100)
            self.assertNotIn('attitude_mrp', result)
            orbit = KeplerOrbit(self.config)
            powers = [solar_sample(self.config, orbit.state(t)[0], t)[0] for t in range(0, 101, 10)]
            expected = sum((a + b) * 5 / 3600 for a, b in zip(powers, powers[1:]))
            self.assertAlmostEqual(result['solar_energy_Wh'], expected)

    def test_interrupt_saves_last_completed_sample(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'solar.csv'
            self.config['simulation'].update(duration_s=100, output_csv=str(path))
            real_sample = solar_sample
            def sample(config, r, t):
                if t == 30:
                    raise KeyboardInterrupt
                return real_sample(config, r, t)
            with patch('cubesat_sim.orbit_solar.solar_sample', side_effect=sample), patch('sys.stderr', io.StringIO()):
                with self.assertRaises(KeyboardInterrupt):
                    run(self.config)
            with path.open() as source:
                rows = list(csv.DictReader(source))
            self.assertEqual([float(row['time_s']) for row in rows], [0, 20])

    def test_minimal_config_and_invalid_inputs(self):
        for section, key, value in [('solar', 'effective_area_m2', -1), ('solar', 'efficiency', 1.1),
                                     ('simulation', 'step_s', 7), ('simulation', 'checkpoint_steps', -1),
                                     ('orbit', 'velocity_m_s', [0, 20000, 0]),
                                     ('orbit', 'velocity_m_s', [0, 100, 0])]:
            config = copy.deepcopy(self.config)
            config[section][key] = value
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as f:
                yaml.safe_dump(config, f)
                f.flush()
                with self.assertRaises(ValueError):
                    load_config(f.name)

    def test_cycle_accounting_and_partial_intervals(self):
        time = np.arange(0, 26, dtype=float)
        data = np.column_stack((np.ones(26), ((time % 10) >= 3).astype(float), time / 3600, np.full(26, 10)))
        orbits, cycles = analyze(time, data)
        np.testing.assert_array_equal(orbits[2], [True, True, False])
        np.testing.assert_array_equal(cycles[2], [False, True, True, False])
        for _, energy, _ in (orbits, cycles):
            self.assertAlmostEqual(sum(energy), 25 / 3600)
        data[:, 1] = 1
        _, cycles = analyze(time, data)
        self.assertFalse(cycles[2].any())
        self.assertEqual(len(cycles[1]), 1)


if __name__ == '__main__':
    unittest.main()
