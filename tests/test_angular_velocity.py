import tempfile
import unittest
from pathlib import Path

import numpy as np

from utilities.plot_angular_velocity import body_to_world, load_rates


class AngularVelocityTests(unittest.TestCase):
    def test_known_rotation_and_magnitude(self):
        rates = np.array([[1., 2., 3.], [1., 0., 0.]])
        attitude = np.array([[0., 0., 0.], [0., 0., np.tan(np.pi / 8)]])
        world = body_to_world(rates, attitude)
        np.testing.assert_allclose(world, [[1, 2, 3], [0, 1, 0]], atol=1e-15)
        np.testing.assert_allclose(np.linalg.norm(world, axis=1), np.linalg.norm(rates, axis=1))

    def test_rotation_matches_basilisk(self):
        from Basilisk.utilities import RigidBodyKinematics
        rng = np.random.default_rng(7)
        sigma = rng.normal(size=(20, 3))
        rates = rng.normal(size=(20, 3))
        expected = [RigidBodyKinematics.MRP2C(s).T @ w for s, w in zip(sigma, rates)]
        np.testing.assert_allclose(body_to_world(rates, sigma), expected, atol=1e-14)

    def test_old_csv_body_and_missing_world_attitude(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'telemetry.csv'
            path.write_text('time_s,wx_rad_s,wy_rad_s,wz_rad_s\n1,1,2,3\n')
            time, rates = load_rates(path, 'body')
            np.testing.assert_array_equal(time, [1])
            np.testing.assert_array_equal(rates, [[1, 2, 3]])
            with self.assertRaisesRegex(ValueError, 'rerun the simulation'):
                load_rates(path, 'world')

    def test_csv_world_conversion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'telemetry.csv'
            path.write_text('time_s,wx_rad_s,wy_rad_s,wz_rad_s,sigma_BN_x,sigma_BN_y,sigma_BN_z\n'
                            f'1,1,0,0,0,0,{np.tan(np.pi / 8)}\n')
            _, rates = load_rates(path, 'world')
            np.testing.assert_allclose(rates, [[0, 1, 0]], atol=1e-15)
