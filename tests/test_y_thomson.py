import unittest

import numpy as np
from Basilisk.utilities import RigidBodyKinematics

from utilities.plot_y_thomson import pointing_metrics


class YThomsonTests(unittest.TestCase):
    def orbit(self, offset=None):
        time = np.array([0., 0.7, 1.8, 3., 4.5])
        n = 0.00113
        phase = n * time
        inc = np.radians(51.6)
        radial = np.column_stack((np.cos(phase), np.sin(phase) * np.cos(inc),
                                  np.sin(phase) * np.sin(inc)))
        normal = np.array([0., -np.sin(inc), np.cos(inc)])
        y = np.broadcast_to(-normal, radial.shape)
        z = -radial
        body = np.stack((np.cross(y, z), y, z), axis=-1)
        if offset is not None:
            body = body @ offset
        attitude = np.array([RigidBodyKinematics.C2MRP(c.T) for c in body])
        rates = np.einsum('nji,j->ni', body, n * normal)
        return time, radial * 6778137, attitude, rates

    def test_perfect_tracking_in_inclined_orbit_with_uneven_samples(self):
        angles, total, residual, orbital = pointing_metrics(*self.orbit())
        np.testing.assert_allclose(angles, 0, atol=3e-6)
        np.testing.assert_allclose(total, 0, atol=3e-6)
        np.testing.assert_allclose(residual, 0, atol=2e-9)
        np.testing.assert_allclose(orbital, 0.00113, atol=2e-9)

    def test_nadir_alone_does_not_hide_wrong_y_axis(self):
        angles, total, residual, _ = pointing_metrics(*self.orbit(np.diag([-1, -1, 1])))
        np.testing.assert_allclose(angles, np.tile([180, 180, 0], (5, 1)), atol=3e-6)
        np.testing.assert_allclose(total, 180, atol=3e-6)
        np.testing.assert_allclose(residual, 0, atol=2e-9)

    def test_zero_inertial_rate_is_not_orbit_tracking(self):
        time, position, attitude, rates = self.orbit()
        _, _, residual, orbital = pointing_metrics(time, position, attitude, rates * 0)
        np.testing.assert_allclose(residual[:, 1], orbital, atol=1e-12)
        np.testing.assert_allclose(residual[:, [0, 2]], 0, atol=1e-12)

    def test_invalid_orbit_and_insufficient_samples(self):
        args = self.orbit()
        with self.assertRaisesRegex(ValueError, 'three'):
            pointing_metrics(*(x[:2] for x in args))
        time, position, attitude, rates = args
        with self.assertRaisesRegex(ValueError, 'nonzero'):
            pointing_metrics(time, position * 0, attitude, rates)
        with self.assertRaisesRegex(ValueError, 'undefined'):
            pointing_metrics(time, np.tile([1., 0, 0], (5, 1)), attitude, rates)
