import unittest

import numpy as np

from cubesat_sim.frames import precession_dcm
from cubesat_sim.sun import sun_position_eci


class SunTests(unittest.TestCase):
    def test_solar_direction_is_in_j2000_not_equator_of_date(self):
        # USNO approximate solar coordinates evaluated at 2026-09-18 00:00 UTC.
        # https://aa.usno.navy.mil/faq/sun_approx
        # Its updated coefficients differ slightly from our older Almanac set.
        of_date_reference = np.array([-0.99634803, 0.07834144, 0.03395899])
        sun = sun_position_eci("2026-09-18T00:00:00+00:00", 0)
        of_date = precession_dcm(9756.5)[0] @ (sun / np.linalg.norm(sun))
        np.testing.assert_allclose(of_date, of_date_reference, atol=4e-5, rtol=0)
        # Missing precession formerly introduced about 0.373 degrees of error.
        self.assertGreater(np.linalg.norm(sun / np.linalg.norm(sun) - of_date_reference), 0.006)

    def test_elapsed_time_and_offset_represent_same_epoch(self):
        a = sun_position_eci("2026-09-18T00:00:00+00:00", 21600)
        b = sun_position_eci("2026-09-18T02:00:00-04:00", 0)
        np.testing.assert_allclose(a, b, atol=1e-5, rtol=0)


if __name__ == "__main__":
    unittest.main()
