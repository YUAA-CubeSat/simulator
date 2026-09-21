import copy
import unittest
from pathlib import Path

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import magneticFieldWMM

from cubesat_sim.config import load_config
from cubesat_sim.earth import EarthOrientation, earth_fixed_dcm, mean_sidereal_angle, precession_dcm
from cubesat_sim.physics import Plant

ROOT = Path(__file__).resolve().parents[1]


class EarthOrientationTests(unittest.TestCase):
    def test_sidereal_time_and_precession_against_published_values(self):
        # Numerical reference values: ERFA t_gmst82 and t_pmat76, respectively.
        # https://github.com/liberfa/erfa/blob/master/src/t_erfa_c.c
        angle, _ = mean_sidereal_angle(53736.0 - 51544.5)
        self.assertAlmostEqual(angle, 1.754174981860675096, delta=1e-12)
        expected = np.array([
            [0.9999995504328350733, 0.0008696632209480960785, 0.0003779153474959888345],
            [-0.0008696632209485112192, 0.9999996218428560614, -0.0000001643284776111886407],
            [-0.0003779153474950335077, -0.0000001643306746147366896, 0.9999999285899790119],
        ])
        actual, _ = precession_dcm(50123.9999 - 51544.5)
        np.testing.assert_allclose(actual, expected, atol=1e-14, rtol=0)

    def test_rotation_direction_phase_and_derivative(self):
        # At J2000, precession is identity and Greenwich is at 280.460618375 deg.
        theta = np.deg2rad(280.460618375)
        expected = np.array([[np.cos(theta), np.sin(theta), 0],
                             [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
        np.testing.assert_allclose(earth_fixed_dcm(0)[0], expected, atol=1e-11, rtol=0)
        start_days = 9756.5
        first, _ = earth_fixed_dcm(start_days)
        later, derivative = earth_fixed_dcm(start_days + 0.25)
        relative = later @ first.T
        # A fixed inertial point moves west in Earth-fixed coordinates.
        angle = np.arctan2(-relative[1, 0], relative[0, 0])
        self.assertAlmostEqual(np.rad2deg(angle), 90.2464, delta=0.0001)
        np.testing.assert_allclose(later @ later.T, np.eye(3), atol=1e-14, rtol=0)
        self.assertAlmostEqual(np.linalg.det(later), 1.0, places=14)
        plus = earth_fixed_dcm(start_days + 0.25 + 0.5/86400)[0]
        minus = earth_fixed_dcm(start_days + 0.25 - 0.5/86400)[0]
        np.testing.assert_allclose(derivative, plus-minus, atol=2e-11, rtol=0)

    def test_epoch_offset_and_elapsed_time_agree(self):
        a = EarthOrientation("2026-09-18T00:00:00Z")
        b = EarthOrientation("2026-09-17T20:00:00-04:00")
        c = EarthOrientation("2026-09-18T06:00:00Z")
        a.UpdateState(21_600_000_000_000)
        b.UpdateState(21_600_000_000_000)
        c.UpdateState(0)
        for other in (b, c):
            np.testing.assert_allclose(a.planet_state_out.read().J20002Pfix,
                                       other.planet_state_out.read().J20002Pfix, atol=1e-13, rtol=0)

    def test_wmm_receives_current_rotation_at_each_field_tick(self):
        config = load_config(ROOT / "config.example.yaml")
        config["simulation"]["field_period_s"] = 1.0
        plant = Plant(copy.deepcopy(config))
        self.assertTrue(plant.wmm.planetPosInMsg.isLinked())
        # Independent WMM evaluation in Earth-fixed coordinates. Its default
        # identity orientation is intentional for this reference calculation.
        reference = magneticFieldWMM.MagneticFieldWMM()
        reference.configureWMMFile(config["environment"]["wmm_coefficients"])
        state_msg = messaging.SCStatesMsg()
        reference.addSpacecraftToModel(state_msg)
        reference.epochInMsg.subscribeTo(plant.epoch_msg)
        reference.Reset(0)
        previous_field = None
        for tick in range(11):
            stamp = (tick // 5) * 1_000_000_000
            self.assertEqual(plant.wmm.planetPosInMsg.timeWritten(), stamp)
            if tick % 5 == 0:
                rotation = np.asarray(plant.earth_orientation.planet_state_out.read().J20002Pfix)
                state = messaging.SCStatesMsgPayload()
                state.r_BN_N = (rotation @ plant.state()["position_m"]).tolist()
                state_msg.write(state, stamp)
                reference.UpdateState(stamp)
                fixed_field = np.asarray(reference.envOutMsgs[0].read().magField_N)
                np.testing.assert_allclose(plant.magnetic_field_n(), rotation.T @ fixed_field,
                                           atol=1e-15, rtol=0)
                if previous_field is not None:
                    self.assertGreater(np.linalg.norm(plant.magnetic_field_n()-previous_field), 1e-10)
            else:
                np.testing.assert_array_equal(plant.magnetic_field_n(), previous_field)
            previous_field = plant.magnetic_field_n().copy()
            if tick < 10:
                plant.step([0, 0, 0], 0)

    def test_wmm_rotates_both_position_and_field_after_six_hours(self):
        config = load_config(ROOT / "config.example.yaml")
        plant = Plant(config)
        reference = magneticFieldWMM.MagneticFieldWMM()
        reference.configureWMMFile(config["environment"]["wmm_coefficients"])
        fixed_msg = messaging.SCStatesMsg()
        reference.addSpacecraftToModel(fixed_msg)
        reference.epochInMsg.subscribeTo(plant.epoch_msg)
        reference.Reset(0)
        # Substitute a controlled spacecraft message into WMM, keeping the same
        # geographic location while the corresponding inertial location rotates.
        moving_msg = messaging.SCStatesMsg()
        plant.wmm.scStateInMsgs[0].subscribeTo(moving_msg)
        fixed_position = np.array([6.8e6, 1.2e6, 0.5e6])
        fields = []
        for seconds in (0, 21600):
            stamp = seconds * 1_000_000_000
            plant.earth_orientation.UpdateState(stamp)
            rotation = np.asarray(plant.earth_orientation.planet_state_out.read().J20002Pfix)
            state = messaging.SCStatesMsgPayload()
            state.r_BN_N = (rotation.T @ fixed_position).tolist()
            moving_msg.write(state, stamp)
            state.r_BN_N = fixed_position.tolist()
            fixed_msg.write(state, stamp)
            plant.wmm.UpdateState(stamp)
            reference.UpdateState(stamp)
            inertial_field = plant.magnetic_field_n()
            fixed_field = np.asarray(reference.envOutMsgs[0].read().magField_N)
            np.testing.assert_allclose(rotation @ inertial_field, fixed_field, atol=1e-15, rtol=0)
            fields.append(inertial_field)
        self.assertGreater(np.linalg.norm(fields[1]-fields[0]), 1e-6)


if __name__ == "__main__":
    unittest.main()
