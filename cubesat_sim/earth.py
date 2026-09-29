"""Approximate J2000-to-Earth-fixed orientation for the magnetic environment.

IAU 1982 mean sidereal time and IAU 1976 precession (Lieske 1979).
UTC approximates UT1 for spin and TT for precession; nutation, polar motion,
and leap-second crossings are not modeled. No external Earth-orientation data
or SPICE kernels are required. See README.md for the accuracy assumptions.
"""

from datetime import datetime, timezone

import numpy as np
from Basilisk.architecture import messaging, sysModel

from .frames import DAY_S, CENTURY_S, _rz, precession_dcm

J2000_UTC = datetime(2000, 1, 1, 12, tzinfo=timezone.utc)


def mean_sidereal_angle(days_since_j2000):
    """Greenwich mean sidereal angle [rad] and its rate [rad/s], UT1 ≈ UTC."""
    t = days_since_j2000 / 36525.0
    # Sidereal seconds minus UT, with the J2000 noon offset. Reduce the
    # diurnal term before evaluating trig functions to retain time resolution.
    seconds = (24110.54841 - DAY_S / 2 + 8640184.812866 * t
               + 0.093104 * t**2 - 6.2e-6 * t**3
               + (days_since_j2000 % 1) * DAY_S)
    angle = (seconds * (2 * np.pi / DAY_S)) % (2 * np.pi)
    rate = (2 * np.pi / DAY_S) * (
        1 + (8640184.812866 + 2 * 0.093104 * t - 3 * 6.2e-6 * t**2) / CENTURY_S)
    return angle, rate


def earth_fixed_dcm(days_since_j2000):
    """Return C_PN and its derivative: r_Earth_fixed = C_PN @ r_J2000."""
    rotation, rotation_dot = _rz(*mean_sidereal_angle(days_since_j2000))
    precession, precession_dot = precession_dcm(days_since_j2000)
    return (rotation @ precession,
            rotation_dot @ precession + rotation @ precession_dot)


class EarthOrientation(sysModel.SysModel):
    """Publish the Earth-centered planet state at the current field-task time."""

    def __init__(self, start_utc):
        super().__init__()
        self.ModelTag = "EarthOrientation"
        epoch = datetime.fromisoformat(start_utc.replace("Z", "+00:00"))
        if epoch.tzinfo is None:
            raise ValueError("Earth orientation epoch must include a UTC offset")
        self._epoch_days = (epoch - J2000_UTC).total_seconds() / DAY_S
        self.planet_state_out = messaging.SpicePlanetStateMsg()

    def UpdateState(self, current_sim_ns):
        days = self._epoch_days + current_sim_ns / (1e9 * DAY_S)
        dcm, derivative = earth_fixed_dcm(days)
        payload = messaging.SpicePlanetStateMsgPayload()
        # Position and velocity remain zero in the Earth-centered inertial frame.
        payload.PlanetName = "earth"
        payload.computeOrient = True
        payload.J20002Pfix = dcm.tolist()
        payload.J20002Pfix_dot = derivative.tolist()
        self.planet_state_out.write(payload, current_sim_ns, self.moduleID)
