"""Pure NumPy J2000/mean-of-date frame rotations (IAU 1976 precession)."""

import numpy as np

DAY_S = 86400.0
CENTURY_S = 36525.0 * DAY_S


def _rz(angle, rate):
    c, s = np.cos(angle), np.sin(angle)
    return (np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]),
            rate * np.array([[-s, c, 0], [-c, -s, 0], [0, 0, 0]]))


def _ry(angle, rate):
    c, s = np.cos(angle), np.sin(angle)
    return (np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]]),
            rate * np.array([[-s, 0, -c], [0, 0, 0], [c, 0, -s]]))


def precession_dcm(days_since_j2000):
    """J2000 -> mean equator/equinox of date, and time derivative [1/s]."""
    t = days_since_j2000 / 36525.0
    # Rows are the zeta, z, theta polynomials, in arcseconds per century power.
    coefficients = np.array([[2306.2181, 0.30188, 0.017998],
                             [2306.2181, 1.09468, 0.018203],
                             [2004.3109, -0.42665, -0.041833]])
    angles = np.deg2rad(coefficients @ np.array([t, t**2, t**3]) / 3600)
    rates = np.deg2rad(coefficients @ np.array([1, 2*t, 3*t**2]) / 3600) / CENTURY_S
    a, da = _rz(-angles[1], -rates[1])
    b, db = _ry(angles[2], rates[2])
    c, dc = _rz(-angles[0], -rates[0])
    return a @ b @ c, da @ b @ c + a @ db @ c + a @ b @ dc


