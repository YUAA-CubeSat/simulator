"""Low precision geocentric solar ephemeris in the J2000 equatorial frame."""

from datetime import datetime, timedelta
import numpy as np

AU_M = 149597870700.0


def sun_position_eci(start_utc: str, elapsed_s: float):
    utc = datetime.fromisoformat(start_utc) + timedelta(seconds=elapsed_s)
    jd = utc.timestamp() / 86400.0 + 2440587.5
    days = jd - 2451545.0
    mean_longitude = np.deg2rad((280.460 + 0.9856474 * days) % 360)
    mean_anomaly = np.deg2rad((357.528 + 0.9856003 * days) % 360)
    longitude = mean_longitude + np.deg2rad(1.915) * np.sin(mean_anomaly) + np.deg2rad(0.020) * np.sin(2 * mean_anomaly)
    obliquity = np.deg2rad(23.439 - 0.0000004 * days)
    radius = (1.00014 - 0.01671 * np.cos(mean_anomaly) - 0.00014 * np.cos(2 * mean_anomaly)) * AU_M
    return radius * np.array([np.cos(longitude), np.cos(obliquity) * np.sin(longitude), np.sin(obliquity) * np.sin(longitude)])
