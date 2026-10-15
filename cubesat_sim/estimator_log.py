"""Full-cadence truth and control-response snapshots, independent of CSV decimation."""
import csv
from pathlib import Path


class EstimatorLog:
    def __init__(self, telemetry_path):
        path = Path(telemetry_path)
        self.path = path.with_name(path.stem + '_estimator.csv')
        self.file = self.path.open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time_s', 'tick', 'wx_rad_s', 'wy_rad_s', 'wz_rad_s',
                              'sigma_BN_x', 'sigma_BN_y', 'sigma_BN_z',
                              'estimate_present', 'estimate_time_s', 'estimate_valid',
                              'estimate_wx_rad_s', 'estimate_wy_rad_s', 'estimate_wz_rad_s'])

    def record(self, plant, tick, estimate):
        state = plant.state()
        self.writer.writerow([plant.time_s, tick, *state['angular_rate_rad_s'],
                              *state['attitude_mrp'], int(estimate is not None),
                              *(estimate if estimate is not None else (0, 0, 0, 0, 0))])

    def flush(self):
        import os
        self.file.flush()
        os.fsync(self.file.fileno())

    def close(self):
        try:
            self.flush()
        finally:
            self.file.close()
