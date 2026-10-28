import json
import csv
import math
from pathlib import Path
import tempfile
import unittest
from utilities.live_dashboard import CATALOG, Dashboard, TelemetryTail


class LiveDashboardTests(unittest.TestCase):
    def test_partial_rows_bounded_history_and_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'live.csv'
            tail = TelemetryTail(path, 3)
            self.assertIn('Waiting', tail.poll())
            path.write_text('time_s,boom_length_m\n0,0\n1,')
            tail.poll()
            self.assertEqual(len(tail.rows), 1)
            with path.open('a') as stream:
                stream.write('1\n2,2\n3,3\n')
            tail.poll()
            self.assertEqual([r[0] for r in tail.rows], ['1', '2', '3'])
            version = tail.version
            tail.poll()
            self.assertEqual(tail.version, version)
            path.write_text('time_s,boom_length_m\n0,9\n')
            tail.poll()
            self.assertEqual(list(tail.rows), [['0', '9']])

    def test_render_cache_append_and_unavailable_panels(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'live.csv'
            path.write_text('time_s,boom_length_m\n0,0\n1,1\n')
            dashboard = Dashboard(path)
            result = dashboard.snapshot(['boom:0', 'body:0', 'orbit:0'])
            panel = result['plots']['boom:0']
            self.assertEqual(panel['data'][0]['x'], [0, 1])
            self.assertEqual(panel['data'][0]['y'], [0, 1])
            self.assertNotIn('image', panel)
            self.assertIn('Missing telemetry columns', result['plots']['body:0']['error'])
            self.assertIn('--config', result['plots']['orbit:0']['error'])
            self.assertEqual(dashboard.snapshot(['boom:0'], {'boom:0': panel['version']})['plots'], {})
            with path.open('a') as stream:
                stream.write('2,0\n')
            self.assertEqual(dashboard.snapshot(['boom:0'])['plots']['boom:0']['data'][0]['y'], [0, 1, 0])

    def test_all_existing_utility_panels_render(self):
        # Use recorded orbit geometry with full synthetic power/attitude fields.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(__file__).resolve().parents[1]
            path = Path(directory) / 'live.csv'
            rows = [dict(time_s=i, x_m=7e6 * math.cos(i / 900),
                         y_m=7e6 * math.sin(i / 900), z_m=0, solar_power_W=2,
                         sunlight_fraction=1, solar_energy_Wh=i / 1800,
                         orbit_period_s=1800 * math.pi) for i in range(20)]
            fields = dict(wx_rad_s=.01, wy_rad_s=.02, wz_rad_s=.03,
                          sigma_BN_x=0, sigma_BN_y=0, sigma_BN_z=0,
                          boom_length_m=1, mx_Am2=.01, my_Am2=.02, mz_Am2=.03,
                          battery_energy_Wh=8, battery_soc=.8, battery_power_W=0,
                          estimate_present=1, estimate_valid=1, estimate_time_s=0,
                          estimate_wx_rad_s=.01, estimate_wy_rad_s=.02, estimate_wz_rad_s=.03)
            for row in rows:
                row.update(fields)
                row['estimate_time_s'] = row['time_s']
            for target in (path, path.with_name('live_estimator.csv')):
                with target.open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
            result = Dashboard(path, root / 'config.example.yaml').snapshot([p['id'] for p in CATALOG])
            for item, value in result['plots'].items():
                self.assertIn('data', value, (item, value))
                self.assertIn('layout', value)
                json.dumps(value, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
