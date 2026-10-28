import unittest
import numpy as np
from utilities.live_traces import Traces, reduced
from utilities.plot_battery import smooth_interval_power


class LiveTraceTests(unittest.TestCase):
    def test_interval_power_and_smoothing_keep_physical_meaning(self):
        t = np.array([0., 1., 4.])
        energy = np.array([8., 8.-.9/3600, 8.+2.1/3600])
        traces = Traces(dict(time_s=t, battery_energy_Wh=energy, solar_energy_Wh=np.array([0, 0, 5.7/3600])))
        panel = traces.panel('battery', 1)
        np.testing.assert_allclose(panel['data'][1]['y'], [-.9, 1, 1])
        smoothed = traces.panel('battery', 2)
        centers, expected = smooth_interval_power(t, np.array([-.9, 0.]), 300)
        np.testing.assert_allclose(smoothed['data'][1]['x'], centers)
        np.testing.assert_allclose(smoothed['data'][1]['y'], expected)

    def test_estimator_matches_sensor_epochs_and_deduplicates(self):
        t = np.array([0., 1., 2.])
        columns = dict(time_s=t, estimate_present=np.ones(3), estimate_valid=np.ones(3), estimate_time_s=np.array([.5, .5, 1.5]))
        for axis in 'xyz':
            columns['w'+axis+'_rad_s'] = t * 2
            columns['sigma_BN_'+axis] = t * 0
            columns['estimate_w'+axis+'_rad_s'] = np.array([1.1, 1.1, 3.1])
        panel = Traces(columns).panel('estimator', 3)
        for trace in panel['data']:
            self.assertEqual(trace['x'], [.5, 1.5])
            np.testing.assert_allclose(trace['y'], [.1, .1])

    def test_display_reduction_preserves_spikes_and_endpoints(self):
        x = np.arange(50000)
        y = np.zeros(50000); y[12345] = 100; y[12346] = -100
        a, b = reduced(x, y)
        self.assertLessEqual(len(a), 2000)
        self.assertEqual((a[0], a[-1]), (0, 49999))
        self.assertIn(100, b); self.assertIn(-100, b)
        self.assertTrue(np.all(np.diff(a) > 0))
