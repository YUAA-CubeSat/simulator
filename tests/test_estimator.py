import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from cubesat_sim.estimator_log import EstimatorLog
from utilities.plot_estimator import load_comparison


class EstimatorTests(unittest.TestCase):
    def test_epoch_alignment_invalid_and_held_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            log = EstimatorLog(Path(directory) / 'telemetry.csv')
            plant = SimpleNamespace(time_s=0)
            plant.state = lambda: {'angular_rate_rad_s': [plant.time_s, 0, -plant.time_s],
                                   'attitude_mrp': [0.1, 0.2, 0.3]}
            for t, estimate in [(0, None), (1, (0.5, 1, 0.6, 0, -0.5)),
                                (2, (0.5, 1, 0.6, 0, -0.5)), (3, (3, 0, 99, 99, 99)),
                                (4, (5, 1, 99, 99, 99))]:
                plant.time_s = t
                log.record(plant, t, estimate)
            log.close()
            time, state, epoch, estimates, truth = load_comparison(log.path)
            np.testing.assert_array_equal(time, [0, 1, 2, 3, 4])
            np.testing.assert_array_equal(epoch, [0.5])
            np.testing.assert_allclose(truth, [[0.5, 0, -0.5]])
            np.testing.assert_allclose(estimates - truth, [[0.1, 0, 0]])
            np.testing.assert_allclose(state[0, 3:], [0.1, 0.2, 0.3])

    def test_no_estimates(self):
        with tempfile.TemporaryDirectory() as directory:
            log = EstimatorLog(Path(directory) / 'legacy.csv')
            plant = SimpleNamespace(time_s=0, state=lambda: {
                'angular_rate_rad_s': [0, 0, 0], 'attitude_mrp': [0, 0, 0]})
            log.record(plant, 0, None)
            log.close()
            self.assertEqual(len(load_comparison(log.path)[2]), 0)
