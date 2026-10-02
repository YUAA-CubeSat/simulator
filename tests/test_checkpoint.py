import csv
import io
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from cubesat_sim.__main__ import run
from cubesat_sim.config import load_config

ROOT = Path(__file__).resolve().parents[1]


class CheckpointTests(unittest.TestCase):
    def test_checkpoints_are_readable_before_cleanup_and_do_not_duplicate_samples(self):
        for interval, expected in ((3, [[0.6], [0.6, 1.0, 1.2], [0.6, 1.0, 1.2]]),
                                   (5, [[1.0], [1.0, 1.2]]),
                                   (0, [[1.0, 1.2]])):
            with self.subTest(interval=interval), TemporaryDirectory() as directory:
                output = Path(directory) / 'telemetry.csv'
                config = load_config(ROOT / 'config.example.yaml')
                config['simulation'].update(duration_s=2, control_period_s=0.2,
                                            output_csv=str(output), checkpoint_steps=interval)
                snapshots = []

                def sync(fd):
                    # An independent reader sees flushed CSV while the run is active.
                    with output.open() as file:
                        rows = list(csv.DictReader(file))
                    snapshots.append([round(float(row['time_s']), 8) for row in rows])
                    self.assertTrue(all(float(row['mx_Am2']) == 0.1 for row in rows))

                def step(tick, *_):
                    if tick == 6:
                        raise KeyboardInterrupt
                    return np.array([0.1, 0, 0]), 0

                with patch('cubesat_sim.__main__.SitlController') as controller, \
                        patch('cubesat_sim.__main__.os.fsync', side_effect=sync), \
                        patch('sys.stderr', io.StringIO()):
                    controller.return_value.step.side_effect = step
                    with self.assertRaises(KeyboardInterrupt):
                        run(config)
                    controller.return_value.close.assert_called_once()
                self.assertEqual(snapshots, expected)

    def test_normal_exit_syncs_tail_and_disabled_output_skips_checkpoints(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled), TemporaryDirectory() as directory:
                config = load_config(ROOT / 'config.example.yaml')
                output = Path(directory) / 'telemetry.csv'
                config['simulation'].update(duration_s=2, checkpoint_steps=3,
                                            output_csv=str(output) if enabled else None)
                with patch('cubesat_sim.__main__.SitlController') as controller, \
                        patch('cubesat_sim.__main__.os.fsync') as sync, \
                        patch('sys.stderr', io.StringIO()):
                    controller.return_value.step.return_value = (np.zeros(3), 0)
                    run(config)
                self.assertEqual(sync.call_count, 4 if enabled else 0)
                if enabled:
                    with output.open() as file:
                        times = [round(float(r['time_s']), 8) for r in csv.DictReader(file)]
                    self.assertEqual(times, [0.6, 1.0, 1.2, 1.8, 2.0])

    def test_config_default_and_validation(self):
        raw = load_config(ROOT / 'config.example.yaml')
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            for value in (None, 0, 1, 1000, -1, 1.5, True, '3'):
                with self.subTest(value=value):
                    if value is None:
                        raw['simulation'].pop('checkpoint_steps', None)
                    else:
                        raw['simulation']['checkpoint_steps'] = value
                    path.write_text(yaml.safe_dump(raw))
                    if value is None or (type(value) is int and value >= 0):
                        self.assertEqual(load_config(path)['simulation']['checkpoint_steps'],
                                         1000 if value is None else value)
                    else:
                        with self.assertRaisesRegex(ValueError, 'checkpoint_steps'):
                            load_config(path)


if __name__ == '__main__':
    unittest.main()
