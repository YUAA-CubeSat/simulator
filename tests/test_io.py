import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from cubesat_sim.config import load_config
from cubesat_sim.io import SitlController

ROOT = Path(__file__).resolve().parents[1]


class SitlTests(unittest.TestCase):
    def controller(self, script, timeout=1):
        controller = SitlController({"command": [sys.executable, "-u", "-c", script], "timeout_s": timeout})
        self.addCleanup(controller.close)
        return controller

    def test_round_trip_and_partial_reply(self):
        controller = self.controller('''
import sys, time
assert input() == 'M 1000 10000 0 -10000 0 20000 0'
assert input() == 'S 1000 1 32767 0 0'
assert input() == 'S 1000 0 0 0 0'
assert input() == 'T 7 1000 1000'
sys.stdout.write('A 7 100000 ')
sys.stdout.flush()
time.sleep(0.02)
print('0 -100000 400', flush=True)
assert input() == 'T 8 2000 1000'
print('A 8 0 0 0 0', flush=True)
''')
        controller.magnetometers(1, [1e-5, 0, -1e-5], [0, 2e-5, 0])
        controller.sun(1, [1, 0, 0])
        controller.sun(1, None)
        dipole, boom = controller.step(7, 1, 1)
        np.testing.assert_allclose(dipole, [0.1, 0, -0.1])
        self.assertEqual(boom, 0.4)
        np.testing.assert_array_equal(controller.step(8, 2, 1)[0], [0, 0, 0])
        controller.close()
        self.assertEqual(controller.process.returncode, 0)

    def test_invalid_replies(self):
        for reply in ['A 8 0 0 0 0', 'debug message', 'A 7 nope 0 0 0', 'x' * 257]:
            with self.subTest(reply=reply):
                controller = self.controller(f'input(); print({reply!r}, flush=True)')
                with self.assertRaises(ValueError):
                    controller.step(7, 0, 1)
                controller.close()

    def test_timeout_and_cleanup(self):
        controller = self.controller('import time; input(); time.sleep(60)', timeout=0.1)
        with self.assertRaises(TimeoutError):
            controller.step(0, 0, 1)
        controller.close()
        self.assertIsNotNone(controller.process.poll())
        self.assertTrue(controller.process.stdin.closed)
        self.assertTrue(controller.process.stdout.closed)

    def test_premature_exit(self):
        controller = self.controller('input()')
        with self.assertRaises(ConnectionError):
            controller.step(0, 0, 1)

    def test_example_executable_and_config_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / 'adcs_example'
            subprocess.run(['cc', '-Wall', '-Wextra', '-Werror', str(ROOT / 'examples/adcs_example.c'),
                            '-o', str(executable)], check=True)
            config = load_config(ROOT / 'config.example.yaml')
            self.assertEqual(config['sitl']['cwd'], str(ROOT))
            config['sitl']['cwd'] = directory
            controller = SitlController(config['sitl'])
            try:
                controller.magnetometers(0, [0, 0, 0], [0, 0, 0])
                controller.sun(0, None)
                np.testing.assert_array_equal(controller.step(0, 0, 1)[0], [0, 0, 0])
                controller.magnetometers(1, [1e-6, -1e-6, 0], [0, 0, 0])
                dipole, boom = controller.step(1, 1, 1)
                np.testing.assert_allclose(dipole, [-0.1, 0.1, 0])
                self.assertEqual(boom, 0)
            finally:
                controller.close()

    def test_invalid_configuration(self):
        for command in [None, './controller', [], [1]]:
            with self.subTest(command=command), self.assertRaisesRegex(ValueError, 'command'):
                SitlController({'command': command, 'timeout_s': 1})
        for timeout in [0, -1, float('nan'), float('inf')]:
            with self.subTest(timeout=timeout), self.assertRaisesRegex(ValueError, 'timeout'):
                SitlController({'command': ['./controller'], 'timeout_s': timeout})


if __name__ == '__main__':
    unittest.main()
