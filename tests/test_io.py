import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from cubesat_sim.config import load_config
from cubesat_sim.io import SitlController

ROOT = Path(__file__).resolve().parents[1]


class SitlTests(unittest.TestCase):
    def controller(self, script, timeout=1, startup_delay=0):
        controller = SitlController({"command": [sys.executable, "-u", "-c", script], "timeout_s": timeout, "startup_delay_s": startup_delay})
        self.addCleanup(controller.close)
        return controller

    def test_round_trip_and_partial_reply(self):
        controller = self.controller('''
import sys, time
assert input() == 'M 1000 0 10000 0 -10000'
assert input() == 'M 1000 1 0 20000 0'
assert input() == 'C 1000 0 1000 -500 250'
assert input() == 'C 1000 1 -400 900 -100'
assert input() == 'S 1000 1 32767 0 0'
assert input() == 'S 1000 0 0 0 0'
assert input() == 'B 1000 800 -1250'
assert input() == 'B 1125 1000 2500'
assert input() == 'B 1250 0 0'
assert input() == 'T 7 1000 1000'
sys.stdout.write('A 7 100000 ')
sys.stdout.flush()
time.sleep(0.02)
print('0 -100000 400', flush=True)
assert input() == 'T 8 2000 1000'
print('A 8 0 0 0 0', flush=True)
''')
        controller.magnetometers(1, [1e-5, 0, -1e-5], [0, 2e-5, 0])
        controller.accelerometers(1, [0.001, -0.0005, 0.00025], [-0.0004, 0.0009, -0.0001])
        controller.sun(1, [1, 0, 0])
        controller.sun(1, None)
        controller.battery(1, 0.8, -1.25)
        controller.battery(1.125, 1, 2.5)
        controller.battery(1.25, 0, 0)
        dipole, boom = controller.step(7, 1, 1)
        np.testing.assert_allclose(dipole, [0.1, 0, -0.1])
        self.assertEqual(boom, 0.4)
        np.testing.assert_array_equal(controller.step(8, 2, 1)[0], [0, 0, 0])
        controller.close()
        self.assertEqual(controller.process.returncode, 0)

    def test_all_magnetometers_have_distinct_ids(self):
        controller = self.controller("""
assert input() == 'M 125 0 1 -2 3'
assert input() == 'M 125 1 4 5 -6'
assert input() == 'M 125 2 -7 8 9'
assert input() == 'T 0 125 1000'
print('A 0 0 0 0 0', flush=True)
""")
        controller.magnetometers(0.125, [1e-9, -2e-9, 3e-9],
                                 [4e-9, 5e-9, -6e-9], [-7e-9, 8e-9, 9e-9])
        controller.step(0, 0.125, 1)
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

    def test_boot_delay_precedes_traffic_and_preserves_response_timeout(self):
        controller = self.controller("import select, sys, time\nassert not select.select([sys.stdin], [], [], 0.2)[0], 'traffic before boot'\nassert input() == 'T 0 0 1000'\nprint('A 0 0 0 0 0', flush=True)\ninput()\ntime.sleep(60)\n", timeout=0.1, startup_delay=0.5)
        np.testing.assert_array_equal(controller.step(0, 0, 1)[0], [0, 0, 0])
        with self.assertRaises(TimeoutError):
            controller.step(1, 1, 1)

    def test_exit_during_boot_fails_promptly(self):
        start = time.monotonic()
        with self.assertRaisesRegex(ConnectionError, 'startup with status 3'):
            self.controller('raise SystemExit(3)', startup_delay=10)
        self.assertLess(time.monotonic() - start, 2)

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
            config['sitl']['startup_delay_s'] = 0
            controller = SitlController(config['sitl'])
            try:
                controller.magnetometers(0, [0, 0, 0], [0, 0, 0])
                controller.accelerometers(0, [0.001, -0.0005, 0], [0, 0.002, 0])
                controller.sun(0, None)
                controller.battery(0, 0.8, 0)
                np.testing.assert_array_equal(controller.step(0, 0, 1)[0], [0, 0, 0])
                controller.magnetometers(1, [1e-6, -1e-6, 0], [9e-5, 8e-5, 7e-5], [-1e-4, 0, 0])
                controller.battery(1, 0.799, -0.5)
                dipole, boom = controller.step(1, 1, 1)
                np.testing.assert_allclose(dipole, [-0.1, 0.1, 0])
                self.assertEqual(boom, 0)
            finally:
                controller.close()

    def test_invalid_configuration(self):
        for command in [None, './controller', [], [1]]:
            with self.subTest(command=command), self.assertRaisesRegex(ValueError, 'command'):
                SitlController({'command': command, 'timeout_s': 1})
        for delay in [-1, float('nan'), float('inf')]:
            with self.subTest(delay=delay), self.assertRaisesRegex(ValueError, 'startup_delay'):
                SitlController({'command': ['./controller'], 'timeout_s': 1, 'startup_delay_s': delay})
        for timeout in [0, -1, float('nan'), float('inf')]:
            with self.subTest(timeout=timeout), self.assertRaisesRegex(ValueError, 'timeout'):
                SitlController({'command': ['./controller'], 'timeout_s': timeout})


if __name__ == '__main__':
    unittest.main()
