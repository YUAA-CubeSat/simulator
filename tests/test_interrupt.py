import csv
import io
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from cubesat_sim.__main__ import run
from cubesat_sim.config import load_config
from cubesat_sim.physics import Plant

ROOT = Path(__file__).resolve().parents[1]


class InterruptTests(unittest.TestCase):
    def config(self, path):
        config = load_config(ROOT / "config.example.yaml")
        config["simulation"].update(duration_s=4, control_period_s=0.2, output_csv=str(path))
        return config

    def test_interrupt_preserves_rows_and_final_completed_step_before_cleanup(self):
        for stop_tick, expected_times in ((5, [1.0]), (6, [1.0, 1.2])):
            for cleanup_error in (False, True):
                with self.subTest(stop_tick=stop_tick, cleanup_error=cleanup_error), TemporaryDirectory() as directory:
                    path = Path(directory) / "telemetry.csv"
                    config = self.config(path)
                    def step(tick, *_):
                        if tick == stop_tick:
                            raise KeyboardInterrupt
                        return np.array([0.1, 0, 0]), 0
                    def close():
                        # Buffered data must be readable before controller cleanup.
                        with path.open() as file:
                            rows = list(csv.DictReader(file))
                        self.assertEqual([float(r["time_s"]) for r in rows], expected_times)
                        self.assertTrue(all(float(r["mx_Am2"]) == 0.1 for r in rows))
                        if cleanup_error:
                            raise RuntimeError("controller cleanup failed")
                    with patch("cubesat_sim.__main__.SitlController") as controller, patch("sys.stderr", io.StringIO()):
                        controller.return_value.step.side_effect = step
                        controller.return_value.close.side_effect = close
                        with self.assertRaises(RuntimeError if cleanup_error else KeyboardInterrupt):
                            run(config)
                        controller.return_value.close.assert_called_once()

    def test_interrupt_mid_dynamics_does_not_serialize_partial_state(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.csv"
            config = self.config(path)
            original = Plant.step
            def interrupt_step(plant, dipole, length):
                if plant.time_s == 1:
                    # Emulate an interrupt after a timestep has partly mutated state.
                    plant.time_s = 1.1
                    raise KeyboardInterrupt
                return original(plant, dipole, length)
            with patch("cubesat_sim.__main__.SitlController") as controller, \
                    patch.object(Plant, "step", interrupt_step), patch("sys.stderr", io.StringIO()):
                controller.return_value.step.return_value = (np.zeros(3), 0)
                with self.assertRaises(KeyboardInterrupt):
                    run(config)
            with path.open() as file:
                rows = list(csv.DictReader(file))
            self.assertEqual([float(r["time_s"]) for r in rows], [1])

    def test_interrupt_during_controller_startup_saves_header(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.csv"
            with patch("cubesat_sim.__main__.SitlController", side_effect=KeyboardInterrupt), patch("sys.stderr", io.StringIO()):
                with self.assertRaises(KeyboardInterrupt):
                    run(self.config(path))
            with path.open() as file:
                reader = csv.DictReader(file)
                self.assertIn("battery_energy_Wh", reader.fieldnames)
                self.assertEqual(list(reader), [])

    def test_cli_sigint_saves_telemetry_without_traceback(self):
        with TemporaryDirectory() as directory:
            path = Path(directory)
            output = path / "telemetry.csv"
            config = self.config(output)
            # A real controller subprocess signals its parent while it waits for
            # an actuator reply, after both scheduled and unscheduled steps.
            script = """
import os, signal, sys
for line in sys.stdin:
    fields = line.split()
    if fields[0] == 'T':
        if fields[1] == '6':
            os.kill(os.getppid(), signal.SIGINT)
        else:
            print('A ' + fields[1] + ' 100000 0 0 0', flush=True)
"""
            config["sitl"].update(command=[sys.executable, "-u", "-c", script], startup_delay_s=0)
            config_path = path / "config.yaml"
            config_path.write_text(yaml.safe_dump(config))
            result = subprocess.run([sys.executable, "-m", "cubesat_sim", str(config_path)],
                                    cwd=ROOT, capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 130, result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            self.assertIn("Simulation stopped (Ctrl+C).", result.stderr)
            self.assertIn(f"Telemetry saved to {output.resolve()}", result.stderr)
            with output.open() as file:
                rows = list(csv.DictReader(file))
            self.assertEqual([float(r["time_s"]) for r in rows], [1, 1.2])

    def test_interrupt_with_telemetry_disabled_still_closes_controller(self):
        config = self.config("unused.csv")
        config["simulation"]["output_csv"] = None
        with patch("cubesat_sim.__main__.SitlController") as controller, patch("sys.stderr", io.StringIO()):
            controller.return_value.step.side_effect = KeyboardInterrupt
            with self.assertRaises(KeyboardInterrupt):
                run(config)
            controller.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
