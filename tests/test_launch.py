"""Direct CLI entry points and the viewer's deferred simulation lifecycle."""
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from cubesat_sim.__main__ import main
from utilities.live_dashboard import SimulationSession, run_simulator_viewer

ROOT = Path(__file__).resolve().parents[1]


class LaunchTests(unittest.TestCase):
    def test_direct_scripts_work_outside_repository(self):
        scripts = [ROOT / 'simulator.py', ROOT / 'cubesat_sim/__main__.py',
                   *sorted((ROOT / 'utilities').glob('plot_*.py')),
                   ROOT / 'utilities/animate_orbit.py', ROOT / 'utilities/live_dashboard.py']
        with tempfile.TemporaryDirectory() as directory:
            env = dict(os.environ)
            env.pop('PYTHONPATH', None)
            for script in scripts:
                with self.subTest(script=script.name):
                    result = subprocess.run([sys.executable, str(script), '--help'],
                                            cwd=directory, env=env, capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn('usage:', result.stdout)

    def test_live_cli_delegates_without_running_simulation(self):
        with patch('utilities.live_dashboard.run_simulator_viewer', return_value=0) as viewer, \
                patch('cubesat_sim.__main__.run') as simulate:
            self.assertEqual(main([str(ROOT / 'config.example.yaml'), '--live']), 0)
            simulate.assert_not_called()
            self.assertIs(viewer.call_args.args[2], simulate)
            self.assertEqual(viewer.call_args.args[1], ROOT / 'config.example.yaml')

    def test_start_gate_is_single_use(self):
        session = SimulationSession()
        self.assertFalse(session.start_requested.is_set())
        self.assertFalse(session.output_ready.is_set())
        self.assertTrue(session.request_start())
        self.assertFalse(session.request_start())
        session.ready()
        self.assertTrue(session.output_ready.is_set())
        session.finish()
        self.assertEqual(session.snapshot()['state'], 'completed')
        self.assertFalse(session.request_start())

    def test_opening_viewer_does_not_run_or_touch_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'telemetry.csv'
            output.write_text('previous run\n')
            session = SimulationSession()
            server = Mock(server_port=8765)
            def opened(url):
                simulate.assert_not_called()
                self.assertEqual(output.read_text(), 'previous run\n')
                self.assertEqual(session.snapshot()['state'], 'waiting')
                session.request_start()
            def start(config, on_output_ready):
                self.assertTrue(session.start_requested.is_set())
                on_output_ready()
                raise KeyboardInterrupt
            simulate = Mock(side_effect=start)
            with patch('utilities.live_dashboard.SimulationSession', return_value=session), \
                    patch('utilities.live_dashboard.make_server', return_value=server), \
                    patch('utilities.live_dashboard.webbrowser.open', side_effect=opened), \
                    patch('sys.stdout', new=io.StringIO()):
                result = run_simulator_viewer({'simulation': {'output_csv': str(output), 'mode': 'sitl'}},
                                             ROOT / 'config.example.yaml', simulate)
            self.assertEqual(result, 130)
            simulate.assert_called_once()
            server.shutdown.assert_called_once()
            server.server_close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
