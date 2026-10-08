import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from utilities.animate_orbit import COLUMNS, frame_geometry, frame_indices, main, panel_parameters, load_animation_telemetry
from cubesat_sim.sun import AU_M, sun_position_eci

ROOT = Path(__file__).resolve().parents[1]


class OrbitAnimationTests(unittest.TestCase):
    def test_frames_use_actual_rows_and_respect_window(self):
        time = np.array([1., 2., 10., 11., 50.])
        frames, selected = frame_indices(time, 3)
        np.testing.assert_array_equal(frames, [0, 3, 4])
        frames, _ = frame_indices(time, 20)
        np.testing.assert_array_equal(frames, np.arange(len(time)))
        frames, selected = frame_indices(time, 20, 2, 11)
        np.testing.assert_array_equal(frames, [1, 2, 3])
        np.testing.assert_array_equal(selected, frames)
        for start, end in ((60, 70), (10, 2), (0, float("nan"))):
            with self.assertRaises(ValueError):
                frame_indices(time, 3, start, end)

    def test_known_body_rotation_and_true_local_sun_direction(self):
        position = np.array([7e6, 0, 0])
        row = np.r_[position, [0, 0, np.tan(np.pi / 8)], [4, 1, 1]]
        normals = np.array([[1, 0, 0], [-1, 0, 0]])
        solar = {"irradiance_W_m2_at_1au": 1361, "conversion_efficiency": 0.95}
        with patch("utilities.animate_orbit.sun_position_eci", return_value=position + [0, AU_M, 0]):
            direction, axes, panel_normals, watts = frame_geometry("unused", 0, row, normals, np.array([0.003, 0.003]), solar)
        np.testing.assert_allclose(direction, [0, 1, 0], atol=1e-15)
        np.testing.assert_allclose(axes[0], [0, 1, 0], atol=1e-15)
        np.testing.assert_allclose(panel_normals, [[0, 1, 0], [0, -1, 0]], atol=1e-15)
        np.testing.assert_allclose(watts, [1361 * 0.95 * 0.003, 0], atol=1e-14)

    def test_shadow_mrp_representation_does_not_flip_attitude(self):
        row = np.array([7e6, 0, 0, 0.2, -0.3, 0.4, 0, 0, 1.])
        shadow = row.copy()
        shadow[3:6] = -row[3:6] / np.dot(row[3:6], row[3:6])
        solar = {"irradiance_W_m2_at_1au": 1361, "conversion_efficiency": 0.95}
        args = (np.eye(3), np.ones(3) * 0.003, solar)
        a = frame_geometry("2026-09-18T00:00:00+00:00", 100, row, *args)
        b = frame_geometry("2026-09-18T00:00:00+00:00", 100, shadow, *args)
        for first, second in zip(a, b):
            np.testing.assert_allclose(first, second, atol=1e-14)

    def test_panel_estimates_match_simulator_in_inertial_frame(self):
        from cubesat_sim.config import load_config
        from cubesat_sim.power import PowerSystem
        config = load_config(ROOT / "config.example.yaml")
        power = PowerSystem(config)
        normals, areas = panel_parameters(config)
        for time in (0, 1000, 86400):
            sun = sun_position_eci(config["simulation"]["start_utc"], time)
            for sign in (-1, 1):
                state = {"position_m": sign * 7e6 * sun / np.linalg.norm(sun),
                         "attitude_mrp": np.array([0.1, -0.2, 0.3])}
                expected, fraction = power.solar_power(state, time)
                row = np.r_[state["position_m"], state["attitude_mrp"], expected, 0, fraction]
                direction, _, _, watts = frame_geometry(config["simulation"]["start_utc"], time,
                                                        row, normals, areas, config["spacecraft"]["solar_charging"])
                np.testing.assert_allclose(direction, (sun - state["position_m"]) / np.linalg.norm(sun - state["position_m"]))
                self.assertAlmostEqual(watts.sum(), expected, places=12)

    def test_orbit_only_exports_from_solar_simulation(self):
        from cubesat_sim.config import load_config
        from cubesat_sim.orbit_solar import run
        from utilities.common import pyplot
        plt = pyplot(False)
        with TemporaryDirectory() as directory:
            path = Path(directory)
            telemetry = path / "solar.csv"
            config = load_config(ROOT / "config.orbit_solar.yaml")
            config["simulation"].update(duration_s=20, telemetry_period_s=10, output_csv=str(telemetry))
            run(config)
            for suffix in ("gif", "html"):
                output = path / f"orbit.{suffix}"
                with patch("sys.argv", ["animate_orbit", str(telemetry), "--config",
                                       str(ROOT / "config.orbit_solar.yaml"), "--frames", "2",
                                       "--start-s", "10", "--end-s", "20", "-o", str(output)]), \
                        patch("utilities.animate_orbit.panel_parameters", side_effect=AssertionError("No panels in orbit-only mode")), \
                        patch.object(plt, "close", wraps=plt.close) as close:
                    main()
                figure = close.call_args.args[0]
                self.assertEqual(len(figure.axes), 1)
                self.assertEqual(figure.axes[0].name, "3d")
                self.assertIn("t = 20.000 s", figure._suptitle.get_text())
                if suffix == "gif":
                    with Image.open(output) as im:
                        self.assertEqual(im.n_frames, 2)
                else:
                    self.assertIn("data:image/png;base64", output.read_text())

    def test_load_position_only_and_full_telemetry(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "orbit.csv"
            for columns in (COLUMNS[:3], COLUMNS[:6], COLUMNS):
                with path.open("w") as file:
                    writer = csv.writer(file)
                    writer.writerow(("time_s", *columns))
                    writer.writerow([0, 7e6, *([0] * (len(columns) - 1))])
                time, data, full = load_animation_telemetry(path)
                self.assertEqual(full, columns == COLUMNS)
                self.assertEqual(data.shape, (1, len(COLUMNS) if full else 3))
            for contents, error in (("time_s,x_m,y_m\n0,7000000,0\n", "Missing telemetry columns: z_m"),
                                    ("time_s,x_m,y_m,z_m\n0,nan,0,0\n", "nonfinite"),
                                    ("time_s,x_m,y_m,z_m\n1,7000000,0,0\n0,7000000,1,0\n", "strictly increasing")):
                path.write_text(contents)
                with self.assertRaisesRegex(ValueError, error):
                    load_animation_telemetry(path)

    def test_export_gif_and_html_with_no_panels(self):
        import yaml
        from utilities.common import pyplot
        plt = pyplot(False)
        with TemporaryDirectory() as directory:
            path = Path(directory)
            config = {"simulation": {"start_utc": "2026-09-18T00:00:00Z"},
                      "environment": {"earth_radius_m": 6378137},
                      "spacecraft": {"solar_charging": {"panels": [], "irradiance_W_m2_at_1au": 1361,
                                                        "conversion_efficiency": 0.95}}}
            (path / "config.yaml").write_text(yaml.safe_dump(config))
            with (path / "telemetry.csv").open("w") as file:
                writer = csv.writer(file)
                writer.writerow(("time_s", *COLUMNS))
                writer.writerow([1, 7e6, 0, 0, 0, 0, 0, 0, -1, 0])
                writer.writerow([2, 7e6, 7600, 0, 0, 0, 0.2, 0, -1, 0])
            for suffix in ("gif", "html"):
                output = path / f"orbit.{suffix}"
                with patch("sys.argv", ["animate_orbit", str(path / "telemetry.csv"), "--config",
                                       str(path / "config.yaml"), "--frames", "2", "-o", str(output)]), \
                        patch.object(plt, "close", wraps=plt.close) as close:
                    main()
                self.assertEqual(len(close.call_args.args[0].axes), 4)
                if suffix == "gif":
                    with Image.open(output) as im:
                        self.assertEqual(im.n_frames, 2)
                else:
                    self.assertIn("data:image/png;base64", output.read_text())


if __name__ == "__main__":
    unittest.main()
