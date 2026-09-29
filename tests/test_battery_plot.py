import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import numpy as np

from utilities.plot_battery import interval_power, main, smooth_interval_power


class BatteryPlotTests(unittest.TestCase):
    def test_variable_intervals_and_energy_conservation(self):
        time = np.array([1, 2, 5, 9], dtype=float)
        energy = np.array([2, 2 - 1 / 3600, 2 + 5 / 3600, 2 + 5 / 3600])
        power = interval_power(time, energy)
        np.testing.assert_allclose(power, [-1, 2, 0], atol=1e-11)
        self.assertAlmostEqual(np.sum(power * np.diff(time)) / 3600, energy[-1] - energy[0])

    def test_pulsed_load_not_lost_when_csv_sample_lands_in_off_time(self):
        # 40% duty at 1.5 W. Both instantaneous CSV samples can be zero,
        # but energy differences must report the actual 0.6 W average debit.
        time = np.array([1, 2, 3], dtype=float)
        energy = 8 - 0.6 * time / 3600
        np.testing.assert_allclose(interval_power(time, energy), [-0.6, -0.6], atol=1e-11)

    def test_first_sample_does_not_invent_initial_energy(self):
        self.assertEqual(len(interval_power(np.array([10]), np.array([3.5]))), 0)

    def test_smoothing_weights_elapsed_time_and_preserves_sum(self):
        time = np.array([0., 1., 4.])
        power = np.array([4., -2.])
        centers, charge = smooth_interval_power(time, np.maximum(power, 0), 4)
        _, discharge = smooth_interval_power(time, np.minimum(power, 0), 4)
        _, net = smooth_interval_power(time, power, 4)
        np.testing.assert_allclose(centers, [0.5, 2.5])
        np.testing.assert_allclose(charge, [1.6, 4 / 7])
        np.testing.assert_allclose(discharge, [-1.2, -12 / 7])
        np.testing.assert_allclose(charge + discharge, net)

    def test_smoothing_constant_signal_at_edges_and_large_window(self):
        time = np.array([1., 2., 5., 9.])
        for window in (0.1, 3, 300):
            _, result = smooth_interval_power(time, np.full(3, -0.5), window)
            np.testing.assert_allclose(result, -0.5)

    def test_both_power_curves_use_the_selected_time_basis(self):
        from matplotlib.colors import to_rgba
        with TemporaryDirectory() as directory:
            csv = Path(directory) / "telemetry.csv"
            csv.write_text("time_s,battery_soc,battery_energy_Wh,solar_energy_Wh,battery_power_W,solar_power_W\n"
                           f"1,0.8,8,0,0,0\n2,0.799,{8-1/3600},{2/3600},-3,4\n")
            for mode in ("interval", "sampled"):
                def check(fig, plt, args):
                    ax = fig.axes[1]
                    if args.power_mode == "interval":
                        solar, net = ax.patches
                        np.testing.assert_allclose(solar.get_data().values, [2], atol=1e-11)
                        np.testing.assert_allclose(net.get_data().values, [-1], atol=1e-11)
                        self.assertEqual(solar.get_edgecolor(), to_rgba("goldenrod"))
                    else:
                        solar, net = ax.lines[:2]
                        np.testing.assert_allclose(solar.get_ydata(), [0, 4])
                        np.testing.assert_allclose(net.get_ydata(), [0, -3])
                        self.assertEqual(solar.get_color(), "goldenrod")
                    self.assertEqual(solar.get_label(), "Solar bus power")
                    self.assertEqual(net.get_label(), "Net battery power")
                    charge, discharge, total = fig.axes[2].lines[:3]
                    np.testing.assert_allclose(charge.get_ydata() + discharge.get_ydata(), total.get_ydata())
                    plt.close(fig)
                with patch("sys.argv", ["plot_battery", str(csv), "--power-mode", mode]), \
                        patch("utilities.plot_battery.save_plot", side_effect=check):
                    main()

    def test_plot_modes_and_single_sample(self):
        with TemporaryDirectory() as directory:
            path = Path(directory)
            csv = path / "telemetry.csv"
            for mode in ("interval", "sampled"):
                extra = ",battery_power_W,solar_power_W" if mode == "sampled" else ",solar_energy_Wh"
                value = ",0,0" if mode == "sampled" else ",0"
                csv.write_text(f"time_s,battery_soc,battery_energy_Wh{extra}\n"
                               f"1,0.8,8{value}\n2,0.799,7.99{value}\n")
                output = path / f"{mode}.png"
                with patch("sys.argv", ["plot_battery", str(csv), "--power-mode", mode, "-o", str(output)]):
                    main()
                self.assertGreater(output.stat().st_size, 0)
            def check_axes(fig, plt, args):
                self.assertEqual(fig.axes[0].get_ylabel(), "Stored energy (Wh)")
                self.assertEqual(fig.axes[3].get_ylabel(), "State of charge (%)")
                np.testing.assert_allclose(fig.axes[0].get_ylim(), [0, 10])
                np.testing.assert_allclose(fig.axes[3].get_ylim(), [0, 100])
                self.assertEqual(len(fig.axes[0].lines), 1)
                plt.close(fig)
            with patch("sys.argv", ["plot_battery", str(csv)]), \
                    patch("utilities.plot_battery.save_plot", side_effect=check_axes):
                # The sampled fixture also needs cumulative solar for interval mode.
                csv.write_text("time_s,battery_soc,battery_energy_Wh,solar_energy_Wh\n1,0.8,8,0\n2,0.799,7.99,0\n")
                main()
            csv.write_text("time_s,battery_soc,battery_energy_Wh,solar_energy_Wh\n1,0.8,8,0\n")
            with patch("sys.argv", ["plot_battery", str(csv), "-o", str(path / "single.png")]):
                main()
            self.assertTrue((path / "single.png").is_file())
            for bounds in (("2", "3"), ("2", "1"), ("nan", "3")):
                with patch("sys.argv", ["plot_battery", str(csv), "--start-s", bounds[0],
                                       "--end-s", bounds[1]]), self.assertRaises(ValueError):
                    main()
            for window in ("0", "-1", "nan", "inf"):
                with patch("sys.argv", ["plot_battery", str(csv), "--smooth-window-s", window]), self.assertRaises(ValueError):
                    main()


if __name__ == "__main__":
    unittest.main()
