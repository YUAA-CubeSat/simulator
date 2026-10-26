"""Plot battery charge (%), stored energy (Wh), and charging/discharging power (W)."""

# Allow direct execution from any working directory as well as package execution.
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


import numpy as np

from utilities.common import load_telemetry, parser, pyplot, run, save_plot


def interval_power(time, energy_Wh):
    """Average stored-energy rate on [time[i], time[i+1]], in watts.

    This includes every dynamics step between recorded energy samples, even
    when the instantaneous power column misses actuator pulses. The first CSV
    row has no preceding energy sample, so no earlier interval is invented.
    """
    return 3600 * np.diff(energy_Wh) / np.diff(time)


def smooth_interval_power(time, power, window_s):
    """Centered time-weighted boxcar for piecewise-constant interval power.

    Clip windows at recording boundaries and normalize by available duration;
    use cumulative integrals so irregular telemetry spacing is handled exactly.
    """
    centers = (time[:-1] + time[1:]) / 2
    if len(power) == 0:
        return centers, np.array([], dtype=float)
    left = np.maximum(time[0], centers - window_s / 2)
    right = np.minimum(time[-1], centers + window_s / 2)
    integral = np.r_[0, np.cumsum(power * np.diff(time))]
    smoothed = (np.interp(right, time, integral) - np.interp(left, time, integral)) / (right - left)
    return centers, smoothed


def main(argv=None, render=None):
    cli = parser(__doc__, "plots/battery.png")
    cli.add_argument("--power-mode", choices=("interval", "sampled"), default="interval",
                     help="interval: energy-derived mean between CSV rows (default); sampled: last dynamics-step power")
    cli.add_argument("--start-s", type=float, help="First simulation time to display")
    cli.add_argument("--end-s", type=float, help="Last simulation time to display")
    cli.add_argument("--smooth-window-s", type=float, default=300,
                     help="Centered time-weighted smoothing window in seconds (default: 300)")
    args = cli.parse_args(argv)
    if not np.isfinite(args.smooth_window_s) or args.smooth_window_s <= 0:
        raise ValueError("--smooth-window-s must be positive and finite")
    columns = ("battery_soc", "battery_energy_Wh")
    if args.power_mode == "sampled":
        columns += ("battery_power_W", "solar_power_W")
    else:
        columns += ("solar_energy_Wh",)
    time, battery = load_telemetry(args.csv, columns)
    # Smooth the complete recording before zooming so the window selection
    # doesn't change results at otherwise identical timestamps.
    net = interval_power(time, battery[:, 1])
    smooth_time, charge = smooth_interval_power(time, np.maximum(net, 0), args.smooth_window_s)
    _, discharge = smooth_interval_power(time, np.minimum(net, 0), args.smooth_window_s)
    positive_soc = battery[:, 0] > 0
    capacity = np.median(battery[positive_soc, 1] / battery[positive_soc, 0]) if positive_soc.any() else None
    start = time[0] if args.start_s is None else args.start_s
    end = time[-1] if args.end_s is None else args.end_s
    if not np.isfinite([start, end]).all() or start > end:
        raise ValueError("Plot time bounds must be finite with start <= end")
    selected = (time >= start) & (time <= end)
    if not selected.any():
        raise ValueError("No telemetry samples in the requested time window")
    time, battery = time[selected], battery[selected]
    plt = pyplot(args.show)
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 10), layout="constrained")
    axes[0].plot(time, battery[:, 1], color="C0")
    axes[0].set(ylabel="Stored energy (Wh)", title="Battery level")
    soc_axis = axes[0].twinx()
    soc_axis.set(ylabel="State of charge (%)", ylim=(0, 100))
    if capacity is not None and capacity > 0:
        # Both axes describe the same line: full capacity is exactly 100%.
        axes[0].set_ylim(0, capacity)
    else:
        # An all-empty recording does not reveal capacity. Show the known zero
        # on both axes without inventing a Wh-to-percent conversion.
        axes[0].set_ylim(0, 1)
        axes[0].set_yticks([0])
        soc_axis.set_yticks([0])
        axes[0].set_title("Battery level · empty recording; capacity unavailable")
    if args.power_mode == "interval":
        if len(time) > 1:
            axes[1].stairs(interval_power(time, battery[:, 2]), time, baseline=None,
                           color="goldenrod", linewidth=0.8, label="Solar bus power")
            axes[1].stairs(interval_power(time, battery[:, 1]), time, baseline=None,
                           color="C2", linewidth=0.8, label="Net battery power")
        else:
            axes[1].text(0.5, 0.5, "At least two energy samples are needed for interval power",
                         transform=axes[1].transAxes, ha="center")
        power_label = "mean between recorded energy samples"
    else:
        axes[1].plot(time, battery[:, 3], color="goldenrod", linewidth=0.8, label="Solar bus power")
        axes[1].plot(time, battery[:, 2], color="C2", linewidth=0.8, label="Net battery power")
        power_label = "last dynamics-step samples"
    axes[1].axhline(0, color="0.4", linewidth=0.8, linestyle="--", zorder=0)
    if len(time) > 1 or args.power_mode == "sampled":
        axes[1].legend(loc="upper right")
    axes[1].set(ylabel="Power (W)",
                title=f"Charging (+) / discharging (−) · {power_label}")
    if len(smooth_time):
        axes[2].plot(smooth_time, charge, color="goldenrod", label="Charging (+)")
        axes[2].plot(smooth_time, discharge, color="C3", label="Discharging (−)")
        axes[2].plot(smooth_time, charge + discharge, color="C2", label="Net (sum)")
        axes[2].legend(loc="upper right")
    else:
        axes[2].text(0.5, 0.5, "At least two energy samples are needed for smoothing",
                     transform=axes[2].transAxes, ha="center")
    axes[2].axhline(0, color="0.4", linewidth=0.8, linestyle="--", zorder=0)
    axes[2].set(xlabel="Simulation time (s)", ylabel="Power (W)",
                title=f"Smoothed battery power · {args.smooth_window_s:g} s centered mean")
    # The smoothed curves retain their full-recording context; only the view
    # limits change for --start-s/--end-s.
    if time[-1] > time[0]:
        axes[2].set_xlim(time[0], time[-1])
    else:
        axes[2].set_xlim(time[0] - 0.5, time[0] + 0.5)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("Battery level and power")
    if render is None:
        save_plot(fig, plt, args)
    else:
        save_plot(fig, plt, args, render=render)


if __name__ == "__main__":
    run(main)
