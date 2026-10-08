"""Plot orbit-only solar telemetry, using integrated energy for all totals."""

import numpy as np

from utilities.common import load_telemetry, parser, pyplot, run, save_plot


def energy_segments(time, energy, boundaries):
    """Split recorded energy at boundaries; edge segments remain explicit."""
    edges = np.unique(np.r_[time[0], boundaries[(boundaries > time[0]) & (boundaries < time[-1])], time[-1]])
    return edges, np.diff(np.interp(edges, time, energy))


def analyze(time, data):
    power, sunlight, energy, periods = data.T
    if len(time) < 2:
        raise ValueError("At least two telemetry samples are required")
    if np.any(power < 0) or np.any((sunlight < 0) | (sunlight > 1)) or np.any(np.diff(energy) < 0):
        raise ValueError("Solar power/energy must be nonnegative and sunlight must be in [0, 1]")
    if np.any(energy < 0) or periods[0] <= 0 or not np.allclose(periods, periods[0]):
        raise ValueError("Expected nonnegative energy and a constant positive orbital period")
    period = periods[0]
    orbit_boundaries = np.arange(np.floor(time[0] / period), np.ceil(time[-1] / period) + 1) * period
    orbit_edges, orbit_energy = energy_segments(time, energy, orbit_boundaries)
    orbit_complete = np.isclose(np.diff(orbit_edges), period, rtol=1e-7, atol=1e-6)
    # Sunrise means first positive visible solar disk after full umbra. The
    # simulator saves adjacent integration samples at every such transition.
    dawns = time[1:][(sunlight[:-1] <= 0) & (sunlight[1:] > 0)]
    cycle_edges, cycle_energy = energy_segments(time, energy, dawns)
    cycle_complete = np.isin(cycle_edges[:-1], dawns) & np.isin(cycle_edges[1:], dawns)
    return (orbit_edges, orbit_energy, orbit_complete), (cycle_edges, cycle_energy, cycle_complete)


def main():
    args = parser("Plot solar production, energy per orbit and sunrise-to-sunrise cycle", "plots/solar.png").parse_args()
    time, data = load_telemetry(args.csv, ("solar_power_W", "sunlight_fraction", "solar_energy_Wh", "orbit_period_s"))
    orbits, cycles = analyze(time, data)
    energy = data[-1, 2] - data[0, 2]
    duration = time[-1] - time[0]
    print(f"Produced in recorded interval: {energy:.6g} Wh; mean power: {energy * 3600 / duration:.6g} W")
    for name, (_, values, complete) in (("orbit", orbits), ("sun cycle", cycles)):
        if complete.any():
            print(f"Complete {name}s: {complete.sum()}; mean energy: {values[complete].mean():.6g} Wh")
        else:
            print(f"No complete {name}s in recording")
    plt = pyplot(args.show)
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained")
    hours = time / 3600
    axes[0, 0].plot(hours, data[:, 2] - data[0, 2])
    axes[0, 0].set(xlabel="Simulation time (h)", ylabel="Produced energy (Wh)", title=f"Cumulative solar energy · {energy:.3f} Wh")
    axes[0, 1].stairs(np.diff(data[:, 2]) * 3600 / np.diff(time), hours, baseline=None, color="goldenrod", label="Interval mean power")
    axes[0, 1].set(xlabel="Simulation time (h)", ylabel="Solar bus power (W)", title="Solar power and eclipse")
    light_axis = axes[0, 1].twinx()
    light_axis.plot(hours, data[:, 1], color="0.5", alpha=0.4)
    light_axis.set(ylabel="Visible Sun fraction", ylim=(0, 1.05))
    for ax, (edges, values, complete), title in ((axes[1, 0], orbits, "Energy per orbit (from simulation epoch)"),
                                                (axes[1, 1], cycles, "Energy per sun cycle (sunrise to sunrise)")):
        indices = np.arange(1, len(values) + 1)
        ax.bar(indices[complete], values[complete], color="C0", label="Complete")
        ax.bar(indices[~complete], values[~complete], color="C1", hatch="//", label="Partial / unbounded")
        ax.set(xlabel="Interval number", ylabel="Produced energy (Wh)", title=title)
        ax.legend()
    for ax in axes.flat:
        ax.grid(alpha=0.3)
    fig.suptitle("Attitude-free solar production · available bus energy before storage or loads")
    save_plot(fig, plt, args)


if __name__ == "__main__":
    run(main)
