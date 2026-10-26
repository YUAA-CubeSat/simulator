"""Animate an orbit, adding attitude, panel illumination, and power when recorded."""

# Allow direct execution from any working directory as well as package execution.
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


import csv
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import yaml

from cubesat_sim.sun import AU_M, sun_position_eci
from utilities.common import load_telemetry, parser, pyplot, run
from utilities.plot_angular_velocity import body_to_world

COLUMNS = ("x_m", "y_m", "z_m", "sigma_BN_x", "sigma_BN_y", "sigma_BN_z",
           "solar_power_W", "battery_power_W", "sunlight_fraction")


def load_animation_telemetry(path):
    """Use the full dashboard only when its complete telemetry is available.

    Orbit-only runs have no attitude or battery/controller state. Do not fill
    those quantities with invented zeros or require spacecraft panel settings.
    """
    with path.open(newline="", encoding="utf-8-sig") as source:
        available = set(csv.DictReader(source).fieldnames or [])
    full_dashboard = set(COLUMNS).issubset(available)
    columns = COLUMNS if full_dashboard else COLUMNS[:3]
    time, data = load_telemetry(path, columns)
    return time, data, full_dashboard


def frame_indices(time, maximum, start_s=None, end_s=None):
    """Select real rows nearest uniform target times; never interpolate MRPs."""
    start = time[0] if start_s is None else start_s
    end = time[-1] if end_s is None else end_s
    if not np.isfinite([start, end]).all() or start > end:
        raise ValueError("Animation time bounds must be finite with start <= end")
    selected = np.flatnonzero((time >= start) & (time <= end))
    if not len(selected):
        raise ValueError("No telemetry samples in the requested time window")
    if len(selected) <= maximum:
        return selected, selected
    times = time[selected]
    targets = np.linspace(times[0], times[-1], min(maximum, len(times)))
    right = np.searchsorted(times, targets).clip(0, len(times) - 1)
    left = np.maximum(right - 1, 0)
    nearest = np.where(abs(times[left] - targets) <= abs(times[right] - targets), left, right)
    return selected[np.unique(nearest)], selected


def panel_parameters(config):
    panels = config["spacecraft"]["solar_charging"]["panels"]
    normals = np.array([p["normal_body"] for p in panels], dtype=float) if panels else np.empty((0, 3))
    if normals.shape != (len(panels), 3):
        raise ValueError("Each panel normal must have three components")
    areas = np.array([float(p["area_m2"]) for p in panels])
    efficiencies = np.array([float(p["efficiency"]) for p in panels])
    lengths = np.linalg.norm(normals, axis=1)
    if not np.isfinite(normals).all() or np.any(lengths == 0):
        raise ValueError("Panel normals must be finite and nonzero")
    if (not np.isfinite(areas).all() or not np.isfinite(efficiencies).all()
            or np.any(areas <= 0) or np.any(efficiencies <= 0) or np.any(efficiencies > 1)):
        raise ValueError("Panel areas must be positive and finite; efficiencies must be in (0, 1]")
    return normals / lengths[:, None], areas * efficiencies


def frame_geometry(start, time, data, normals, area_efficiency, solar):
    """Vectors in J2000 ECI; panel estimates at the recorded state timestamp."""
    position, sigma = data[:3], data[3:6]
    sun = sun_position_eci(start, time)
    to_sun = sun - position  # Use the real Sun distance, never a display marker.
    distance = np.linalg.norm(to_sun)
    sun_direction = to_sun / distance
    axes = body_to_world(np.eye(3), sigma)
    panel_normals = body_to_world(normals, sigma)
    incidence = np.maximum(0, panel_normals @ sun_direction)
    flux = float(solar["irradiance_W_m2_at_1au"]) * (AU_M / distance) ** 2
    panel_power = (flux * float(solar["conversion_efficiency"]) * data[8]
                   * incidence * area_efficiency)
    return sun_direction, axes, panel_normals, panel_power


def main(argv=None, render=None):
    cli = parser(__doc__, "plots/orbit.gif")
    cli.add_argument("--config", type=Path, default=Path("config.yaml"), help="Config used for this simulation")
    cli.add_argument("--frames", type=int, default=300, help="Maximum animation frames (default: 300)")
    cli.add_argument("--fps", type=int, default=20, help="Playback frames per second (default: 20)")
    cli.add_argument("--start-s", type=float, help="First simulation time to include")
    cli.add_argument("--end-s", type=float, help="Last simulation time to include")
    args = cli.parse_args(argv)
    if args.frames < 2 or args.fps < 1:
        cli.error("--frames must be at least 2 and --fps must be positive")
    if args.output.suffix.lower() not in (".gif", ".html"):
        cli.error("animation output must have a .gif or .html extension")
    time, data, full_dashboard = load_animation_telemetry(args.csv)
    with args.config.open() as source:
        config = yaml.safe_load(source)
    try:
        start = config["simulation"]["start_utc"]
        start = start.isoformat() if isinstance(start, datetime) else start
        epoch = datetime.fromisoformat(start.replace("Z", "+00:00"))
        if epoch.utcoffset() is None:
            raise ValueError("simulation.start_utc must include a timezone")
        radius = float(config["environment"]["earth_radius_m"]) / 1000
        if full_dashboard:
            solar = config["spacecraft"]["solar_charging"]
            normals, area_efficiency = panel_parameters(config)
            for key in ("irradiance_W_m2_at_1au", "conversion_efficiency"):
                if not np.isfinite(float(solar[key])) or float(solar[key]) <= 0:
                    raise ValueError(f"solar_charging.{key} must be finite and positive")
            if float(solar["conversion_efficiency"]) > 1:
                raise ValueError("solar_charging.conversion_efficiency must be at most 1")
    except (KeyError, TypeError, AttributeError) as exc:
        required = "simulation.start_utc and environment.earth_radius_m"
        if full_dashboard:
            required += ", plus spacecraft.solar_charging for the full dashboard"
        raise ValueError(f"Config requires {required}") from exc
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("Earth radius must be finite and positive")
    if full_dashboard and np.any((data[:, 8] < 0) | (data[:, 8] > 1)):
        raise ValueError("Recorded sunlight_fraction must be between 0 and 1")
    frames, selected = frame_indices(time, args.frames, args.start_s, args.end_s)
    if full_dashboard:
        geometry = [frame_geometry(start, time[i], data[i], normals, area_efficiency, solar) for i in frames]
    else:
        sun_vectors = np.array([sun_position_eci(start, time[i]) - data[i, :3] for i in frames])
        sun_directions = sun_vectors / np.linalg.norm(sun_vectors, axis=1)[:, None]
    position = data[:, :3] / 1000
    extent = max(radius, np.linalg.norm(position[selected], axis=1).max())
    plt = pyplot(args.show)
    from matplotlib.animation import FuncAnimation, PillowWriter
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(14, 10) if full_dashboard else (9, 9), layout="constrained")
    if full_dashboard:
        grid = fig.add_gridspec(2, 2, height_ratios=(2, 1))
        orbit_ax = fig.add_subplot(grid[0, 0], projection="3d")
        attitude_ax = fig.add_subplot(grid[0, 1], projection="3d")
        power_ax = fig.add_subplot(grid[1, 0])
        panels_ax = fig.add_subplot(grid[1, 1])
    else:
        orbit_ax = fig.add_subplot(111, projection="3d")
    longitude, latitude = np.meshgrid(np.linspace(0, 2 * np.pi, 40), np.linspace(0, np.pi, 20))
    orbit_ax.plot_surface(radius * np.cos(longitude) * np.sin(latitude),
                          radius * np.sin(longitude) * np.sin(latitude), radius * np.cos(latitude),
                          color="royalblue", alpha=0.3, linewidth=0)
    # Draw the path from recorded rows, not chords between animation frames.
    overview = selected[np.unique(np.linspace(0, len(selected) - 1, min(5000, len(selected))).astype(int))]
    orbit_ax.plot(*position[overview].T, color="gray", alpha=0.5)
    satellite, = orbit_ax.plot([], [], [], "o", color="red", markersize=7, label="Satellite")
    sun_ray, = orbit_ax.plot([], [], [], color="goldenrod", linewidth=2, label="Toward Sun (length reduced)")
    if full_dashboard:
        orbit_axes = [orbit_ax.plot([], [], [], color=c, linewidth=2)[0] for c in ("C3", "C2", "C0")]
    limit = extent * 1.6
    orbit_ax.set(xlim=(-limit, limit), ylim=(-limit, limit), zlim=(-limit, limit),
                 xlabel="ECI X (km)", ylabel="ECI Y (km)", zlabel="ECI Z (km)",
                 title="Orbit in J2000 ECI · Earth to scale")
    orbit_ax.set_box_aspect((1, 1, 1))
    orbit_ax.legend(loc="lower left", fontsize=8)

    if full_dashboard:
        # Configs define panel normals/areas, not physical mounting positions.
        vertices = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
        faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
        body = Poly3DCollection([], facecolor="slategray", edgecolor="black", alpha=0.15)
        attitude_ax.add_collection3d(body)
        body_axes = [attitude_ax.plot([], [], [], color=c, linewidth=2, label=f"Body {name}")[0]
                     for c, name in zip(("C3", "C2", "C0"), "XYZ")]
        labels = [attitude_ax.text(0, 0, 0, name, color=c) for name, c in zip("XYZ", ("C3", "C2", "C0"))]
        local_sun, = attitude_ax.plot([], [], [], color="goldenrod", marker="o", markevery=[-1],
                                      linewidth=3, label="Toward Sun")
        panel_lines = [attitude_ax.plot([], [], [], color="gold", marker="o", markersize=4)[0] for _ in normals]
        panel_labels = [attitude_ax.text(0, 0, 0, f"P{i + 1}", fontsize=8) for i in range(len(normals))]
        attitude_ax.set(xlim=(-1.8, 1.8), ylim=(-1.8, 1.8), zlim=(-1.8, 1.8),
                        xlabel="ECI X", ylabel="ECI Y", zlabel="ECI Z",
                        title="Attitude · schematic body and panel normals\nFixed ECI view; not to scale")
        attitude_ax.set_box_aspect((1, 1, 1))
        attitude_ax.legend(loc="upper left", fontsize=8)

        power_ax.plot(time[selected], data[selected, 6], color="goldenrod", label="Solar bus power (recorded)", linewidth=0.8)
        power_ax.plot(time[selected], data[selected, 7], color="C2", label="Net battery power (recorded)", linewidth=0.8)
        cursor = power_ax.axvline(time[frames[0]], color="red", linewidth=1)
        power_ax.axhline(0, color="gray", linewidth=0.5)
        power_ax.set(xlabel="Simulation time (s)", ylabel="Power (W)", title="Recorded power · charging (+), discharging (−)")
        power_ax.grid(alpha=0.3)
        power_ax.legend(fontsize=8, loc="upper right")
        bars = panels_ax.bar(range(len(normals)), np.zeros(len(normals)), color="goldenrod")
        normal_labels = [", ".join(f"{x:g}" for x in n) for n in normals]
        panels_ax.set_xticks(range(len(normals)), [f"P{i + 1}\n({n})" for i, n in enumerate(normal_labels)], fontsize=8)
        panel_max = max((g[3].max(initial=0) for g in geometry), default=0)
        panels_ax.set(ylim=(0, max(0.1, panel_max * 1.3)), ylabel="Estimated bus power (W)",
                      xlabel="Panel / outward body normal", title="Panel geometry at this timestamp")
        panels_ax.grid(axis="y", alpha=0.3)
        if not len(normals):
            panels_ax.text(0.5, 0.5, "No configured solar panels", transform=panels_ax.transAxes, ha="center")
        status = panels_ax.text(0.02, 0.97, "", transform=panels_ax.transAxes, va="top", fontsize=9)
    title = fig.suptitle("")

    def update(index):
        row = frames[index]
        if full_dashboard:
            sun_direction, axes, panel_normals, panel_power = geometry[index]
        else:
            sun_direction = sun_directions[index]
        p = position[row]
        satellite.set_data_3d(*p[:, None])
        sun_ray.set_data_3d(*np.array([p, p + sun_direction * extent * 0.5]).T)
        if full_dashboard:
            for line, axis in zip(orbit_axes, axes):
                line.set_data_3d(*np.array([p, p + axis * extent * 0.12]).T)
            body.set_verts([body_to_world(vertices[face], data[row, 3:6]) for face in faces])
            for line, label, axis in zip(body_axes, labels, axes):
                line.set_data_3d(*np.array([np.zeros(3), axis]).T)
                label.set_position_3d(axis * 1.1)
            local_sun.set_data_3d(*np.array([np.zeros(3), sun_direction * 1.6]).T)
            for line, label, normal, watts in zip(panel_lines, panel_labels, panel_normals, panel_power):
                line.set_data_3d(*np.array([normal * 0.55, normal * 1.3]).T)
                line.set_color("goldenrod" if watts > 0 else "gray")
                label.set_position_3d(normal * 1.4)
            for bar, watts in zip(bars, panel_power):
                bar.set_height(watts)
            cursor.set_xdata([time[row], time[row]])
            illumination = data[row, 8]
            shadow = "Umbra" if illumination == 0 else "Sunlit" if illumination == 1 else "Penumbra"
            status.set_text(f"{shadow} · visible Sun {100 * illumination:.1f}%\n"
                            f"Solar: {data[row, 6]:.3f} W recorded / {panel_power.sum():.3f} W geometry")
        stamp = epoch + timedelta(seconds=float(time[row]))
        title.set_text(f"t = {time[row]:.3f} s · {stamp.isoformat()}\n"
                       f"Recorded row {row + 1} · sampled playback ({len(frames)} frames)")

    if render is not None:
        update(len(frames) - 1)
        try:
            return render(fig)
        finally:
            plt.close(fig)

    animation = FuncAnimation(fig, update, frames=len(frames), interval=1000 / args.fps, blit=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.suffix.lower() == ".html":
        from matplotlib import rc_context
        with rc_context({"animation.embed_limit": float("inf")}):
            args.output.write_text(animation.to_jshtml(fps=args.fps, embed_frames=True), encoding="utf-8")
    else:
        animation.save(args.output, writer=PillowWriter(fps=args.fps), dpi=90)
    print(f"Saved {args.output} ({len(frames)} frames, t={time[frames[0]]:g}–{time[frames[-1]]:g} s)")
    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    run(main)
