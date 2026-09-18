"""Animate telemetry orbit coordinates with Earth and the Sun direction."""

from datetime import datetime
from pathlib import Path

import numpy as np
import yaml
from matplotlib.animation import FuncAnimation, PillowWriter

from cubesat_sim.sun import sun_position_eci
from utilities.common import load_telemetry, parser, pyplot, run


def main():
    cli = parser(__doc__, "plots/orbit.gif")
    cli.add_argument("--config", type=Path, default=Path("config.yaml"), help="Config used for this simulation")
    cli.add_argument("--frames", type=int, default=300, help="Maximum animation frames (default: 300)")
    cli.add_argument("--fps", type=int, default=20, help="Playback frames per second (default: 20)")
    args = cli.parse_args()
    if args.frames < 2 or args.fps < 1:
        cli.error("--frames must be at least 2 and --fps must be positive")
    if args.output.suffix.lower() not in (".gif", ".html"):
        cli.error("animation output must have a .gif or .html extension")
    time, position = load_telemetry(args.csv, ("x_m", "y_m", "z_m"))
    with args.config.open() as source:
        config = yaml.safe_load(source)
    try:
        start = config["simulation"]["start_utc"]
        start = start.isoformat() if isinstance(start, datetime) else start
        if datetime.fromisoformat(start).utcoffset() is None:
            raise ValueError("simulation.start_utc must include a timezone")
        radius = float(config["environment"]["earth_radius_m"]) / 1000
    except (KeyError, TypeError) as exc:
        raise ValueError("Config requires simulation.start_utc and environment.earth_radius_m") from exc
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("Earth radius must be finite and positive")
    position = position / 1000
    # Uniform simulation-time sampling preserves playback speed even for irregular CSV times.
    frame_time = np.linspace(time[0], time[-1], min(args.frames, len(time)))
    orbit = np.column_stack([np.interp(frame_time, time, position[:, i]) for i in range(3)])
    extent = max(radius, np.linalg.norm(position, axis=1).max())
    sun = np.asarray([sun_position_eci(start, t) for t in frame_time])
    sun = sun / np.linalg.norm(sun, axis=1)[:, None] * extent * 1.65
    plt = pyplot(args.show)
    fig = plt.figure(figsize=(8, 8), layout="constrained")
    ax = fig.add_subplot(projection="3d")
    longitude, latitude = np.meshgrid(np.linspace(0, 2 * np.pi, 48), np.linspace(0, np.pi, 24))
    ax.plot_surface(radius * np.cos(longitude) * np.sin(latitude),
                    radius * np.sin(longitude) * np.sin(latitude), radius * np.cos(latitude),
                    color="royalblue", alpha=0.65, linewidth=0)
    ax.text(0, 0, radius * 1.1, "Earth")
    # Keep a bounded overview path for large telemetry files.
    overview = position[np.unique(np.linspace(0, len(time) - 1, min(5000, len(time))).astype(int))]
    ax.plot(*overview.T, color="gray", alpha=0.4, label="Recorded orbit")
    trail, = ax.plot([], [], [], color="C1", linewidth=1.5)
    satellite, = ax.plot([], [], [], "o", color="red", markersize=6, label="Satellite")
    sun_marker, = ax.plot([], [], [], "o", color="gold", markersize=18, label="Sun (distance and size reduced)")
    sun_line, = ax.plot([], [], [], "--", color="goldenrod", alpha=0.6)
    limit = extent * 1.9
    ax.set(xlim=(-limit, limit), ylim=(-limit, limit), zlim=(-limit, limit),
           xlabel="ECI X (km)", ylabel="ECI Y (km)", zlabel="ECI Z (km)")
    ax.set_box_aspect((1, 1, 1))
    ax.legend(loc="lower left")
    title = ax.set_title("")

    def update(index):
        satellite.set_data_3d(*orbit[index:index + 1].T)
        trail.set_data_3d(*orbit[:index + 1].T)
        sun_marker.set_data_3d(*sun[index:index + 1].T)
        sun_line.set_data_3d(*np.vstack((np.zeros(3), sun[index])).T)
        title.set_text(f"CubeSat orbit — t = {frame_time[index]:.1f} s\nEpoch: {start}")
        return satellite, trail, sun_marker, sun_line, title

    animation = FuncAnimation(fig, update, frames=len(frame_time), interval=1000 / args.fps, blit=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.suffix.lower() == ".html":
        # File exports should include every frame, regardless of notebook embed limits.
        from matplotlib import rc_context
        with rc_context({"animation.embed_limit": float("inf")}):
            args.output.write_text(animation.to_jshtml(fps=args.fps, embed_frames=True), encoding="utf-8")
    else:
        animation.save(args.output, writer=PillowWriter(fps=args.fps), dpi=90)
    print(f"Saved {args.output}")
    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    run(main)
