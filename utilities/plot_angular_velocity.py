"""Plot angular velocity components and total magnitude in body or world coordinates."""

from pathlib import Path

import numpy as np

from utilities.common import load_telemetry, parser, pyplot, run, save_plot


def body_to_world(rates, attitude_mrp):
    """Rotate omega_BN_B into inertial coordinates using C_BN transpose."""
    sigma = np.asarray(attitude_mrp, dtype=float)
    rates = np.asarray(rates, dtype=float)
    squared = np.sum(sigma * sigma, axis=-1, keepdims=True)
    cross = np.cross(sigma, rates)
    return rates + (4 * (1 - squared) * cross + 8 * np.cross(sigma, cross)) / (1 + squared) ** 2


def load_rates(path, frame):
    columns = ("wx_rad_s", "wy_rad_s", "wz_rad_s")
    if frame == "world":
        columns += ("sigma_BN_x", "sigma_BN_y", "sigma_BN_z")
    try:
        time, data = load_telemetry(path, columns)
    except ValueError as exc:
        if frame == "world" and "Missing telemetry columns" in str(exc):
            raise ValueError(f"{exc}. World-frame plots require attitude telemetry; "
                             "rerun the simulation to generate it, or use --frame body.") from exc
        raise
    rates = body_to_world(data[:, :3], data[:, 3:]) if frame == "world" else data
    return time, rates


def main():
    cli = parser(__doc__, "plots/angular_velocity_body.png")
    cli.set_defaults(output=None)
    cli.add_argument("--frame", choices=("body", "world"), default="body",
                     help="Coordinate frame: body (default) or fixed inertial world (ECI)")
    args = cli.parse_args()
    if args.output is None:
        args.output = Path(f"plots/angular_velocity_{args.frame}.png")
    time, rates = load_rates(args.csv, args.frame)
    values = np.column_stack((rates, np.linalg.norm(rates, axis=1)))
    plt = pyplot(args.show)
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10), layout="constrained")
    labels = ("X / roll", "Y / pitch", "Z / yaw") if args.frame == "body" else ("X", "Y", "Z")
    for i, (ax, label) in enumerate(zip(axes, (*labels, "Magnitude"))):
        ax.plot(time, values[:, i], color=f"C{i}")
        ax.set_ylabel(f"{label}\n(rad/s)")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("Simulation time (s)")
    frame_label = "body frame" if args.frame == "body" else "world frame / ECI"
    fig.suptitle(f"Angular velocity ({frame_label})")
    save_plot(fig, plt, args)


if __name__ == "__main__":
    run(main)
