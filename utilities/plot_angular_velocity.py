"""Plot body angular velocity components and total magnitude."""

import numpy as np

from utilities.common import load_telemetry, parser, pyplot, run, save_plot


def main():
    args = parser(__doc__, "plots/angular_velocity.png").parse_args()
    time, rates = load_telemetry(args.csv, ("wx_rad_s", "wy_rad_s", "wz_rad_s"))
    values = np.column_stack((rates, np.linalg.norm(rates, axis=1)))
    plt = pyplot(args.show)
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10), layout="constrained")
    for i, (ax, label) in enumerate(zip(axes, ("X / roll", "Y / pitch", "Z / yaw", "Magnitude"))):
        ax.plot(time, values[:, i], color=f"C{i}")
        ax.set_ylabel(f"{label}\n(rad/s)")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle("Angular velocity (body frame)")
    save_plot(fig, plt, args)


if __name__ == "__main__":
    run(main)
