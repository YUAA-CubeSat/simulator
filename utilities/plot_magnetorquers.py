"""Plot the three applied body-frame magnetorquer dipoles."""

# Allow direct execution from any working directory as well as package execution.
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


from utilities.common import load_telemetry, parser, pyplot, run, save_plot


def main(argv=None, render=None):
    args = parser(__doc__, "plots/magnetorquers.png").parse_args(argv)
    time, dipoles = load_telemetry(args.csv, ("mx_Am2", "my_Am2", "mz_Am2"))
    plt = pyplot(args.show)
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8), layout="constrained")
    for i, (ax, axis) in enumerate(zip(axes, "XYZ")):
        ax.step(time, dipoles[:, i], where="post", color=f"C{i}")
        ax.set_ylabel(f"{axis} dipole (A·m²)")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle("Applied magnetorquer outputs (body frame)")
    if render is None:
        save_plot(fig, plt, args)
    else:
        save_plot(fig, plt, args, render=render)


if __name__ == "__main__":
    run(main)
