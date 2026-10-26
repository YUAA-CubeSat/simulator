"""Plot gravity gradient boom (GGB) extension over time."""

# Allow direct execution from any working directory as well as package execution.
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


from utilities.common import load_telemetry, parser, pyplot, run, save_plot


def main(argv=None, render=None):
    args = parser(__doc__, "plots/ggb_extension.png").parse_args(argv)
    time, length = load_telemetry(args.csv, ("boom_length_m",))
    plt = pyplot(args.show)
    fig, ax = plt.subplots(figsize=(10, 4), layout="constrained")
    ax.plot(time, length[:, 0])
    ax.set(title="Gravity gradient boom extension", xlabel="Simulation time (s)", ylabel="Extension (m)")
    ax.grid(alpha=0.3)
    if render is None:
        save_plot(fig, plt, args)
    else:
        save_plot(fig, plt, args, render=render)


if __name__ == "__main__":
    run(main)
