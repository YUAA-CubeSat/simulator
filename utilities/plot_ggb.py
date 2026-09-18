"""Plot gravity gradient boom (GGB) extension over time."""

from utilities.common import load_telemetry, parser, pyplot, run, save_plot


def main():
    args = parser(__doc__, "plots/ggb_extension.png").parse_args()
    time, length = load_telemetry(args.csv, ("boom_length_m",))
    plt = pyplot(args.show)
    fig, ax = plt.subplots(figsize=(10, 4), layout="constrained")
    ax.plot(time, length[:, 0])
    ax.set(title="Gravity gradient boom extension", xlabel="Simulation time (s)", ylabel="Extension (m)")
    ax.grid(alpha=0.3)
    save_plot(fig, plt, args)


if __name__ == "__main__":
    run(main)
