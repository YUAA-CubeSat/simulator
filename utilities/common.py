"""CSV loading and command line options shared by the plotting scripts."""

import argparse
import csv
from pathlib import Path

import numpy as np


def parser(description, output):
    result = argparse.ArgumentParser(description=description)
    result.add_argument("csv", nargs="?", type=Path, default=Path("telemetry.csv"))
    result.add_argument("-o", "--output", type=Path, default=Path(output))
    result.add_argument("--show", action="store_true", help="Also open an interactive window")
    return result


def load_telemetry(path, columns):
    names = ("time_s", *columns)
    with path.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        missing = set(names) - set(reader.fieldnames or [])
        if missing:
            raise ValueError("Missing telemetry columns: " + ", ".join(sorted(missing)))
        rows = []
        for line, row in enumerate(reader, 2):
            try:
                rows.append([float(row[name]) for name in names])
            except (ValueError, TypeError) as exc:
                raise ValueError(f"Invalid telemetry value on line {line}") from exc
    if not rows:
        raise ValueError("Telemetry contains no samples")
    data = np.asarray(rows)
    if not np.isfinite(data).all():
        raise ValueError("Telemetry contains nonfinite values")
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError("Telemetry times must be strictly increasing")
    return data[:, 0], data[:, 1:]


def pyplot(show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def save_plot(fig, plt, args):
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print(f"Saved {args.output}")
    if args.show:
        plt.show()
    plt.close(fig)


def run(main):
    try:
        main()
    except (OSError, ValueError) as exc:
        raise SystemExit(f"Error: {exc}") from exc
