#!/usr/bin/env python3
"""Plot a twin_sim CSV run.

Reads the CSV that tools/twin_sim writes (engine + thermal time-series,
plus the optional noisy sensor channels) and draws it as a stacked
multi-panel figure with a plain metadata header. Columns are matched by
header name, so extra columns added later are ignored rather than shifting
the layout. --y overrides the layout with an explicit column list.

Examples:
    python tools/plot_run.py runs/rt.csv
    python tools/plot_run.py -                       # read CSV from stdin
    python tools/plot_run.py runs/rt.csv --y cht_c egt_c
    python tools/plot_run.py --out runs/rt.png --sim -p rapid-throttle --sensor

`--sim` consumes every argument after it and forwards them to twin_sim, so
put any plot_run options before it.

Needs matplotlib:  pip install -r tools/requirements.txt
"""
from __future__ import annotations

import argparse
import csv
import io
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Panel layout: (ylabel, [(column, "left"|"right"), ...]). A panel is drawn
# only if at least one of its columns is present in the CSV.
PANEL_SPECS = [
    ("speed / command", [("rpm", "left"), ("throttle", "right")]),
    ("temperature (°C)",
     [("egt_c", "left"), ("cht_c", "left"), ("oil_c", "left"),
      ("ambient_c", "left")]),
    ("manifold / torque", [("map_kpa", "left"), ("torque_nm", "right")]),
    ("altitude (m)", [("alt_m", "left")]),
]

# base column -> (sensor value column, sensor validity column)
SENSOR_MAP = {
    "rpm": ("s_rpm", "s_rpm_ok"),
    "map_kpa": ("s_map_kpa", "s_map_ok"),
    "cht_c": ("s_cht_c", "s_cht_ok"),
    "egt_c": ("s_egt_c", "s_egt_ok"),
    "oil_c": ("s_oil_c", "s_oil_ok"),
}

# Placeholder engineering limits (caution, redline) per channel -- tune
# these alongside the physics constants.
LIMITS = {
    "rpm": (2500.0, 2700.0),
    "cht_c": (230.0, 260.0),
    "egt_c": (1050.0, 1150.0),
    "oil_c": (110.0, 130.0),
}
SHORT = {"rpm": "rpm", "map_kpa": "map", "cht_c": "cht", "egt_c": "egt",
         "oil_c": "oil"}


def find_twin_sim(explicit):
    if explicit:
        return explicit
    for name in ("twin_sim.exe", "twin_sim"):
        cand = REPO_ROOT / "build" / name
        if cand.exists():
            return str(cand)
    sys.exit("plot_run: twin_sim binary not found under build/. Build it "
             "(cmake --build build) or pass --twin-sim PATH.")


def run_twin_sim(exe, args):
    import subprocess

    proc = subprocess.run([exe, *args], capture_output=True, text=True)
    sys.stderr.write(proc.stderr)  # surface the '# twin_sim ...' banner / errors
    if proc.returncode != 0:
        sys.exit(proc.returncode)
    return proc.stdout, proc.stderr


def load_csv(stream):
    """Parse twin_sim CSV. Tolerates a merged-in '#' banner and a UTF-8 BOM.
    Returns (fieldnames, {column: [float, ...]}, banner_text)."""
    raw = list(stream)
    banner = "".join(ln for ln in raw if ln.lstrip().startswith("#"))
    lines = [ln for ln in raw if ln.strip() and not ln.lstrip().startswith("#")]
    reader = csv.DictReader(lines)
    if not reader.fieldnames:
        sys.exit("plot_run: no CSV header found in input.")
    cols = {name: [] for name in reader.fieldnames}
    for row in reader:
        for name in reader.fieldnames:
            try:
                cols[name].append(float(row.get(name, "")))
            except (TypeError, ValueError):
                cols[name].append(float("nan"))
    return list(reader.fieldnames), cols, banner


def parse_banner(text):
    out = {}
    for line in text.splitlines():
        if "profile=" not in line:
            continue
        for tok in line.lstrip("#").split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                out[k] = v
    return out


def header_text(cols, banner, source_label, title):
    if title:
        return title
    b = parse_banner(banner)
    t = cols.get("t", [])
    n = len(t)
    dt = float(b["dt"]) if "dt" in b else (t[1] - t[0] if n > 1 else float("nan"))
    dur = float(b["duration"]) if "duration" in b else (t[-1] if t else 0.0)
    rate = (1.0 / dt) if dt and dt == dt else float("nan")
    sensor = ("+sensor" in banner) or any(k.startswith("s_") for k in cols)
    src = source_label if len(source_label) <= 70 else "..." + source_label[-67:]
    gen = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")
    line1 = (f"{b.get('profile', '--')}   dt={dt:.3f}s   dur={dur:.1f}s   "
             f"load={b.get('load', '--')} N·m   seed={b.get('seed', '--')}   "
             f"sensor={'on' if sensor else 'off'}")
    line2 = f"{src}   ·   {n} samples @ {rate:.0f} Hz   ·   {gen}"
    return line1 + "\n" + line2


def is_flat(values, tol=1e-6):
    fin = [v for v in values if v == v]
    return not fin or (max(fin) - min(fin)) <= tol


def cyl_group(cols, base):
    """Sorted per-cylinder column names for `base` (e.g. cht_c_1, cht_c_2...)."""
    found = []
    for name in cols:
        if name.startswith(base + "_"):
            tail = name[len(base) + 1:]
            if tail.isdigit():
                found.append((int(tail), name))
    return [name for _, name in sorted(found)]


def group_spread(cols, names):
    """Largest max-min across the named columns at any single sample."""
    worst = 0.0
    for row in zip(*(cols[n] for n in names)):
        finite = [v for v in row if v == v]
        if finite:
            worst = max(worst, max(finite) - min(finite))
    return worst


def masked_sensor(values, ok):
    """Sensor series with dropped samples (ok == 0) blanked to NaN."""
    return [float("nan") if k < 0.5 else v for v, k in zip(values, ok)]


def series_max(vals):
    fin = [v for v in vals if v == v]
    if not fin:
        return float("nan"), 0
    hi = max(fin)
    return hi, vals.index(hi)


def render(plt, cols, header, args):
    t = cols["t"]
    explicit = bool(args.y)

    if explicit:
        missing = [c for c in args.y if c not in cols]
        if missing:
            sys.exit(f"plot_run: no such column(s): {', '.join(missing)}")
        panels = [("selected", [(c, "left") for c in args.y])]
    else:
        panels = []
        for ylabel, entries in PANEL_SPECS:
            present = [(c, s) for (c, s) in entries if c in cols]
            if not present:
                continue
            if ylabel.startswith("altitude") and all(
                    is_flat(cols[c]) for c, _ in present):
                continue
            panels.append((ylabel, present))

        # Per-cylinder panels, shown only when the cylinders actually
        # disagree (a fault run). The lumped column rides along as a
        # dashed reference.
        for base, label in (("cht_c", "cyl CHT (°C)"), ("egt_c", "cyl EGT (°C)")):
            grp = cyl_group(cols, base)
            if len(grp) >= 2 and group_spread(cols, grp) > 1.0:
                entries = [(c, "left") for c in grp]
                if base in cols:
                    entries.append((base, "left"))
                panels.append((label, entries))

        if not panels:
            sys.exit("plot_run: none of the expected columns are present; "
                     "use --y COL ...")

    n = len(panels)
    fig_h = max(args.height, 2.0 * n)  # ~2 in per panel once past ~4
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(args.width, fig_h))
    if n == 1:
        axes = [axes]

    palette = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for ax, (ylabel, entries) in zip(axes, panels):
        ax_right = None
        for ci, (col, side) in enumerate(entries):
            target = ax
            if side == "right":
                ax_right = ax_right or ax.twinx()
                target = ax_right
            # Colour every series explicitly -- a twin axis otherwise
            # restarts matplotlib's cycle and collides with the left axis.
            color = palette[ci % len(palette)]
            is_ref = col == "ambient_c" or (
                ylabel.startswith("cyl ") and col in ("cht_c", "egt_c"))
            extra = {"linestyle": "--", "alpha": 0.7} if is_ref else {}
            target.plot(t, cols[col], label=col, color=color, linewidth=1.3,
                        **extra)
            if args.sensor and col in SENSOR_MAP:
                sval, sok = SENSOR_MAP[col]
                if sval in cols and sok in cols:
                    target.plot(t, masked_sensor(cols[sval], cols[sok]),
                                label=f"{col} (sensor)", color=color,
                                linewidth=0.7, alpha=0.4)
            if col in ("cht_c", "egt_c") and not explicit and not is_ref:
                hi, idx = series_max(cols[col])
                near_end = t[idx] > 0.75 * t[-1]
                target.plot([t[idx]], [hi], marker="v", markersize=5, color=color)
                target.annotate(f"{SHORT[col]} max {hi:.0f} @ {t[idx]:.0f}s",
                                xy=(t[idx], hi),
                                xytext=(-6 if near_end else 6, 6),
                                textcoords="offset points", fontsize=7,
                                color=color, ha="right" if near_end else "left")

        if args.limits and not explicit:
            ymax = ax.get_ylim()[1]
            reds = []
            for col, side in entries:
                if col not in LIMITS or side != "left":
                    continue
                hi, _ = series_max(cols[col])
                if hi < 0.6 * LIMITS[col][0]:
                    continue  # channel nowhere near its limits -> declutter
                caut, red = LIMITS[col]
                ax.axhline(red, color="tab:red", linestyle="--", linewidth=1.0,
                           alpha=0.55)
                ax.axhline(caut, color="tab:orange", linestyle=":",
                           linewidth=1.0, alpha=0.55)
                ax.annotate(f"{SHORT[col]} redline", xy=(1.0, red),
                            xycoords=("axes fraction", "data"), xytext=(-3, 1),
                            textcoords="offset points", fontsize=6.5,
                            color="tab:red", ha="right", va="bottom", alpha=0.8)
                reds.append(red)
            if reds and max(reds) <= ymax * 2.6:
                ax.set_ylim(top=max(ymax, max(reds) * 1.05))

        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        handles, labels = ax.get_legend_handles_labels()
        if ax_right is not None:
            h2, l2 = ax_right.get_legend_handles_labels()
            handles += h2
            labels += l2
        ax.legend(handles, labels, loc="upper left", fontsize=8,
                  ncol=min(4, max(1, len(labels))))

    axes[-1].set_xlabel("t (s)")
    fig.suptitle(header, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    if args.out:
        fig.savefig(args.out, dpi=120)
        print(f"wrote {args.out}")
    else:
        plt.show()


def main():
    ap = argparse.ArgumentParser(
        description="Plot a twin_sim CSV run.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="`--sim` forwards all following args to twin_sim; "
               "put plot_run options before it.")
    ap.add_argument("csv", nargs="?",
                    help="CSV file, or '-' for stdin. Omit when using --sim.")
    ap.add_argument("--sim", nargs=argparse.REMAINDER, metavar="ARG",
                    help="Run twin_sim with the remaining args and plot its output.")
    ap.add_argument("--twin-sim", metavar="PATH", help="Path to the twin_sim binary.")
    ap.add_argument("--y", nargs="+", metavar="COL",
                    help="Plot exactly these columns on one panel (skip the layout).")
    ap.add_argument("--sensor", action="store_true",
                    help="Overlay s_* sensor channels on their truth lines.")
    ap.add_argument("--limits", action="store_true",
                    help="Overlay caution / redline lines on the instrumented panels.")
    ap.add_argument("--out", metavar="FILE",
                    help="Save the figure here instead of opening a window.")
    ap.add_argument("--title", help="Replace the metadata header with this text.")
    ap.add_argument("--width", type=float, default=11.0, help="Figure width (in).")
    ap.add_argument("--height", type=float, default=8.0, help="Figure height (in).")
    args = ap.parse_args()

    if args.sim is not None:
        text, banner = run_twin_sim(find_twin_sim(args.twin_sim), args.sim)
        _fields, cols, embedded = load_csv(io.StringIO(text))
        banner += embedded
        source_label = "twin_sim " + " ".join(args.sim)
        if "--sensor" in args.sim:
            args.sensor = True
    elif args.csv == "-" or (args.csv is None and not sys.stdin.isatty()):
        _fields, cols, banner = load_csv(sys.stdin)
        source_label = "stdin"
    elif args.csv:
        with open(args.csv, encoding="utf-8-sig") as fp:
            _fields, cols, banner = load_csv(fp)
        source_label = args.csv
    else:
        ap.error("give a CSV path, '-' for stdin, or --sim ...")

    if "t" not in cols or not cols["t"]:
        sys.exit("plot_run: CSV has no usable 't' column -- is this a twin_sim run?")

    header = header_text(cols, banner, source_label, args.title)

    try:
        import matplotlib

        if args.out:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        sys.exit("plot_run: matplotlib is required.  "
                 "pip install -r tools/requirements.txt")

    render(plt, cols, header, args)


if __name__ == "__main__":
    main()
