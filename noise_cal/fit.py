# Copyright 2026 Avidbots Corp.
# Offline fit of the parametric_linear noise model from labeled residual data.
#
# Pure stdlib: a small OLS solver (normal equations + Gaussian elimination) so
# the toolkit runs in CI without numpy. Fits, per channel, a mean model and a
# heteroscedastic std model over the (speed, age, darkness, surface) context.
"""Fit context-conditioned noise coefficients and export the model JSON."""

import argparse
import csv
import json
import math
import random

from .schema import FEATURES
from .exporter import build_model, write_model

# E[|x|] = sigma * sqrt(2/pi) for x ~ N(0, sigma^2); invert to recover an
# unbiased per-sample estimate of sigma from a centered residual magnitude.
_HALF_NORMAL = math.sqrt(math.pi / 2.0)


def _solve_ols(x_rows, y, ridge=1e-9):
    """Solve min_b ||X b - y||^2 via the normal equations (X'X + ridge I) b = X'y.

    x_rows: list of feature vectors (each length p). Returns b (length p).
    A tiny ridge keeps a rank-deficient design (e.g. a constant feature) solvable.
    """
    p = len(x_rows[0])
    xtx = [[0.0] * p for _ in range(p)]
    xty = [0.0] * p
    for row, yi in zip(x_rows, y):
        for i in range(p):
            xty[i] += row[i] * yi
            ri = row[i]
            for j in range(p):
                xtx[i][j] += ri * row[j]
    for i in range(p):
        xtx[i][i] += ridge

    # Gaussian elimination with partial pivoting on the augmented matrix.
    a = [xtx[i][:] + [xty[i]] for i in range(p)]
    for col in range(p):
        pivot = max(range(col, p), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-18:
            continue
        a[col], a[pivot] = a[pivot], a[col]
        pv = a[col][col]
        for j in range(col, p + 1):
            a[col][j] /= pv
        for r in range(p):
            if r == col:
                continue
            factor = a[r][col]
            if factor == 0.0:
                continue
            for j in range(col, p + 1):
                a[r][j] -= factor * a[col][j]
    return [a[i][p] for i in range(p)]


def _design(rows, surfaces):
    """Build the design matrix columns: [1, speed, age, darkness, surf dummies].

    The base surface (smallest id) is folded into the intercept; one dummy per
    other surface gives its offset relative to base.
    """
    extra = [s for s in surfaces if s != surfaces[0]]
    x = []
    for r in rows:
        darkness = 1.0 - r["lighting"]
        base = [1.0, r["speed"], r["age"], darkness]
        dummies = [1.0 if r["surface_id"] == s else 0.0 for s in extra]
        x.append(base + dummies)
    return x, extra


def fit_channel(rows):
    """Fit mean + std coefficients for one channel's residual rows.

    rows: list of dicts with keys surface_id, speed, lighting, age, residual.
    Returns the per-channel dict consumed by exporter.build_channel.
    """
    surfaces = sorted({int(r["surface_id"]) for r in rows})
    x, extra = _design(rows, surfaces)
    y_mean = [r["residual"] for r in rows]

    beta_mean = _solve_ols(x, y_mean)
    # Predicted mean per row, to center the residuals for the std fit.
    mean_pred = [sum(xi * bi for xi, bi in zip(row, beta_mean)) for row in x]
    y_std = [
        abs(r["residual"] - mp) * _HALF_NORMAL for r, mp in zip(rows, mean_pred)
    ]
    beta_std = _solve_ols(x, y_std)

    def unpack(beta):
        base = beta[0]
        coef = {"speed": beta[1], "age": beta[2], "darkness": beta[3]}
        surf = {surfaces[0]: 0.0}
        for s, b in zip(extra, beta[4:]):
            surf[s] = b
        return base, coef, surf

    base_mean, mean_coef, surf_mean = unpack(beta_mean)
    base_std, std_coef, surf_std = unpack(beta_std)

    return {
        "base_mean": base_mean,
        "mean_coef": mean_coef,
        "surface_mean_offset": surf_mean,
        "base_std": base_std,
        "std_coef": std_coef,
        "surface_std_offset": surf_std,
        "n_samples": len(rows),
    }


def fit_rows(rows):
    """Group residual rows by channel and fit each. Returns {channel: fit}."""
    by_channel = {}
    for r in rows:
        by_channel.setdefault(r["channel"], []).append(r)
    return {name: fit_channel(rs) for name, rs in by_channel.items()}


def load_csv(path):
    """Read labeled residual rows from a CSV with the documented columns."""
    rows = []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        for raw in reader:
            rows.append(
                {
                    "channel": raw["channel"],
                    "surface_id": int(float(raw["surface_id"])),
                    "speed": float(raw["speed"]),
                    "lighting": float(raw["lighting"]),
                    "age": float(raw["age"]),
                    "residual": float(raw["residual"]),
                }
            )
    return rows


# Known coefficients used by the synthetic generator; tests recover these.
SYNTHETIC_TRUTH = {
    "range": {
        "base_std": 0.012,
        "std_coef": {"speed": 0.004, "age": 0.0009, "darkness": 0.020},
        "surface_std_offset": {0: 0.0, 1: 0.010, 2: 0.030},
        "base_mean": 0.0,
    },
    "odom_pose_yaw": {
        "base_std": 0.005,
        "std_coef": {"speed": 0.010, "age": 0.0, "darkness": 0.0},
        "surface_std_offset": {0: 0.0, 1: 0.004, 2: 0.012},
        "base_mean": 0.0,
    },
}


def generate_synthetic(path, n=4000, seed=7):
    """Write a synthetic labeled CSV so the pipeline is exercisable without bags.

    Samples context uniformly and draws each residual from N(mean, std) where
    std follows SYNTHETIC_TRUTH. Deterministic for a given seed (CI smoke test).
    """
    rng = random.Random(seed)
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["channel", "surface_id", "speed", "lighting", "age", "residual"])
        for _ in range(n):
            for channel, truth in SYNTHETIC_TRUTH.items():
                surface_id = rng.choice([0, 1, 2])
                speed = rng.uniform(0.0, 1.5)
                lighting = rng.uniform(0.0, 1.0)
                age = rng.uniform(0.0, 100.0)
                darkness = 1.0 - lighting
                std = (
                    truth["base_std"]
                    + truth["std_coef"]["speed"] * speed
                    + truth["std_coef"]["age"] * age
                    + truth["std_coef"]["darkness"] * darkness
                    + truth["surface_std_offset"][surface_id]
                )
                std = max(1e-6, std)
                residual = rng.gauss(truth["base_mean"], std)
                writer.writerow(
                    [channel, surface_id, "%.6f" % speed, "%.6f" % lighting,
                     "%.6f" % age, "%.8f" % residual]
                )
    return path


def _report(channel_fits):
    """Human-readable goodness-of-fit summary (base std + sample counts)."""
    lines = ["channel              n      base_std   base_mean"]
    for name in sorted(channel_fits):
        f = channel_fits[name]
        lines.append(
            "%-18s %7d   %9.5f   %9.5f"
            % (name, f["n_samples"], f["base_std"], f["base_mean"])
        )
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Fit the context-conditioned noise model and export JSON."
    )
    parser.add_argument(
        "--data", help="Labeled residual CSV (columns: channel,surface_id,"
        "speed,lighting,age,residual)."
    )
    parser.add_argument(
        "--synthetic", metavar="PATH",
        help="Generate a synthetic CSV at PATH and fit from it (CI smoke path)."
    )
    parser.add_argument("--n", type=int, default=4000, help="Synthetic sample count.")
    parser.add_argument("--seed", type=int, default=7, help="Synthetic RNG seed.")
    parser.add_argument("--out", required=True, help="Output model JSON path.")
    parser.add_argument("--report", action="store_true", help="Print a fit report.")
    args = parser.parse_args(argv)

    data_path = args.data
    if args.synthetic:
        generate_synthetic(args.synthetic, n=args.n, seed=args.seed)
        data_path = args.synthetic
    if not data_path:
        parser.error("provide --data or --synthetic")

    rows = load_csv(data_path)
    channel_fits = fit_rows(rows)
    fit_source = "synthetic" if args.synthetic else "rosbag"
    model = build_model(channel_fits, metadata={"fit_source": fit_source})
    write_model(model, args.out)

    if args.report:
        print(_report(channel_fits))
    print("wrote %s (%d channels)" % (args.out, len(channel_fits)))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
