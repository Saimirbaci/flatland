# Copyright 2026 Avidbots Corp.
# Failure-boundary estimator/reporter for the closed-loop DR controller: after a
# run, characterize *where* in the genome space the algorithm-under-test fails.
"""Estimate and report the algorithm-under-test's failure boundary.

After the closed-loop domain-randomization run has concentrated samples near the
AUT's failure region, this module turns the accumulated run history (the ledger
records: genome dict + composite stress score + terminal condition) into a
human- and machine-readable description of the **failure boundary** — the
frontier in the genome space separating scenarios the AUT survives from those it
fails.

Three complementary views are produced:

1. **Per-dimension boundary** — for each *active* knob, the empirical
   ``P(fail)`` as a function of the knob value, the value where it crosses 0.5,
   the monotonic direction, a point-biserial sensitivity, and the marginal
   failure region. Guarded by a minimum-sample threshold so sparse dimensions
   report low confidence rather than overclaiming.
2. **Logistic-regression surrogate** — a hand-rolled, dependency-free logistic
   regression over the normalized genome vector (fixed-iteration gradient
   descent, zero init -> deterministic). Its weights rank the dimensions by how
   strongly they drive failure and characterize the global decision surface.
3. **Frontier examples** — the highest-stress failing genomes, for replay.

``estimate_boundary(history, fail_threshold)`` returns the report dict;
``write_report(report, out_dir)`` emits ``failure_boundary.json`` (machine
contract) and ``failure_boundary.md`` (summary).
"""

import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple

from .genome import Genome, load_param_space

SCHEMA_VERSION = 1

# Terminal conditions that count as a failure regardless of composite score.
FAIL_TERMINALS = frozenset({
    "collision", "collisions", "goal_failure", "timeout", "wall_timeout"})

# A per-dimension boundary needs at least this many *active* samples to be
# reported with confidence; below it the dimension is flagged low-confidence.
MIN_ACTIVE_SAMPLES = 8


def is_failure(score: float, terminal: str, fail_threshold: float) -> bool:
    """The fail criterion: high stress OR an explicit failure terminal."""
    if terminal and str(terminal).lower() in FAIL_TERMINALS:
        return True
    return float(score) >= float(fail_threshold)


def _pearson(xs: List[float], ys: List[float]) -> float:
    """Pearson correlation (point-biserial when ``ys`` is 0/1). 0 if degenerate."""
    n = len(xs)
    if n < 2:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 0.0 or syy <= 0.0:
        return 0.0
    return sxy / math.sqrt(sxx * syy)


def _bin_pfail(pairs: List[Tuple[float, int]], n_bins: int = 5):
    """Bin (value, fail) pairs by value quantiles -> list of (center, p_fail, n).

    Quantile binning keeps each bin populated even with a skewed value
    distribution (the DR loop concentrates samples).
    """
    if not pairs:
        return []
    ordered = sorted(pairs, key=lambda t: t[0])
    n = len(ordered)
    n_bins = max(1, min(n_bins, n))
    bins = []
    for b in range(n_bins):
        lo_i = (b * n) // n_bins
        hi_i = ((b + 1) * n) // n_bins
        if hi_i <= lo_i:
            continue
        chunk = ordered[lo_i:hi_i]
        center = sum(v for v, _ in chunk) / len(chunk)
        p = sum(f for _, f in chunk) / len(chunk)
        bins.append((center, p, len(chunk)))
    return bins


def _crossing(bins) -> Optional[float]:
    """Value where binned P(fail) crosses 0.5 (linear interp), else None."""
    for i in range(1, len(bins)):
        (c0, p0, _), (c1, p1, _) = bins[i - 1], bins[i]
        if (p0 < 0.5 <= p1) or (p1 < 0.5 <= p0):
            if p1 == p0:
                return 0.5 * (c0 + c1)
            t = (0.5 - p0) / (p1 - p0)
            return c0 + t * (c1 - c0)
    return None


def _fail_region(bins) -> Optional[List[float]]:
    """Contiguous value span where binned P(fail) >= 0.5 (genome units)."""
    hi_centers = [c for c, p, _ in bins if p >= 0.5]
    if not hi_centers:
        return None
    return [round(min(hi_centers), 4), round(max(hi_centers), 4)]


# ----------------------------------------------------------------- data assembly
def _assemble(history: List[Dict[str, Any]], fail_threshold: float, ps):
    """Reconstruct genomes + labels, skipping invalid (broken-sim) records."""
    genomes: List[Genome] = []
    labels: List[int] = []
    scores: List[float] = []
    terminals: List[str] = []
    excluded = 0
    for rec in history:
        if rec.get("invalid"):
            excluded += 1
            continue
        gd = rec.get("genome")
        if not gd:
            excluded += 1
            continue
        try:
            g = Genome.from_dict(gd, ps)
        except Exception:
            excluded += 1
            continue
        score = float(rec.get("score", rec.get("composite_score", 0.0)))
        terminal = rec.get("terminal", "")
        genomes.append(g)
        labels.append(1 if is_failure(score, terminal, fail_threshold) else 0)
        scores.append(score)
        terminals.append(terminal)
    return genomes, labels, scores, terminals, excluded


def _norm_vectors(genomes: List[Genome], ps):
    lows, highs = Genome.vector_bounds(ps)
    out = []
    for g in genomes:
        vec = g.to_vector()
        out.append([0.0 if hi <= lo else max(0.0, min(1.0, (v - lo) / (hi - lo)))
                    for v, lo, hi in zip(vec, lows, highs)])
    return out


# --------------------------------------------------------- logistic surrogate
def _fit_logistic(X: List[List[float]], y: List[int], iters: int = 400,
                  lr: float = 0.3, l2: float = 1e-3):
    """Deterministic hand-rolled logistic regression (zero init, fixed iters)."""
    if not X:
        return [], 0.0
    n, dim = len(X), len(X[0])
    w = [0.0] * dim
    b = 0.0
    for _ in range(iters):
        gw = [0.0] * dim
        gb = 0.0
        for xi, yi in zip(X, y):
            z = b + sum(w[j] * xi[j] for j in range(dim))
            # numerically stable sigmoid
            if z >= 0:
                p = 1.0 / (1.0 + math.exp(-z))
            else:
                e = math.exp(z)
                p = e / (1.0 + e)
            err = p - yi
            for j in range(dim):
                gw[j] += err * xi[j]
            gb += err
        for j in range(dim):
            w[j] -= lr * (gw[j] / n + l2 * w[j])
        b -= lr * (gb / n)
    return w, b


# ------------------------------------------------------------------- estimator
def estimate_boundary(history: List[Dict[str, Any]], fail_threshold: float = 0.6,
                      param_space: Optional[Dict[str, Any]] = None,
                      top_k_frontier: int = 8) -> Dict[str, Any]:
    """Characterize the AUT's failure boundary from the run history.

    Args:
        history: ledger-style records, each with ``genome`` (dict),
            ``score``/``composite_score``, and ``terminal`` (+ optional
            ``invalid``, ``genome_hash``).
        fail_threshold: composite-score threshold for the fail criterion.
        param_space: parsed param_space (loaded when omitted).
        top_k_frontier: number of hardest failing genomes to report.

    Returns a JSON-serializable report dict (see module docstring / the
    ``failure_boundary.json`` contract).
    """
    ps = param_space or load_param_space()
    dims = ps["dimensions"]
    genomes, labels, scores, terminals, excluded = _assemble(
        history, fail_threshold, ps)
    n = len(genomes)
    n_fail = sum(labels)

    report: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "fail_threshold": float(fail_threshold),
        "fail_terminals": sorted(FAIL_TERMINALS),
        "n_samples": n,
        "n_failures": n_fail,
        "n_excluded_invalid": excluded,
        "fail_rate": round(n_fail / n, 4) if n else 0.0,
        "per_dim": [],
        "surrogate_weights": [],
        "surrogate_bias": 0.0,
        "ranked_dimensions": [],
        "frontier_examples": [],
        "coverage_note": "",
    }
    if n == 0:
        report["coverage_note"] = "No valid samples; nothing to estimate."
        return report
    if n_fail == 0:
        report["coverage_note"] = (
            "No failures observed at fail_threshold=%.3f; boundary not reached. "
            "Lower the threshold or run more generations." % fail_threshold)
    elif n_fail == n:
        report["coverage_note"] = (
            "All samples failed; the run never sampled the survivable region. "
            "Widen exploration or raise fail_threshold.")

    # ---- per-dimension empirical boundary ----
    for d in dims:
        name, t = d["name"], d["type"]
        entry: Dict[str, Any] = {"name": name, "type": t,
                                 "group": d.get("group", "")}
        if t == "gate":
            on = [(g, lab) for g, lab in zip(genomes, labels)
                  if float(g.values[name]) >= 0.5]
            off = [(g, lab) for g, lab in zip(genomes, labels)
                   if float(g.values[name]) < 0.5]
            p_on = (sum(l for _, l in on) / len(on)) if on else None
            p_off = (sum(l for _, l in off) / len(off)) if off else None
            entry.update({
                "n_on": len(on), "n_off": len(off),
                "p_fail_on": round(p_on, 4) if p_on is not None else None,
                "p_fail_off": round(p_off, 4) if p_off is not None else None,
                "direction": ("on" if (p_on or 0) >= (p_off or 0) else "off"),
                "confidence": ("high" if min(len(on), len(off)) >= MIN_ACTIVE_SAMPLES
                               else "low"),
            })
        elif t == "categorical":
            n_cat = _n_categories_for(ps, d)
            per_cat = []
            for c in range(n_cat):
                sel = [lab for g, lab in zip(genomes, labels)
                       if int(g.values[name]) % max(1, n_cat) == c]
                if sel:
                    per_cat.append({"category": c, "n": len(sel),
                                    "p_fail": round(sum(sel) / len(sel), 4)})
            worst = max(per_cat, key=lambda e: e["p_fail"], default=None)
            entry.update({
                "per_category": per_cat,
                "worst_category": worst["category"] if worst else None,
                "direction": "category",
                "confidence": "high" if len(per_cat) and all(
                    e["n"] >= MIN_ACTIVE_SAMPLES for e in per_cat) else "low",
            })
        else:  # float / int — only over samples where the knob is active
            pairs = [(float(g.values[name]), lab)
                     for g, lab in zip(genomes, labels) if g._knob_active(d)]
            n_active = len(pairs)
            bins = _bin_pfail(pairs)
            xs = [v for v, _ in pairs]
            ys = [float(f) for _, f in pairs]
            entry.update({
                "n_active": n_active,
                "sensitivity": round(_pearson(xs, ys), 4),
                "direction": _direction(pairs),
                "crossing": (round(_crossing(bins), 4)
                             if _crossing(bins) is not None else None),
                "fail_region": _fail_region(bins),
                "p_fail_curve": [[round(c, 4), round(p, 4), nn]
                                 for c, p, nn in bins],
                "confidence": ("high" if n_active >= MIN_ACTIVE_SAMPLES
                               else "low"),
            })
        report["per_dim"].append(entry)

    # ---- logistic-regression surrogate ----
    if 0 < n_fail < n:
        X = _norm_vectors(genomes, ps)
        w, b = _fit_logistic(X, labels)
        report["surrogate_bias"] = round(b, 6)
        report["surrogate_weights"] = [
            {"name": d["name"], "weight": round(wj, 6),
             "abs_weight": round(abs(wj), 6)}
            for d, wj in zip(dims, w)]
        report["ranked_dimensions"] = [
            e["name"] for e in sorted(report["surrogate_weights"],
                                      key=lambda e: e["abs_weight"], reverse=True)
        ]
    else:
        # Degenerate label set: rank by |sensitivity| instead so the report is
        # still useful (and honest about the surrogate not fitting).
        ranked = sorted(
            (e for e in report["per_dim"] if e["type"] not in ("gate", "categorical")),
            key=lambda e: abs(e.get("sensitivity", 0.0)), reverse=True)
        report["ranked_dimensions"] = [e["name"] for e in ranked]

    # ---- frontier examples (hardest failing genomes) ----
    failing = [(g, sc, term, lab) for g, sc, term, lab in
               zip(genomes, scores, terminals, labels) if lab == 1]
    failing.sort(key=lambda t: t[1], reverse=True)
    for g, sc, term, _ in failing[:top_k_frontier]:
        report["frontier_examples"].append({
            "genome_hash": g.hash(), "seed": g.seed,
            "score": round(float(sc), 4), "terminal": term,
            "difficulty": round(g.difficulty(), 4),
            "values": dict(g.values),
        })
    return report


def _n_categories_for(ps, d):
    ref = d.get("categories_ref")
    if ref == "fault_types":
        return len(ps["fault_types"])
    return d.get("n_categories", 8)


def _direction(pairs: List[Tuple[float, int]]) -> str:
    """Does a higher knob value increase failure?"""
    fails = [v for v, f in pairs if f]
    safes = [v for v, f in pairs if not f]
    if not fails or not safes:
        return "unknown"
    return "higher" if (sum(fails) / len(fails)) >= (sum(safes) / len(safes)) \
        else "lower"


# --------------------------------------------------------------------- reporting
def write_report(report: Dict[str, Any], out_dir: str) -> Dict[str, str]:
    """Write ``failure_boundary.json`` + ``failure_boundary.md`` into ``out_dir``.

    Returns the two artifact paths.
    """
    os.makedirs(out_dir, exist_ok=True)
    json_path = os.path.join(out_dir, "failure_boundary.json")
    md_path = os.path.join(out_dir, "failure_boundary.md")
    with open(json_path, "w") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    with open(md_path, "w") as f:
        f.write(_render_markdown(report))
    return {"json": json_path, "markdown": md_path}


def _render_markdown(r: Dict[str, Any]) -> str:
    lines = ["# Failure boundary report", ""]
    lines.append("- **Samples:** %d (failures: %d, fail rate: %.1f%%)"
                 % (r["n_samples"], r["n_failures"], 100.0 * r["fail_rate"]))
    lines.append("- **Fail criterion:** composite_score >= %.3f OR terminal in %s"
                 % (r["fail_threshold"], ", ".join(r["fail_terminals"])))
    if r.get("n_excluded_invalid"):
        lines.append("- **Excluded (invalid/broken):** %d"
                     % r["n_excluded_invalid"])
    if r.get("coverage_note"):
        lines.append("- **Coverage:** %s" % r["coverage_note"])
    lines.append("")

    ranked = r.get("ranked_dimensions", [])
    if ranked:
        lines.append("## Top failure-driving dimensions")
        lines.append("")
        wmap = {e["name"]: e for e in r.get("surrogate_weights", [])}
        if wmap:
            lines.append("| rank | dimension | surrogate weight |")
            lines.append("| ---: | --- | ---: |")
            for i, name in enumerate(ranked[:10], 1):
                w = wmap.get(name, {}).get("weight", 0.0)
                lines.append("| %d | `%s` | %+.4f |" % (i, name, w))
        else:
            lines.append("_Surrogate did not fit (degenerate labels); "
                         "ranked by per-dim sensitivity:_")
            lines.append("")
            for i, name in enumerate(ranked[:10], 1):
                lines.append("%d. `%s`" % (i, name))
        lines.append("")

    # Per-dim crossings worth reporting (high confidence, float/int).
    crossings = [e for e in r.get("per_dim", [])
                 if e.get("crossing") is not None and e.get("confidence") == "high"]
    if crossings:
        lines.append("## Per-dimension failure boundary (P(fail)=0.5 crossing)")
        lines.append("")
        lines.append("| dimension | direction | crossing | fail region | sensitivity |")
        lines.append("| --- | --- | ---: | --- | ---: |")
        for e in crossings:
            region = ("[%.3f, %.3f]" % tuple(e["fail_region"])
                      if e.get("fail_region") else "—")
            lines.append("| `%s` | %s | %.4f | %s | %+.3f |"
                         % (e["name"], e["direction"], e["crossing"], region,
                            e.get("sensitivity", 0.0)))
        lines.append("")

    # Gate toggles with a clear failure bias.
    gates = [e for e in r.get("per_dim", [])
             if e["type"] == "gate" and e.get("confidence") == "high"]
    if gates:
        lines.append("## Structure toggles (gates)")
        lines.append("")
        lines.append("| gate | P(fail) on | P(fail) off | drives failure |")
        lines.append("| --- | ---: | ---: | --- |")
        for e in gates:
            lines.append("| `%s` | %s | %s | %s |"
                         % (e["name"],
                            _fmt(e.get("p_fail_on")), _fmt(e.get("p_fail_off")),
                            "when ON" if e["direction"] == "on" else "when OFF"))
        lines.append("")

    fx = r.get("frontier_examples", [])
    if fx:
        lines.append("## Frontier examples (hardest failures)")
        lines.append("")
        lines.append("| genome | seed | score | terminal | difficulty |")
        lines.append("| --- | ---: | ---: | --- | ---: |")
        for e in fx:
            lines.append("| `%s` | %d | %.3f | %s | %.3f |"
                         % (e["genome_hash"], e["seed"], e["score"],
                            e["terminal"] or "—", e["difficulty"]))
        lines.append("")
    return "\n".join(lines) + "\n"


def _fmt(x) -> str:
    return "%.3f" % x if isinstance(x, (int, float)) else "—"
