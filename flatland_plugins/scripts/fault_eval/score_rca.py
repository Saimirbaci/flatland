#!/usr/bin/env python3
# @copyright Copyright 2026 Avidbots Corp.
# @name   score_rca.py
# @brief  Offline scorer for the fault-injection benchmark
# @author Saimir Baci
#
# OFFLINE TOOLING ONLY. This is the *single* consumer of the sealed ground-truth
# label. It is NOT part of the simulator runtime/build and must never feed the
# label back into the live sim or the RCA input. It reads:
#   * the sealed sidecar manifest written by the FaultInjector
#     (ground_truth_path), and
#   * the RCA's hypotheses file,
# and emits scoring metrics. See ../../doc/fault_injection.md for the formats.

import argparse
import json
import sys


def _load(path):
    with open(path, "r") as f:
        return json.load(f)


def _effective_end(end, horizon):
    """Indefinite / not-yet-ended faults (end < 0) extend to the horizon."""
    return horizon if end is None or end < 0 else end


def _overlap(a_on, a_end, b_on, b_end):
    """Length of the temporal intersection of [a_on,a_end] and [b_on,b_end]."""
    return max(0.0, min(a_end, b_end) - max(a_on, b_on))


def _temporal_iou(a_on, a_end, b_on, b_end):
    inter = _overlap(a_on, a_end, b_on, b_end)
    union = (a_end - a_on) + (b_end - b_on) - inter
    return inter / union if union > 0 else 0.0


def score(manifest, hypotheses, horizon=None):
    truths = manifest.get("faults", [])
    hyps = hypotheses.get("hypotheses", [])

    # Horizon for indefinite faults: a bit past the latest known time.
    times = []
    for t in truths:
        times += [t.get("onset_time", 0.0), t.get("end_time", 0.0)]
    for h in hyps:
        times += [h.get("onset_time", 0.0), h.get("end_time", 0.0),
                  h.get("detected_at", 0.0)]
    if horizon is None:
        horizon = max([abs(x) for x in times] + [0.0]) + 1.0

    # Only ground-truth faults that actually fired (onset resolved) are scorable.
    active_truths = [t for t in truths if t.get("onset_time", -1.0) >= 0.0]

    # Greedy match: best (type+component) hypothesis by temporal overlap.
    used = set()
    matches = []
    for ti, t in enumerate(active_truths):
        t_on = t.get("onset_time", 0.0)
        t_end = _effective_end(t.get("end_time", -1.0), horizon)
        best, best_ov = None, 0.0
        for hi, h in enumerate(hyps):
            if hi in used:
                continue
            if h.get("fault_type") != t.get("fault_type"):
                continue
            if h.get("affected_component") != t.get("affected_component"):
                continue
            h_on = h.get("onset_time", 0.0)
            h_end = _effective_end(h.get("end_time", -1.0), horizon)
            ov = _overlap(t_on, t_end, h_on, h_end)
            if ov > best_ov:
                best, best_ov = hi, ov
        if best is not None:
            used.add(best)
            matches.append((ti, best))

    tp = len(matches)
    fp = len(hyps) - tp
    fn = len(active_truths) - tp
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = (2 * precision * recall / (precision + recall)
          if (precision + recall) else 0.0)

    onset_errs, sev_errs, ious, ttds, comp_hits = [], [], [], [], []
    per_fault = []
    for ti, hi in matches:
        t, h = active_truths[ti], hyps[hi]
        t_on = t.get("onset_time", 0.0)
        t_end = _effective_end(t.get("end_time", -1.0), horizon)
        h_on = h.get("onset_time", 0.0)
        h_end = _effective_end(h.get("end_time", -1.0), horizon)
        onset_err = abs(t_on - h_on)
        sev_err = abs(t.get("peak_severity", 0.0) - h.get("peak_severity", 0.0))
        iou = _temporal_iou(t_on, t_end, h_on, h_end)
        comp_hit = (h.get("affected_model") == t.get("affected_model"))
        onset_errs.append(onset_err)
        sev_errs.append(sev_err)
        ious.append(iou)
        comp_hits.append(1.0 if comp_hit else 0.0)
        if "detected_at" in h:
            ttds.append(h["detected_at"] - t_on)
        per_fault.append({
            "fault_id": t.get("fault_id"),
            "fault_type": t.get("fault_type"),
            "affected_component": t.get("affected_component"),
            "onset_error": onset_err,
            "peak_severity_error": sev_err,
            "temporal_iou": iou,
            "component_match": comp_hit,
            "time_to_detect": (h["detected_at"] - t_on
                               if "detected_at" in h else None),
        })

    def _mean(xs):
        return sum(xs) / len(xs) if xs else None

    return {
        "num_truth_faults": len(active_truths),
        "num_hypotheses": len(hyps),
        "true_positives": tp,
        "false_positives": fp,
        "false_negatives": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "fault_type_accuracy": (tp / len(active_truths)
                                if active_truths else 0.0),
        "component_accuracy": _mean(comp_hits),
        "mean_onset_error": _mean(onset_errs),
        "mean_peak_severity_error": _mean(sev_errs),
        "mean_temporal_iou": _mean(ious),
        "mean_time_to_detect": _mean(ttds),
        "per_fault": per_fault,
    }


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Score RCA hypotheses against the sealed fault manifest.")
    p.add_argument("--manifest", required=True,
                   help="sealed ground-truth sidecar JSON (ground_truth_path)")
    p.add_argument("--hypotheses", required=True,
                   help="RCA hypotheses JSON")
    p.add_argument("--horizon", type=float, default=None,
                   help="sim-time horizon for indefinite faults (s)")
    p.add_argument("--out", default=None,
                   help="write the metrics report JSON here (default: stdout)")
    args = p.parse_args(argv)

    report = score(_load(args.manifest), _load(args.hypotheses), args.horizon)
    text = json.dumps(report, indent=2)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text + "\n")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
