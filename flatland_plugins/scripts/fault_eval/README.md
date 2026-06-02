# Fault-injection offline evaluation harness

`score_rca.py` is the **only** consumer of the sealed ground-truth label. It is
offline tooling — not part of the simulator runtime or the catkin build — and
must never feed the label back into the live sim or the RCA input.

## Inputs

1. **Sealed manifest** — the JSON sidecar written by the `FaultInjector`
   (`ground_truth_path`, default `/tmp/flatland_fault_ground_truth.json`).
2. **RCA hypotheses** — a JSON file the RCA produces from the in-band bag alone.

Both schemas are documented in `../../doc/fault_injection.md`.

## Usage

```bash
python3 score_rca.py \
    --manifest /tmp/flatland_fault_ground_truth.json \
    --hypotheses rca_hypotheses.json \
    [--horizon 60] [--out report.json]
```

No third-party dependencies (Python 3 standard library only).

## Output

A JSON metrics report: precision / recall / F1 over faults, fault-type accuracy,
affected-component accuracy, mean onset-time error, mean temporal IoU
(duration/overlap), mean peak-severity error, mean time-to-detect, plus a
`per_fault` breakdown. Hypotheses are greedily matched to ground-truth faults by
matching `fault_type` + `affected_component` and maximizing temporal overlap.

## Workflow

```
flatland (FaultInjector)  --in-band-->  record_rca_bag.launch  -->  rca.bag  -->  RCA  -->  rca_hypotheses.json
                          \--sealed-->  ground_truth manifest  ----------------------------/  (scoring only)
                                                                          score_rca.py  -->  report.json
```
