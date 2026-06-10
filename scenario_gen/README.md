# Closed-loop domain-randomization controller

This package's adversarial/curriculum scenario generator can run as a
**closed-loop domain-randomization (DR) controller** for *active
failure-finding*: instead of sampling scenarios from a fixed distribution, it
maintains an **adaptive** randomization distribution over the genome parameter
space, shifts that distribution each generation toward the regions where the
algorithm-under-test (AUT) is most likely to fail, and — after the run —
estimates and reports the AUT's **failure boundary**.

It plugs into the existing machinery unchanged: the same genome space
(`param_space.json` / `genome.py`), the same headless evaluator
(`run.evaluate` → `scenario_run.launch` → `ScenarioScorer` → `composite_score`),
and the same generation loop (`orchestrate.run_curriculum`). It is a Python-only
addition — no C++/CMake/pluginlib changes.

## Pieces

| Module | Role |
| --- | --- |
| `dr_distribution.py` | `DRDistribution` — a per-dimension adaptive distribution over the genome (truncated-Gaussian floats, Bernoulli gates, categorical indices). `refit()` is the CEM adaptation hook; `entropy()`/`spread()` track convergence; `to_dict()`/`from_dict()` checkpoint it. |
| `proposers/dr_proposer.py` | `DomainRandomizationProposer` — wraps `DRDistribution` in the `Proposer` contract (`make_proposer("dr", …)`). Buffers per-scenario `update()`s and refits at generation boundaries. |
| `failure_boundary.py` | `estimate_boundary()` + `write_report()` — turns the run history into a per-dimension boundary, a logistic-regression surrogate, and frontier examples; emits `failure_boundary.json` + `failure_boundary.md`. |

## The adaptation rule (CEM)

The distribution adapts via the **Cross-Entropy Method**: each generation the
highest-stress (elite) genomes are selected, and every knob's parameters are
moment-matched toward the elite set —

* **float / int** knobs → truncated-Gaussian mean+std in normalized space,
* **gate** knobs → Bernoulli `p`,
* **categorical** knobs → a smoothed categorical weight vector.

Every family carries an **entropy/variance floor** so the search keeps exploring
and never collapses. CEM was chosen over a Bayesian/surrogate optimizer to keep
the implementation pure-stdlib (matching `genome.py`), deterministic under a
seeded RNG, and compatible with the `Proposer.update(genome, score)` feedback
contract. `refit()` is a single pluggable hook, so a surrogate strategy can be
added later without changing the proposer interface.

## The fail criterion

A scenario counts as a **failure** when

```
composite_score >= fail_threshold   OR   terminal in {collision, goal_failure, timeout, wall_timeout}
```

`invalid` (broken-sim) records are excluded from the boundary estimate so a
crashed run never masquerades as a discovered failure.

## CLI usage

```bash
# Closed-loop DR run against an algorithm-under-test (writes the boundary report
# into the ledger dir at the end):
python -m scenario_gen.orchestrate \
    --proposer dr --fail-threshold 0.6 \
    --aut "roslaunch my_nav bringup.launch" \
    --generations 20 --per-generation 8 --ledger ./scenario_runs

# Offline smoke run (no ROS) via the deterministic stub evaluator:
python -m scenario_gen.orchestrate --proposer dr --stub --generations 3
```

Outputs in the ledger dir: the usual `ledger.jsonl` + `curriculum_state.json`,
plus `dr_distribution.json` (distribution checkpoint, refreshed each
generation), `failure_boundary.json`, and `failure_boundary.md`. The summary
dict gains `failure_boundary_report`, `n_failures`, `fail_rate`, and
`top_failure_dimensions`. Resume (`--no-resume` to disable) replays the ledger
back through the proposer, so a DR run continues exactly where it stopped.

## `failure_boundary.json` contract

```jsonc
{
  "schema_version": 1,
  "fail_threshold": 0.6,
  "fail_terminals": ["collision", "collisions", "goal_failure", "timeout", "wall_timeout"],
  "n_samples": 160,            // valid (non-invalid) scenarios analyzed
  "n_failures": 14,
  "n_excluded_invalid": 0,
  "fail_rate": 0.0875,
  "coverage_note": "...",      // honest note when failures are sparse / absent / total
  "per_dim": [                 // one entry per genome knob
    {
      "name": "fault0_peak", "type": "float", "group": "fault",
      "n_active": 71,          // samples where this knob's gate/slot is active
      "direction": "higher",   // higher value -> more failure ("lower"/"on"/"off"/"category")
      "crossing": 0.61,        // value where empirical P(fail) crosses 0.5 (null if none)
      "fail_region": [0.6, 0.86],
      "sensitivity": 0.79,     // point-biserial correlation with failure
      "p_fail_curve": [[center, p_fail, n], ...],
      "confidence": "high"     // "low" below the min-sample threshold
    }
    // gate knobs report p_fail_on / p_fail_off; categorical knobs report per_category
  ],
  "surrogate_weights": [{"name": "...", "weight": 2.67, "abs_weight": 2.67}, ...],
  "surrogate_bias": -1.2,
  "ranked_dimensions": ["fault0_enable", "fault0_peak", ...],  // by |surrogate weight|
  "frontier_examples": [{"genome_hash", "seed", "score", "terminal", "difficulty", "values"}, ...]
}
```

`examples/sample_failure_boundary.json` is a committed sample showing the shape
(generated from a synthetic rule for illustration). **Read coverage honestly:**
when failures are sparse, per-dimension crossings are noisy and are flagged
`confidence: "low"` / `crossing: null` rather than overclaimed — run more
generations or lower `--fail-threshold` to sharpen the boundary.

See [`examples/README.md`](examples/README.md) for the single-scenario
render/evaluate workflow and the `scenario_result.json` shape the boundary
estimator consumes.
