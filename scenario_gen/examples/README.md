# Scenario generator — worked example

`example_genome.json` is a hand-designed **adversarial** scenario for the
`conestogo` template that compounds three stressors a navigation stack handles
poorly together:

* **aisle narrowing** (`mut_aisle_delta_m = 0.18`) — tightens the corridors,
* **encoder drift** (`fault0`, peak 0.8 from t=6 s) — odometry slowly diverges,
* **dynamic obstacle** (`fault1`, from t=4 s) — a mover at the chokepoint.

Difficulty ≈ 0.56, genome hash `9c3334cba701` (seed 42).

### Render it to a world

```bash
python -m scenario_gen.render \
    --template conestogo --seed 42 \
    --genome scenario_gen/examples/example_genome.json \
    --out /tmp/example_scenario
# -> /tmp/example_scenario/world.yaml  (+ render_meta.json, genome.json)
```

The rendered `world.yaml` uses **absolute** map/model paths (so it is
location-independent at runtime) and is therefore *generated*, not committed.

### Evaluate it against an algorithm-under-test

```bash
python -m scenario_gen.run \
    --template conestogo --seed 42 \
    --genome scenario_gen/examples/example_genome.json \
    --aut "roslaunch my_nav bringup.launch" \
    --out /tmp/example_scenario
```

`sample_scenario_result.json` shows the **shape** of the sealed stress-score
output the runner writes (composite score + component metrics + weights +
reproducibility metadata). Higher `composite_score` = the algorithm did worse.

### Closed-loop DR controller

`sample_failure_boundary.json` shows the **shape** of the failure-boundary report
the closed-loop domain-randomization controller emits at the end of a run
(per-dimension `P(fail)=0.5` crossings, a logistic-regression surrogate ranking
the failure-driving knobs, and frontier examples). It is generated from a
synthetic rule for illustration. See [`../README.md`](../README.md) for the DR
controller, the `--proposer dr --fail-threshold` CLI, and the
`failure_boundary.json` contract.

See [`../../docs/scenario_generator.md`](../../docs/scenario_generator.md) for the
full contract and the curriculum loop.
