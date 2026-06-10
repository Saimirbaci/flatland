# AI-Driven Adversarial / Curriculum Scenario Generator

An **outer loop** that lets an AI policy (RL black-box optimizer **or** LLM)
propose parameterized map-mutation + fault scenarios that *specifically stress*
an algorithm-under-test (failure-seeking, not random), with a difficulty
**curriculum** and full **deterministic reproducibility**.

It is built entirely on top of Flatland's existing primitives:

| Existing primitive | Role here |
|---|---|
| `MapMutator` (`flatland_server/.../map_mutator.h`) | procedural map perturbation (`layers[].mutation`) |
| `FaultInjector` world plugin (`flatland_plugins/.../fault_injector.h`) | sensor/drive/localization/environment faults + sealed ground truth |
| `RngManager` (`flatland_server/.../random.h`) | one seed → fully reproducible run |
| `ScenarioScorer` + `flatland_scenario_runner` (added here) | quantitative *stress score* from the sim |
| `flatland_benchmark` / `benchmark.launch` | headless run model the harness mirrors |

```
              ┌─────────────── outer loop (Python, scenario_gen/) ───────────────┐
 curriculum ──▶ proposer (RL | LLM) ──▶ genome ──▶ render.py ──▶ world.yaml
   ▲                                                                  │
   │                                                                  ▼
   │                                            run.py + scenario_run.launch (ROS, headless)
   │                                                                  │
   │                                       flatland_scenario_runner → scenario_result.json
   └──────────────── (genome, stress score, failure summary) ◀────────┘
```

---

## 1. The scenario "genome" (the search space)

The genome is the **single versioned contract** every component agrees on:
[`scenario_gen/param_space.json`](../scenario_gen/param_space.json) (machine
readable) and `scenario_gen/genome.py` (the `Genome` class).

A genome is a **fixed-length** vector of named, bounded knobs — fixed length so
RL/black-box optimizers see a stable action space and the LLM emits a stable
JSON schema. Structure (which map ops, how many faults) is expressed with **0/1
gates** and a fixed number of **fault slots** rather than variable-length lists.

Knob groups (`param_space.json → dimensions`):

* **`map`** — wall jitter (translation/rotation), aisle widen/narrow delta,
  clutter add/remove + blob size, obstacle-density scale. Each op has an
  `*_enable` gate. These map 1:1 onto `MapMutator::FromConfig` keys
  (`max_translation_m`, `dilate_erode_range_m`, `add_count_range`,
  `blob_size_m_range`, `target_scale`, …).
* **`fault`** — `n_fault_slots` (default 3) identical slots, each with
  `enable` gate, `type` (categorical over `param_space.json → fault_types`),
  `onset_s`, `peak`, `ramp_up_s`, `hold_s`, and a generic `magnitude` mapped to
  the type's primary `params:` key (`fault_types[type].param * scale`). These map
  1:1 onto a `FaultInjector` `faults:` entry (`severity.onset_time/ramp_up/hold/
  peak/profile`, `target.model/component/topic`, `params`).
* **`placement`** — `start_idx` / `goal_idx`, **categorical over the world
  template's free-space candidate poses**. Indexing into a curated candidate
  list (never a raw x/y) guarantees the robot/goal never land inside a wall — the
  primary validity safeguard.

Each knob carries a `difficulty_weight ∈ [0,1]`. `Genome.difficulty()` returns a
scalar in ~[0,1] (weighted mean of *active* knobs' normalized magnitudes) that
the curriculum uses to bin scenarios.

`fault_types` also pins, per taxonomy type, a **default target component+topic**
and the **primary param** the magnitude knob drives — so a genome can never pick
an invalid `(type, component)` pair. The taxonomy mirrors
`FaultInjector::ParseFaultKind`: sensor (`sensor_bias`, `sensor_drift`,
`sensor_scale`, `noise_inflation`, `dropout`, `stuck`, `laser_sector_occlusion`,
`latency`, `ghost_return`), drive (`torque_loss`, `wheel_slip`,
`asymmetric_drive`, `deadband`, `motor_degradation`), localization
(`encoder_drift`, `odom_slip`, `amcl_divergence`), environment
(`dynamic_obstacle`, `spill`).

### Genome ↔ YAML

`render.py` is the **only** translator. A genome + seed →
fully-formed, *flat, sealed* `world.yaml`:

```yaml
properties:
  seed: <genome.seed>
layers:
  - { name: "2d", map: "map.yaml", color: [0,1,0,1], mutation: { ...decoded map knobs... } }
models:
  - { name: robot, pose: [<start pose>], model: "robot.model.yaml" }
plugins:
  - type: FaultInjector
    name: faults
    ground_truth_path: "<out>/fault_ground_truth.json"
    faults: [ ...one entry per enabled fault slot... ]
```

The render is *flat* (no Lua `$eval`) on purpose: a run is reproducible from the
genome + seed alone, with nothing left to evaluate at load time. Render then
**dry-loads** the YAML through `YamlReader` (via a lightweight schema check, and
optionally a real `roslaunch` load) before a run is attempted, so an
out-of-bounds or malformed genome fails fast instead of mid-run.

---

## 2. Stress score (the algorithm-under-test signal)

The C++ side emits a quantitative **stress score** measuring how badly the
algorithm-under-test performs, read against the **sealed out-of-band ground
truth** so the algorithm never observes its own grade.

* `flatland_server/include/flatland_server/scenario_scorer.h` /
  `src/scenario_scorer.cpp` — a **pure, unit-testable** accumulator + score math
  (no ROS, no Box2D), mirroring how `FaultInjectionRegistry` hosts pure helpers.
* `flatland_server/src/flatland_scenario_runner.cpp` — a thin ROS node that
  feeds the scorer from live topics and the **true pose read out-of-band via
  `tf2`** (global frame → robot base frame, the same true transform
  `model_tf_publisher` emits), then writes `scenario_result.json`.

Component metrics (weights configurable via the `score_weights` rosparam):

| metric | source | meaning |
|---|---|---|
| localization error | true pose (tf) vs estimate (`amcl_pose`) | mean/max position error (m) |
| collisions | `flatland_msgs/Collisions` | contact count over the run |
| goal failure | true pose vs goal (`goal_x/goal_y/goal_radius`) | reached? + final distance |
| tracking error | true pose vs straight start→goal line | path deviation (m) |

`CompositeScore` maps each component to ~[0,1] (saturating) and returns the
weighted mean — **higher = the algorithm did worse** (more stress). The runner
writes a sealed `scenario_result.json` next to the fault/map manifests:

```json
{
  "seed": 42, "genome_hash": "abc123...", "composite_score": 0.73,
  "metrics": { "mean_localization_error_m": 0.41, "collision_count": 5,
               "goal_reached": false, "goal_distance_m": 3.2,
               "mean_tracking_error_m": 0.6, "sim_time_s": 60.0,
               "time_to_goal_s": -1.0 },
  "weights": { "localization_error": 1.0, "collisions": 1.0,
               "goal_failure": 1.0, "tracking_error": 1.0 },
  "terminal": "timeout"
}
```

A crashed/hung run is scored by the harness as **max stress + `invalid: true`**
so the optimizer never rewards a scenario that merely broke the simulator.

> **Reward-hacking guard.** A purely failure-seeking objective will drift toward
> *unsolvable* scenarios. The curriculum applies a **solvability regularizer**:
> scenarios where even a clean reference run fails are penalized / excluded, and
> the curriculum targets a success band (default 50–70%) rather than 0% success.

---

## 3. Reproducibility

One integer seed reproduces a run exactly: harness rosparam `seed` →
`properties.seed` → `RngManager::Seed` → every `MapMutator`/`FaultInjector`
draw. The orchestrator persists, per scenario, `{genome_hash, seed, score,
ground_truth manifest paths, world.yaml path}` to a **run ledger** so any
discovered failure replays standalone:

```bash
python -m scenario_gen.run --genome ledger/gen07/scn03.genome.json --seed 42
```

---

## 4. Architecture decision — where the AI policy lives

**Decision: external Python orchestrator (Option A).** The proposer/curriculum
"brain" lives entirely in `scenario_gen/` (Python); only the **genome schema**
and the **score emission** live in the C++ core. Flatland stays a pure,
deterministic black-box evaluator invoked per scenario.

* **Option A — external Python orchestrator (chosen).**
  *Pros:* clean separation; first-class access to RL/LLM ecosystems; **no ML
  deps in the catkin build**; Flatland stays a deterministic evaluator; trivially
  parallelizable across processes. *Cons:* per-scenario `roslaunch` latency;
  genome/score plumbed as JSON across the language boundary.
* **Option B — in-engine C++ world plugin** that re-parameterizes between
  episodes. *Rejected:* couples ML/runtime deps into the ROS node, complicates
  the build, and fights the existing "one world.yaml = one sealed run" model.

Consequence: the C++ additions are minimal and dependency-light
(`ScenarioScorer` + `flatland_scenario_runner`), and everything adaptive is
Python under `scenario_gen/`, kept out of `catkin build`.

---

## 5. Components (`scenario_gen/`)

| File | Role |
|---|---|
| `param_space.json` | versioned genome contract (the search space) |
| `genome.py` | `Genome`: encode/decode vector, JSON, random, clamp, difficulty, hash |
| `render.py` | genome + template → loadable `world.yaml` (+ dry-load validation) |
| `run.py` | one genome → headless run via `scenario_run.launch` → stress score |
| `proposers/base.py` | `Proposer` interface: `propose(history) -> genome`, `update(genome, score)` |
| `proposers/rl_proposer.py` | CMA-ES / Gaussian black-box optimizer over the vector |
| `proposers/llm_proposer.py` | Claude-driven proposer; schema-validated JSON genomes + rationale |
| `curriculum.py` | difficulty controller + scenario archive (MAP-Elites-style), solvability regularizer |
| `orchestrate.py` | the generation loop + reproducible run ledger (CLI entrypoint) |
| `templates/` | world templates (base map/model + free-space start/goal candidates) |

---

## 6. End-to-end usage

```bash
# 0. (once) build the C++ runner + scorer
catkin build flatland_server flatland_plugins

# 1. render a single genome to a world (no ROS needed) — sanity check
python -m scenario_gen.render --template conestogo --seed 42 \
    --out /tmp/scn --random

# 2. evaluate one genome headlessly against your algorithm-under-test
python -m scenario_gen.run --template conestogo --seed 42 --random \
    --aut "roslaunch my_nav bringup.launch" --out /tmp/scn

# 3. run an adversarial curriculum loop
python -m scenario_gen.orchestrate \
    --template conestogo --proposer llm \
    --aut "roslaunch my_nav bringup.launch" \
    --generations 20 --per-generation 8 \
    --target-success 0.5 0.7 \
    --ledger ./scenario_runs

# 4. replay a discovered failure exactly
python -m scenario_gen.run --genome ./scenario_runs/gen12/scn03.genome.json --seed 42
```

### Configurable inputs

| input | flag / param | default |
|---|---|---|
| algorithm-under-test launch cmd | `--aut` | a built-in deterministic stub (for smoke tests) |
| proposer strategy | `--proposer {rl,llm,hybrid}` | `rl` |
| stress-score weights | `score_weights` rosparam / `--weights` | localization=collisions=goal=tracking=1.0 |
| curriculum success band | `--target-success LO HI` | `0.5 0.7` |
| LLM model | `--llm-model` | `claude-opus-4-8` |
| compute budget | `--generations` × `--per-generation` | `10 × 5` |

> **Security.** A rendered world is *flat YAML with no Lua*, so a generated
> scenario is data, not code. But the genome's `model:`/`map:` references resolve
> against the chosen template directory only — render rejects paths that escape
> it. Continue to load only templates you trust (see `.claude/rules/common/security.md`).
