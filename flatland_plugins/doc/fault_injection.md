# Fault-Injection Framework with Sealed Out-of-Band Ground Truth

YAML-driven, time- and condition-triggered fault injection for Flatland. A fault
manifests **only** as a realistic perturbation of the normal signals the robot
publishes (sensor topics, `odom`, `tf`, drivetrain response) — exactly the
in-band data a Root-Cause-Analysis (RCA) agent must reason over. The
ground-truth label (fault type, onset, duration, severity, affected component)
is written to a **sealed, out-of-band** sidecar manifest and a reserved-namespace
topic, both of which are **excluded** from the RCA bag/stream. The label is
consumed *only* by the offline evaluation harness for scoring.

> **CRITICAL CONTRACT.** If the RCA can see the label, the benchmark is void.
> The in-band sensor/odom/tf messages are byte-for-byte schema-identical to a
> clean run — no extra fields, no extra topics. The only ground-truth surfaces
> are the sidecar file and the reserved `/_ground_truth/...` namespace, which the
> recording config excludes and an automated test verifies are absent from the
> bag.

## Components

| Piece | Location | Role |
|-------|----------|------|
| `FaultInjector` (WorldPlugin) | `flatland_plugins/src/fault_injector.cpp` | Parses the `faults:` list, evaluates triggers + severity ramps each step, writes effects into the registry, and emits ground truth **only** out-of-band. |
| `FaultInjectionRegistry` (singleton) | `flatland_plugins/src/fault_injection_registry.cpp` | In-process store of the per-step active fault effects. Written once per step by `FaultInjector`, read by sensor/drive plugins. Also hosts the pure ramp/trigger math (`SeverityAt`, `ConditionMet`). |
| Sensor/drive hooks | `imu.cpp`, `laser.cpp`, `gps.cpp`, `diff_drive.cpp`, `tricycle_drive.cpp` | Query the registry just before publishing/commanding and perturb **only** their normal output. No active effect → exact previous behavior. |
| `FaultGroundTruth[Array]` msgs | `flatland_msgs/msg/` | The sealed label schema. Published *only* on the reserved topic; never embedded in any sensor message. |
| RCA bag launch | `flatland_plugins/launch/record_rca_bag.launch` | `rosbag record` that excludes `/_ground_truth.*` by prefix. |
| Offline scorer | `flatland_plugins/scripts/fault_eval/score_rca.py` | The *only* consumer of the label. Turns (sealed truth, RCA hypotheses) into metrics. |

## Data flow (one timestep)

```
                       model plugins                world plugins
  BeforePhysicsStep:  [laser/gps/diff_drive] ----> [FaultInjector]
                          read registry  <----------- SetEffects(snapshot)
                          perturb in-band             publish /_ground_truth/faults
                          output (scan/odom/tf)       write sidecar manifest (on change)
```

`FaultInjector` is a *world* plugin and the plugin manager steps world plugins
**after** model plugins, so the effects a sensor reads in step *N* are the ones
the injector wrote in step *N-1*. Severity changes over ramps measured in
seconds, so this one-step (≈`dt`) pipeline latency is negligible and
deterministic. The first step is primed with an empty registry (no effect).

## World YAML schema

The `FaultInjector` is a world plugin, declared under the top-level `plugins:`
list of a **world** file:

```yaml
plugins:
  - type: FaultInjector
    name: faults
    ground_truth_topic: "/_ground_truth/faults"   # reserved namespace (default)
    ground_truth_path: "/tmp/flatland_fault_ground_truth.json"  # sealed sidecar
    faults:
      - id: laser_drift_1
        type: sensor_drift           # see "Fault taxonomy" below
        target:
          model: robot               # model name
          component: laser_front     # the plugin's `name`
          topic: scan                # documentation only (not used for routing)
        trigger:
          kind: time                 # time | condition
        severity:
          onset_time: 5.0            # sim seconds since world start
          ramp_up: 2.0               # s to climb to peak
          hold: 10.0                 # s at peak (<0 = indefinite, never ends)
          ramp_down: 2.0             # s to fall back to 0
          peak: 0.8                  # peak severity in [0,1]
          profile: linear            # step | linear | exp
        params:
          bias: 0.5                  # fault-specific magnitude(s), see taxonomy

      - id: veer_after_5m
        type: asymmetric_drive
        target: { model: robot, component: diff_drive }
        trigger:
          kind: condition
          condition: distance_travelled   # see "Triggers"
          threshold: 5.0                   # metres
          depends_on: laser_drift_1        # optional fault chaining
        severity: { onset_time: 0.0, ramp_up: 1.0, hold: -1, ramp_down: 0.0,
                    peak: 1.0, profile: linear }
        params: { yaw_bias: 0.4 }
```

`onset_time` for a **condition** trigger is ignored; the onset is latched to the
sim time at which the condition first becomes true, and the ramp is measured from
that latched instant.

### Severity ramp math — `severity(t) ∈ [0,1]`

Let `e = t − onset` (0 before onset). The window is
`ramp_up → hold → ramp_down`:

* `t < onset` → `0`
* `0 ≤ e < ramp_up` → climbing: `peak · shape(e / ramp_up)`
* peak phase (`hold < 0` ⇒ indefinite) → `peak`
* falling within `ramp_down` → `peak · shape(1 − e_down / ramp_down)`
* after the window → `0`

`shape(f)` per profile: `linear` → `f`; `exp` → eased
`(e^{k f} − 1)/(e^{k} − 1)`, `k=3`; `step` → the whole window is at `peak`
(ramps ignored). Severity linearly scales each fault's perturbation magnitude,
so `params` values are the **peak** magnitudes reached at `severity = 1`.

### Triggers

* **time** — fires on `severity.onset_time` (sim seconds since world start).
* **condition** — evaluated every step against world state; onset latches on the
  first true evaluation. Supported `condition` types:
  * `distance_travelled` — cumulative path length of the target model ≥ `threshold` (m)
  * `x_greater` / `y_greater` — model world x/y ≥ `threshold` (m)
  * `elapsed` — sim seconds since start ≥ `threshold` (equivalent to a time trigger)
  * `after_fault` — fires once the `depends_on` fault has become active
  * `depends_on` (optional, any condition) — additionally gate on the named
    fault having fired (fault chaining)

## Fault taxonomy

Each `type` maps to a `FaultKind`. Perturbations apply to the **in-band** output
only and scale with `severity`.

### Sensor faults
| type | applies to | `params` | effect |
|------|-----------|----------|--------|
| `sensor_bias`   | imu, gps | `bias` | additive constant offset |
| `sensor_drift`  | imu, gps | `bias` (per-second drift rate) | offset accumulates over active time |
| `sensor_scale`  | imu | `scale` | multiplicative gain error (`1+severity·scale`) |
| `noise_inflation` | imu, laser | `noise` (extra std-dev) | inflates Gaussian noise std |
| `dropout`       | imu, laser, gps | `dropout_prob` (0..1) | randomly skips publishing a message |
| `stuck`         | imu, laser | — | freezes (republishes the last) message |
| `quantization`  | imu | `step` | rounds output to a coarse grid |
| `laser_sector_occlusion` | laser | `sector_center`, `sector_width` (rad) | NaNs ranges inside an angular sector |

### Drivetrain faults
| type | applies to | `params` | effect (causal — flows through physics) |
|------|-----------|----------|------|
| `torque_loss` / `wheel_slip` | diff_drive, tricycle_drive | `loss` (0..1) | scales achieved drive velocity by `1 − severity·loss` |
| `asymmetric_drive` | diff_drive, tricycle_drive | `yaw_bias` (rad/s or rad) | injects a steering/yaw bias → robot veers |
| `deadband` | diff_drive | `deadband` | zeroes sub-threshold commands |
| `stuck_wheel` | diff_drive, tricycle_drive | — | drops drive velocity to ~0 |

Drivetrain faults are applied **after** the existing actuator/dynamics ramp and
**before** `SetLinearVelocity`/friction drive, and are clamped, so the resulting
`odom`/`tf`/motion reflect the fault through Box2D rather than a cosmetic edit.

## Sealed ground truth

* **Reserved topic** `/_ground_truth/faults` (`flatland_msgs/FaultGroundTruthArray`)
  — absolute name; never recorded into the RCA bag.
* **Sidecar manifest** (`ground_truth_path`, JSON) — rewritten only on state
  change (a fault latches its onset, or ends). Schema per fault:
  `fault_id, fault_type, affected_model, affected_component, affected_topic,
  onset_time, end_time, peak_severity, trigger_description`. `end_time < 0`
  while a fault is still active or indefinite.

The existing per-plugin `ground_truth_pub` (e.g. `imu/ground_truth`,
`odometry/ground_truth`) is a **clean-signal debug aid, NOT the sealed RCA
label** — it carries no fault metadata and may be recorded freely.

See `record_rca_bag.launch` for the exclusion config and
`scripts/fault_eval/score_rca.py` for the scoring contract.

## Producer / consumer contract

* **In-band (RCA may consume):** every robot topic — `scan`, `imu/filtered`,
  `gps/fix`, `odometry/filtered`, `twist`, `tf`, `/clock`, and the clean
  per-plugin `*/ground_truth` debug aids. These are recorded by
  `record_rca_bag.launch`.
* **Sealed (RCA must NOT consume):** the reserved `/_ground_truth/...` topic and
  the sidecar manifest file. Excluded from the bag; consumed only by the scorer.

### RCA hypothesis format (scorer input)

The RCA emits a JSON file of hypotheses, scored against the sealed manifest:

```json
{
  "hypotheses": [
    {
      "fault_type": "torque_loss",
      "affected_model": "robot",
      "affected_component": "diff_drive",
      "onset_time": 12.3,
      "end_time": -1.0,
      "peak_severity": 0.6,
      "detected_at": 13.1
    }
  ]
}
```

`onset_time`/`end_time`/`detected_at` are sim-seconds since world start, matching
the manifest. `end_time < 0` means "still active / indefinite".

### Metrics (computed by `score_rca.py`)

Hypotheses are greedily matched to ground-truth faults (same
type + component, maximal temporal overlap). Reported per scenario:
fault-type accuracy, affected-component accuracy, onset-time error,
duration/temporal IoU, peak-severity error, mean time-to-detect, and
precision/recall/F1 over all faults.
