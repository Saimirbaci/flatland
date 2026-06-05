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
| Sensor/drive hooks | `imu.cpp`, `laser.cpp`, `gps.cpp`, `bumper.cpp`, `diff_drive.cpp`, `tricycle_drive.cpp` | Query the registry just before publishing/commanding and perturb **only** their normal output. No active effect → exact previous behavior. |
| `LocalizationFault` (ModelPlugin) | `flatland_plugins/src/localization_fault.cpp` | Synthetic AMCL: publishes `amcl_pose` + the `map→odom` tf; injects `amcl_divergence` (estimate diverges from truth, odom untouched). |
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
| `sensor_bias`   | imu, gps, laser | `bias` | additive constant offset (laser: on every finite range) |
| `sensor_drift`  | imu, gps | `bias` (per-second drift rate) | offset accumulates over active time |
| `sensor_scale`  | imu | `scale` | multiplicative gain error (`1+severity·scale`) |
| `noise_inflation` | imu, laser, gps | `noise` (extra std-dev) | inflates Gaussian noise std (gps: on lat/lon) |
| `dropout`       | imu, laser, gps, bumper | `dropout_prob` (0..1) | randomly skips publishing a message |
| `stuck`         | imu, laser, gps, bumper | — | freezes (republishes the last) message |
| `quantization`  | imu | `step` | rounds output to a coarse grid |
| `laser_sector_occlusion` | laser | `sector_center`, `sector_width` (rad) | NaNs ranges inside an angular sector |
| `ghost_return`  | laser, bumper | `ghost_prob` (0..1); laser: `ghost_range` (m); bumper: `ghost_force` | laser: spurious finite returns in (preferentially empty) beams, clamped to `[range_min, range_max]`; bumper: appends a phantom collision (message fields only) |
| `latency`       | laser, gps, bumper | `latency` (s) | delays delivery by `severity·latency` (sim-time buffer, original stamp kept) — see the latency contract above |

#### Latency fault contract

The `latency` fault models delivery delay. When active, a sensor plugin pushes
each finished message (laser `LaserScan`, GPS `NavSatFix`, bumper `Collisions`)
into a small per-plugin FIFO queue keyed by a **sim-time** release instant
`release = capture_sim_time + severity·latency`, and only publishes it once the
`Timekeeper` sim clock reaches that release time. The buffered message keeps its
**original `header.stamp`** (the capture time) — it is *not* re-stamped to the
delivery time — so the RCA observes stale-but-schema-identical data (a growing
timestamp lag), the realistic in-band signature of latency. The queue is bounded
(old entries past a cap are flushed/published immediately so it cannot grow
without limit) and is driven entirely by sim time, never wall clock, so the
sealing and determinism guarantees are untouched. `severity = 0` (no active
effect) leaves the publish path byte-for-byte identical to a clean run.

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

#### Actuator-stage drivetrain faults

These four perturb the drivetrain by modulating the shared **actuator-dynamics
model** (`ActuatorDynamics`: the effort/force-torque cap and the command-latency
delay line) rather than post-multiplying the commanded velocity. They are still
fully **causal** — the true Box2D motion *and* the resulting `odom`/`tf` reflect
them — and ground truth is sealed out-of-band exactly as for the drivetrain
faults above. With no active effect the actuator pipeline is byte-for-byte the
clean-run pipeline.

| type | applies to | `params` | effect (acts through the actuator model) |
|------|-----------|----------|------|
| `motor_degradation` | diff_drive, tricycle_drive | `degrade` (0..1, default 1) | scales the actuator effort cap by `1 − severity·degrade`, lowering the achievable acceleration (`a_max = F/m`) and, in friction mode, the per-wheel motor force. **Requires** a configured `max_force`/`max_torque` to have any effect (it shrinks an existing cap). |
| `asymmetric_wheel_speed` | diff_drive | `imbalance` (0..1, default 1), `side` (`<0.5` = left/+y, else right/−y) | decomposes the commanded twist into per-wheel speeds via `wheel_separation`, scales one side by `1 − severity·imbalance`, and recomposes → a coupled linear drop **and** yaw drift (distinct from `asymmetric_drive`'s pure additive yaw bias). With no `wheel_separation` it falls back to an equivalent linear-drop + yaw-bias approximation. |
| `locked_wheel` | diff_drive | `side` (`<0.5` = left, else right) | scales one wheel's speed to `1 − severity` (≈0 at full severity) → a pivot; distinct from `stuck_wheel`, which drops *both* wheels. Same per-wheel recompose / fallback as `asymmetric_wheel_speed`. |
| `controller_latency` | diff_drive, tricycle_drive | `latency` (s) | adds `severity·latency` of transport deadtime to the actuator command-latency delay line for the duration of the fault, so the drive response to a `cmd_vel` step is delayed (on top of any configured `command_latency`). |

`asymmetric_wheel_speed` and `locked_wheel` are **diff_drive-only**: a tricycle
has a single front drive wheel, so a per-rear-wheel speed imbalance is not
physically meaningful there.

### Localization / odometry faults

These are the **localization-failure class**: the perturbation that motivates an
RCA over odom / IMU / localization signals. Unlike the drivetrain faults above
they are **measurement-domain, NOT causal** — the true Box2D motion is left
untouched; only the *reported* odom pose, encoder velocity, and the synthetic
localization estimate diverge from the ground truth. (Contrast `torque_loss` /
`wheel_slip`, which physically change how the robot moves.)

| type | applies to | `params` | effect (measurement-domain) |
|------|-----------|----------|------|
| `encoder_drift` | diff_drive, tricycle_drive | `x_rate`, `y_rate`, `yaw_rate` (per-second at severity 1) | reported odom pose dead-reckons away from truth at the given rate; true motion unchanged |
| `odom_slip` | diff_drive, tricycle_drive | `slip` | reported translation (odom delta + encoder twist) is scaled by `1 + severity·slip` (`>0` over-reports, `<0` under-reports) |
| `amcl_divergence` | `LocalizationFault` component | `x`, `y`, `yaw` | localization estimate (`amcl_pose`) and the `map→odom` tf diverge from truth by the severity-scaled offset; odom is untouched |

### Environment / dynamic-world faults

These are the **environment class**: instead of perturbing a published message,
the `FaultInjector` physically **mutates the running world** on the fault's
onset/end transitions — spawning a moving obstacle, relocating furniture, or
activating a low-friction spill region. The perturbation is therefore observable
**only through the robot's normal sensors** (a near laser return, a bumper
contact, traction loss in `odom`); there is no in-band annotation. Crucially
these kinds are **not** written into the `FaultInjectionRegistry` effect snapshot
(no sensor/drive plugin consumes them) — they act on `world_` directly. The
ground-truth label is still sealed out-of-band exactly as for every other kind.

Because the obstacle/spill *is* the signal, the only sealing requirement is that
the injected models carry **neutral names** (`obstacle_1`, never
`fault_person_injected`) so their normal `tf`/markers reveal nothing a
hand-authored world obstacle would not. The `FaultInjector` appends a neutral
numeric suffix to the configured `name` automatically.

| type | `params` | effect (mutates the world) |
|------|----------|------|
| `dynamic_obstacle` | `model` (path to obstacle `.model.yaml`), `x0`,`y0`,`yaw0` (spawn pose), `waypoints` (list of `[x,y]`, optional), `speed` (m/s, scaled by severity), `despawn_on_end` (0/1) | spawns a kinematic obstacle at the spawn pose on onset and drives it along its waypoints via Box2D velocity (so it can block/nudge the robot); on the end-edge it stops, and is removed if `despawn_on_end` |
| `moved_furniture` | `model` (path, used only if spawning), `to_x`,`to_y`,`to_yaw` (destination pose), `spawn_if_absent` (0/1) | relocates the `target.model` to the destination pose on onset; if absent and `spawn_if_absent`, spawns it there instead |
| `spill` | `center_x`,`center_y` (m), `radius` (m), `mu_min` (slipperiest multiplier at peak severity) | activates a circular low-friction overlay on the world's surface-friction field on onset (severity scales the multiplier toward `mu_min`); removed on the end-edge so grip recovers |

The `model` / `waypoints` params are strings/lists, so the environment kinds
parse their `params:` block with a dedicated reader; the numeric params above are
read through the same typed `YamlReader` path as every other fault.

For `dynamic_obstacle` the `target.model` is the **spawned** obstacle's base
name and `target.component` should be `environment`; for `spill` the
`target.component` is `environment` and `target.model` is the robot/area label.
The sealed manifest records `affected_component = environment` and a
`affected_topic` of `""` (or `scan` to hint the dominant observable channel).

**IMU bias** for a localization-failure scenario is **not a new type** — it is the
existing `sensor_bias` / `sensor_drift` applied to the `imu` component. Those
perturb the IMU orientation (yaw) and gyro (`wz`) channels (`sensor_drift`
accumulates a heading drift), exactly what an EKF/AMCL stack consumes.

**Onset & persistence semantics.** `encoder_drift` / `odom_slip` latch a
"diverged" state the first time either fires: until then the reported odom is
copied **byte-for-byte** from the ground truth (a clean run is unchanged). Once
latched, the dead-reckoned odom integrates the per-step truth delta with
slip/drift applied, and the accumulated error **persists after the fault window
closes** — a real odometry error does not heal itself. `amcl_divergence` instead
tracks the severity envelope directly (estimate offset grows with severity, then
holds), so it relaxes back toward truth if the fault ramps down.

#### The `LocalizationFault` plugin (synthetic AMCL)

Flatland ships no localization node, so the `LocalizationFault` **model plugin**
(`flatland_plugins/src/localization_fault.cpp`) stands in for one. Each step it
reads the body's true world pose and publishes:

* `amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`, in the `map_frame_id`
  frame) — the localization estimate. **In-band** (RCA may consume).
* the `map_frame_id → odom_frame_id` tf, computed as
  `estimate · truth⁻¹` in SE(2).

**Clean-run contract:** with no `amcl_divergence` active the estimate equals the
truth and `map→odom` is the **identity** transform. This assumes flatland's
diff_drive/tricycle odom is anchored at the spawn/world origin (so the `odom`
frame coincides with `map`). Config: `body`, `map_frame_id` (default `map`),
`odom_frame_id` (default `odom`), `amcl_pose_pub` (default `amcl_pose`),
`update_rate`, `publish_tf` (default `true`).

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
  `gps/fix`, `odometry/filtered`, `twist`, `amcl_pose`, `tf` (incl. `map→odom`),
  `/clock`, and the clean per-plugin `*/ground_truth` debug aids. These are
  recorded by `record_rca_bag.launch`.
* **Sealed (RCA must NOT consume):** the reserved `/_ground_truth/...` topic and
  the sidecar manifest file. Excluded from the bag; consumed only by the scorer.

**Environment faults add no in-band label.** A spawned obstacle / relocated
furniture / activated spill publishes only the *normal* model topics and Box2D
markers a hand-authored world object would (the obstacle **is** the signal). The
injected models carry neutral names (`obstacle_1`, …) and no fault-specific tf
frame or marker namespace, so nothing in-band distinguishes them from a static
world obstacle. The only fault metadata — type, onset, end, location, severity —
still flows exclusively to the sealed manifest + reserved topic, with
`affected_component = environment`. The exclude-by-prefix bag config records the
obstacle/spill as ordinary in-band world state and drops only `/_ground_truth.*`,
so no new topic or exclusion is required.

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
