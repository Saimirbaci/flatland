---
name: flatland-fault-injection
description: Work on Flatland's YAML-driven fault-injection framework — the registry singleton, the FaultInjector world plugin, per-sensor/drive perturbation hooks, the sealed out-of-band ground truth, and adding a new fault kind. Invoke when adding/changing a fault type, a sensor/drive fault hook, the localization/odometry faults, or the RCA ground-truth contract.
---

# Fault injection with sealed ground truth

A fault manifests **only** as a realistic perturbation of the normal signals the robot publishes
(sensor topics, `odom`, `tf`, drivetrain response) — the in-band data an RCA agent reasons over. The
ground-truth label (type, onset, duration, severity, component) is written **out-of-band** to a
sealed sidecar manifest and a reserved-namespace topic, both excluded from the RCA bag and consumed
only by the offline scorer.

> **CRITICAL CONTRACT.** If the RCA can see the label, the benchmark is void. In-band messages must
> stay byte-for-byte schema-identical to a clean run — no extra fields, no extra topics. The only
> ground-truth surfaces are the sidecar file and the reserved `/_ground_truth/...` namespace.

Full taxonomy + contract: **`flatland_plugins/doc/fault_injection.md`** — read it before changing
the schema or the producer/consumer split.

## Components
| Piece | Location |
|-------|----------|
| `FaultInjector` (WorldPlugin) | `flatland_plugins/src/fault_injector.cpp` — parses `faults:`, evaluates triggers + severity ramps each step, writes effects into the registry, emits ground truth out-of-band. |
| `FaultInjectionRegistry` (singleton) | `flatland_plugins/src/fault_injection_registry.cpp` (+ header) — per-step active-effect store; single writer per step, copy-on-read `GetEffect`. Hosts the pure helpers `SeverityAt`, `ConditionMet`, `ParseFaultKind`, `ParseRampProfile`, `ComponentKey`. |
| Sensor/drive hooks | `imu.cpp`, `laser.cpp`, `gps.cpp`, `bumper.cpp`, `diff_drive.cpp`, `tricycle_drive.cpp` — query the registry by `ComponentKey(model, plugin)` just before publishing/commanding and perturb **only** their normal output. No active effect → exact previous behavior. |
| `LocalizationFault` (ModelPlugin) | `flatland_plugins/src/localization_fault.cpp` — synthetic AMCL: publishes `amcl_pose` + `map→odom` tf; injects `amcl_divergence`. |
| Sealed label | `flatland_msgs/msg/FaultGroundTruth[Array]`, reserved topic `/_ground_truth/faults`, JSON sidecar (`ground_truth_path`). |
| Bag + scorer | `launch/record_rca_bag.launch` (records `-a --exclude /_ground_truth.*`); `scripts/fault_eval/score_rca.py` (stdlib-only, offline, the **only** label consumer). |

## Step-order invariant
`FaultInjector` is a **world** plugin; the plugin manager steps world plugins **after** model
plugins. So effects a `BeforePhysicsStep` sensor reads in step *N* are the ones the injector wrote
in *N-1* (≈`dt` lag, deterministic, documented). `AfterPhysicsStep` consumers (imu) read same-step
effects. First step is primed empty.

## Two fault classes — keep them distinct
- **Causal / drivetrain** (`torque_loss`, `wheel_slip`, `asymmetric_drive`, `deadband`,
  `stuck_wheel`): physically change how the robot moves — applied through Box2D before
  `SetLinearVelocity`/friction. The true motion *and* the reported odom both reflect it.
- **Measurement-domain** (`encoder_drift`, `odom_slip`, `amcl_divergence`): the true Box2D motion is
  **untouched**; only the *reported* odom pose / encoder twist / localization estimate diverge.
  - `encoder_drift` / `odom_slip` live in `diff_drive.cpp` / `tricycle_drive.cpp`. They **latch** a
    `odom_diverged_` state on first onset, seed the dead-reckoned pose from current truth, then
    integrate the per-step truth delta with slip/drift applied. Clean run (never fired) → odom
    copied **byte-for-byte** from truth. Error **persists** after the fault window (real odom error
    doesn't heal). See the `odom_*_`/`last_gt_*_`/`gt_sample_valid_` members.
  - **IMU bias is NOT a new kind** — it is the existing `sensor_bias`/`sensor_drift` on the `imu`
    component.

## Adding a new fault kind (the recurring change)
1. Add the enum value to `FaultKind` in `include/flatland_plugins/fault_injection_registry.h`.
2. Map the YAML `type:` string in `ParseFaultKind` (`fault_injection_registry.cpp`).
3. In the consuming sensor/drive plugin's step: `FaultInjectionRegistry::Get().GetEffect(fault_key_,
   FaultKind::kYours)`; if `effect.active`, scale by `effect.severity` and read tuned values via
   `effect.Param("name")`. **Guard the clean path** — no active effect must reproduce prior output
   exactly.
4. Document the type + params in `doc/fault_injection.md` (taxonomy table) — required.
5. Tests: a gtest for any pure math in the registry, plus a rostest (`.cpp` + `.test`) asserting the
   in-band perturbation and the clean-run invariant. See `test/odom_fault_test.*`,
   `test/localization_fault_test.*`, `test/sensor_fault_test.*`; worlds under
   `test/<...>_fault_tests/`. Wire targets in `CMakeLists.txt` with `add_rostest_gtest`.
6. If it's a new plugin (not a hook on an existing one), also register it in `flatland_plugins.xml`
   + `PLUGINLIB_EXPORT_CLASS` + add `src/<name>.cpp` to the library in `CMakeLists.txt` (see
   `flatland-plugin-authoring`).

## Never break
- **Never emit the label in-band.** No new topic outside `/_ground_truth/...`, no field on a sensor
  message. The rostest that verifies the bag excludes the GT topic must keep passing.
- **Never perturb when no fault is active** — the clean-run path stays byte-for-byte identical.
- Keep `score_rca.py` stdlib-only and offline (no ROS import).
