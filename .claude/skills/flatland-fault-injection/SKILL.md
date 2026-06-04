---
name: flatland-fault-injection
description: Understand and extend Flatland's YAML-driven fault-injection framework — the FaultInjector world plugin, the FaultInjectionRegistry singleton, sensor/drive perturbation hooks, the sealed out-of-band ground truth, and the offline RCA scorer. Invoke when adding a new fault kind, wiring a plugin into the registry, or touching the ground-truth/scoring path.
---

# Flatland fault injection

A YAML-driven framework that perturbs sensor/drive output at runtime while sealing the
ground-truth label **out of band**, so a root-cause-analysis (RCA) consumer must rediscover faults
from in-band signals only. Full contract + taxonomy: `flatland_plugins/doc/fault_injection.md`.

## How it fits together
- **`FaultInjector`** (`src/fault_injector.cpp`) — a **WorldPlugin** (registered in
  `flatland_plugins.xml` + `PLUGINLIB_EXPORT_CLASS`). Parses the `faults:` list from the world YAML,
  evaluates triggers + severity ramps each `BeforePhysicsStep`, writes the active per-step effects
  into the registry, and emits ground truth **only** to the reserved topic `/_ground_truth/faults`
  (`flatland_msgs/FaultGroundTruthArray`) and a sealed JSON sidecar (`ground_truth_path`).
- **`FaultInjectionRegistry`** (`src/fault_injection_registry.cpp`, `include/.../fault_injection_registry.h`)
  — process singleton (mirrors `RngManager`): single writer per step, copy-on-read `GetEffect`. Also
  hosts the pure, unit-testable math: `SeverityAt`, `ConditionMet`, `ParseFaultKind`,
  `ParseRampProfile`, `ComponentKey`.
- **Sensor/drive hooks** — `imu.cpp`, `laser.cpp`, `gps.cpp`, `bumper.cpp`, `diff_drive.cpp`,
  `tricycle_drive.cpp` query the registry by `ComponentKey(model, plugin)` just before
  publishing/commanding and perturb **only** their normal output. **No active effect → byte-for-byte
  identical to a clean run** — preserve this invariant in every change.
- **Sealing** — `launch/record_rca_bag.launch` records `-a --exclude /_ground_truth.*`;
  `scripts/fault_eval/score_rca.py` (stdlib-only, offline) is the *only* label consumer. Never embed
  the label in a sensor message.

## Step-order subtlety
World plugins step *after* model plugins, so `AfterPhysicsStep` consumers (imu) see the same-step
effects, while `BeforePhysicsStep` consumers (laser, diff_drive) read the previous step's effects
(≈one dt of lag — documented, not a bug).

## Adding a new fault kind
1. Add the enum value in `include/flatland_plugins/fault_injection_registry.h` (`FaultKind`).
2. Map the YAML `type` string in `ParseFaultKind` (`src/fault_injection_registry.cpp`).
3. Implement the perturbation in each applicable plugin's fault hook (e.g. `Laser::ApplyLaserFaults`),
   scaling the effect by `severity` and doing nothing when there is no active effect.
4. Document it in the taxonomy table of `flatland_plugins/doc/fault_injection.md`.
5. Add coverage in `test/fault_injection_test.cpp` (pure math) and `test/sensor_fault_test.cpp` +
   `sensor_fault_test.test` (behavioral), with worlds under `test/sensor_fault_tests/`.

## Two reference patterns from the sensor-fault work
- **`stuck`** — cache the last published message (`last_ranges_` / `last_scan_valid_` in `laser.h`)
  and republish it while the fault is active.
- **`latency`** — buffer finished messages in a bounded, **sim-time-keyed FIFO** (`latency_queue_`,
  `kMaxLatencyQueue` in `laser.h`); release each once the `Timekeeper` clock reaches
  `capture_sim_time + severity·latency`, keeping the **original `header.stamp`**. Drive it from sim
  time, never wall clock, and always drain matured entries (even on a dropped step).

## Tests
`test/fault_injection_test.cpp` (gtest core) · `test/fault_injector_test.cpp`/`.test` (rostest;
includes the void-benchmark guard that the bag exclusion drops the GT topic but keeps in-band ones) ·
`test/sensor_fault_test.cpp`/`.test` (per-sensor perturbation behavior).
