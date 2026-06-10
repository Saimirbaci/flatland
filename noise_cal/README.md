# noise_cal — offline calibration for Flatland's context-conditioned noise

`noise_cal` fits the **`parametric_linear`** noise model
([docs/noise_model_format.md](../docs/noise_model_format.md)) from labeled real
(or synthetic) sensor/odometry residual data and exports the versioned JSON that
`flatland_server/noise_model.cpp` loads at runtime. Like `scenario_gen/`, it is
**pure stdlib** (no numpy/pandas), so it runs in CI without ML dependencies.

## Why

Flatland's baseline sensor/drive noise was a single static Gaussian `std`. Real
noise grows with **speed**, depends on the **surface**, worsens in poor
**lighting**, and degrades with sensor **age**. This toolkit calibrates that
dependence from data so the simulated baseline noise is realistic — and the
fault-injection framework then layers faults on top of a realistic baseline.

## Input data

A CSV of residuals (`measured - ground_truth`) with paired context labels:

| column       | meaning |
|--------------|---------|
| `channel`    | which stream (e.g. `range`, `odom_pose_yaw`) |
| `surface_id` | surface bucket under the body (0 = dry/nominal) |
| `speed`      | body speed [m/s] |
| `lighting`   | ambient lighting 0..1 |
| `age`        | sensor age [hours] |
| `residual`   | observed noise sample for that channel |

These come from rosbag post-processing (align estimate vs. ground truth, label
context) or — for testing/CI — the built-in synthetic generator.

## Calibrate → export → load

```bash
# 1a. From real labeled data:
python -m noise_cal.fit --data residuals.csv --out model.json --report

# 1b. Or exercise the whole pipeline with synthetic data (no bags needed):
python -m noise_cal.fit --synthetic /tmp/synth.csv --n 4000 --out model.json --report

# 2. Reference the file from a model/world YAML plugin:
#    - type: Laser
#      noise_model: model.json      # relative to the model dir / absolute
#      sensor_age_hours: 800

# 3. flatland_server/noise_model.cpp loads + validates it (schema_version 1).
#    A missing/invalid file logs a warning and falls back to legacy constant
#    std, so a bad model never breaks world load.
```

`scenario_gen` can also attach a calibrated model when rendering worlds (see the
`noise_model_path` / `lighting` / `sensor_age_hours` knobs in `param_space.json`).

## How the fit works

For each channel, `fit.py`:

1. Fits a **mean** model `residual ~ 1 + speed + age + darkness + surface` by
   ordinary least squares (a small pure-Python normal-equations solver).
2. Centers the residuals, forms the half-normal std estimate
   `|r - mean| * sqrt(pi/2)`, and fits the **std** model with the same design.

The base surface (smallest id) is folded into the intercept; other surfaces get
offsets relative to it — matching the C++ evaluation exactly.

## Tests

```bash
pytest noise_cal/tests          # synthetic round trip + coefficient recovery
```
