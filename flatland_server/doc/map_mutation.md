# Procedural Map Mutation

Flatland can parametrically perturb a layer's static occupancy map **at load
time** so mapping/localization is tested against *novel* maps instead of one
fixed collected map. Walls are shifted/rotated within tolerance, corridors are
widened or narrowed, and clutter is added or removed — all deterministically
seeded so a given run is exactly reproducible.

The mutation runs once, in `Layer::MakeLayer`, on the `cv::Mat` occupancy
bitmap **between** `cv::imread` and the construction of the Box2D collision
geometry / `nav_msgs/OccupancyGrid`. Both the physics edges and the occupancy
grid therefore see the same mutated map automatically. There is **no per-step
cost** — this is a startup transform, not a plugin.

## Determinism

All randomness flows through the central seeded RNG authority
(`flatland_server/random.h`, `RngManager`; see `doc/determinism.md`). Each layer
derives a private engine from the global run seed and a stable **`seed_key`**
(the layer name by default), so identical `(seed, seed_key, config, image)`
inputs always yield a byte-identical mutated map, and distinct layers get
independent streams. Set the run seed via the `seed` node param or world
`properties.seed`.

## Schema

A `mutation:` block lives inside a layer entry, alongside `map:`, `color:`, and
`properties:`. **Absent or `enabled: false` ⇒ the map loads byte-for-byte as
before.** All numeric ranges are validated (`min ≤ max`, non-negative where
required) and throw a `YAMLException` with file/field context otherwise.

| Key | Type | Meaning |
|-----|------|---------|
| `enabled` | bool | Master switch. Default `false`. |
| `seed_key` | string | RNG stream key override. Default: the layer name. |
| `manifest_path` | string | Optional sealed JSON sidecar path (see below). Relative paths resolve against the world YAML dir. |
| `dump_path` | string | Optional path to dump the mutated PNG for inspection. |
| `wall_jitter.max_translation_m` | float ≥ 0 | Max rigid shift of the whole map (meters). |
| `wall_jitter.max_rotation_deg` | float ≥ 0 | Max rotation about the image center (degrees). |
| `aisle_width.dilate_erode_range_m` | `[min, max]` | Sampled corridor delta. **Positive ⇒ dilate obstacles (narrow aisles); negative ⇒ erode (widen aisles).** |
| `clutter.add_count_range` | `[min, max]` ints ≥ 0 | Number of obstacle blobs to add in free space. |
| `clutter.remove_fraction` | float `[0,1]` | Fraction of *small* (non-structural) components to clear. |
| `clutter.blob_size_m_range` | `[min, max]` ≥ 0 | Added-blob radius range (meters). |
| `clutter.max_component_size_m` | float ≥ 0 | A component is removable (and a blob never larger) only if its bounding box ≤ this. Keeps structural walls intact. Default `0.5`. |
| `obstacle_density.target_scale` | float ≥ 0 | Multiplies the sampled clutter add-count (`>1` denser, `<1` sparser). |

Each present op sub-block is itself optional; omit one to skip that mutation.

### Tolerance semantics

Magnitudes are intentionally bounded so the map stays recognizable and
traversable: `wall_jitter` is a single rigid transform within the configured
translation/rotation envelope; `aisle_width` uses a morphological kernel sized
from `dilate_erode_range_m`; clutter removal only touches components whose
bounding box fits within `max_component_size_m`, so load-bearing walls are never
deleted. Keep deltas small (a few cm / few degrees) to avoid disconnecting
corridors.

## Sealed manifest (out-of-band ground truth)

When `manifest_path` is set, a JSON sidecar is written recording the global
seed, the derived per-layer seed, the `seed_key`, the layer name, and the
ordered list of applied operations (type, magnitude, pixel location, detail).
This mirrors the fault-injection framework's sealed ground-truth contract (see
`flatland_plugins/doc/fault_injection.md`): it is **written out-of-band and
consumed offline only** — never published on an in-band topic — so a
mapping/localization evaluator knows exactly which novel map a run used without
the system-under-test being able to observe it. Write failures warn rather than
crash.

## Worked example

```yaml
properties:
  seed: 42          # makes the mutation (and manifest) reproducible

layers:
  - name: "2d"
    map: "map.yaml"
    color: [0, 1, 0, 1]
    mutation:
      enabled: true
      manifest_path: "/tmp/run.mutation.json"
      dump_path: "/tmp/run.mutated.png"
      wall_jitter:
        max_translation_m: 0.10
        max_rotation_deg: 2.0
      aisle_width:
        dilate_erode_range_m: [-0.05, 0.05]
      clutter:
        add_count_range: [3, 6]
        remove_fraction: 0.25
        blob_size_m_range: [0.10, 0.30]
        max_component_size_m: 0.5
      obstacle_density:
        target_scale: 1.0
```

## Security note

World/model YAML is **code, not data** — `yaml_preprocessor.cpp` evaluates
embedded Lua before parsing. Map mutation adds no new capability (it is numeric
config only), but as always **only load worlds from trusted sources**.

## Code pointers

- `flatland_server/include/flatland_server/map_mutator.h` — `MutationConfig`,
  `MutationManifest`, `MapMutator::{FromConfig,Apply,WriteManifest}`.
- `flatland_server/src/map_mutator.cpp` — OpenCV perturbation ops.
- `flatland_server/src/layer.cpp` — invocation in `Layer::MakeLayer`.
- `flatland_server/src/world.cpp` — `mutation:` parsing in `World::LoadLayers`.
- `flatland_server/test/map_mutator_test.cpp` — pure unit tests.
