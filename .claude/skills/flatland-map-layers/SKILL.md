---
name: flatland-map-layers
description: Understand Flatland's map/layer loading — turning an occupancy image into Box2D collision geometry via OpenCV findContours + simplify_map → chain loops, plus the procedural map-mutation engine and runtime mid-episode geometry rebuild. Invoke for map loading, layer config, the simplify_map performance path, deterministically perturbing/mutating a layer's occupancy map at load time, or changing a layer's collision geometry mid-episode (the DynamicMap world plugin).
---

# Map layers → Box2D collision geometry

A **layer** is a static map: an image plus a YAML descriptor, turned into Box2D edges the robot can
collide with. This is the recently optimized hot path (commits: OpenCV `findContours` replacing
custom vectorization, Box2D **chain loops** replacing per-segment edges, and `simplify_map`).

## File
- `flatland_server/src/layer.cpp` (+ `include/flatland_server/layer.h`) — owns the whole pipeline.

## Pipeline
1. Read the layer's image (occupancy bitmap) with **OpenCV**, convert to `CV_32FC1` in `[0,1]`.
2. *(Optional)* **Procedural map mutation** — if a `mutation:` block is enabled, the bitmap is
   deterministically perturbed here, between `imread`/`convertTo` and any geometry build (see below).
3. Extract obstacle outlines with `cv::findContours`.
4. **`simplify_map`** reduces contour vertex count (Douglas-Peucker-style). Controlled by the
   `simplify_map` arg: `0` = none, `1` = moderate, `2` = significant (see `launch/benchmark.launch`).
   Higher = fewer Box2D edges = faster stepping, at some shape fidelity cost (~10–20% speedup, and
   baseline-faster even at 0 per the commit notes).
5. Build Box2D **chain-loop** fixtures (`b2ChainShape` loops) from the simplified contours and attach
   them to the layer's static body, registering collision bits via `collision_filter_registry`.

The same `cv::Mat` also feeds the `nav_msgs/OccupancyGrid` the layer publishes, so a mutation in
step 2 is reflected in **both** the physics edges and the published grid — no per-view divergence.

## Procedural map mutation
A `mutation:` block (a sibling of `map:`/`color:`/`properties:` in a layer entry) deterministically
perturbs the occupancy bitmap at load time, so mapping/localization can be tested against *novel*
maps rather than one fixed collected map. **Absent or `enabled: false` ⇒ the map loads
byte-for-byte as before** — the default `MutationConfig{}` is a no-op clone.

- **Code:** `flatland_server/src/map_mutator.cpp` (+ `include/.../map_mutator.h`) — the pure,
  bitmap-in/bitmap-out engine: `MapMutator::{FromConfig, Apply, WriteManifest}`. No ROS graph, no
  Box2D, unit-testable in isolation. Invoked from `Layer::MakeLayer` (`layer.cpp`); the `mutation:`
  block is parsed in `World::LoadLayers` (`world.cpp`), which also normalizes `manifest_path` /
  `dump_path` relative to the world YAML dir.
- **Ops** (each sub-block optional): `wall_jitter` (bounded rigid shift/rotate of the whole map),
  `aisle_width` (morphological dilate/erode — positive delta narrows aisles, negative widens),
  `clutter` (add blobs in free space / remove small non-structural components — `max_component_size_m`
  keeps load-bearing walls intact), `obstacle_density.target_scale` (multiplier on the clutter
  add-count). All ranges are validated (`min ≤ max`, non-negative) and throw `YAMLException`.
- **Determinism:** all randomness flows through `RngManager` (`random.h`). The layer derives a
  private `std::default_random_engine` via `RngManager::Get().DeriveEngine(seed_key)` — `seed_key`
  defaults to the layer name. Identical `(global seed, seed_key, config, image)` ⇒ byte-identical
  mutated map; distinct layers get independent streams. Set the run seed via the `seed` node param
  or world `properties.seed`.
- **Sealed manifest (out-of-band ground truth):** when `manifest_path` is set, a JSON sidecar records
  the global/derived seed, `seed_key`, layer name, and ordered op records. Like the fault-injection
  contract (`[[flatland-fault-injection]]`), it is **written out-of-band and consumed offline only**
  — never published in-band — so an evaluator knows which novel map a run used without the
  system-under-test observing it. Write failures warn, never throw.
- **Full schema + worked example:** `flatland_server/doc/map_mutation.md`. Tests:
  `test/map_mutator_test.cpp` (pure gtest) + `test/map_mutator_integration_test.{cpp,test}` (rostest,
  worlds under `test/map_mutator_tests/`).

## Runtime mid-episode mutation (vs. the load-time mutator above)
The `MapMutator` block perturbs the bitmap **once, at load**. To change a layer's geometry *during* a
run (a shelf moves, a corridor closes — testing mapping/localization against a non-static world), use
the runtime rebuild path instead:

- **`Layer::RebuildCollisionFromBitmap(bitmap, occupied_thresh, resolution)`** (`layer.cpp` /
  `layer.h`) — atomically swaps an image-based layer's collision geometry **in place**, without
  recreating the `Body` or `Layer`, so World ownership, the layer name maps, and any cached `Body*` /
  debug-viz handle stay valid. It destroys every fixture on the static body, re-runs the
  `findContours → simplify → b2ChainShape` pipeline (`LoadFromBitmap`) against the new bitmap,
  rebuilds the cached `OccupancyGrid` (`BuildOccupancyGrid`) using the **stored** layer origin, and
  caches the new bitmap. **MUST be called only between physics steps** (e.g. a world plugin's
  `BeforePhysicsStep`), never inside a Box2D contact callback — destroying fixtures mid-solve is
  unsafe. See the `flatland-physics-box2d` skill.
- **Layer now caches its bitmap + transform** (`bitmap_`, `resolution_`, `occupied_thresh_`,
  `origin_`, with `GetBitmap()`/`GetResolution()`/`GetOccupiedThresh()`/`GetOrigin()` accessors) so a
  mutator can read-modify-write the current map. These are populated only for **image-based** layers;
  `GetBitmap()` is empty for line-segment/empty layers (callers must treat that as "no edits apply").
  **Callers clone before mutating** — `GetBitmap()` returns a const ref to the live cache.
- **`World::GetLayer(name)`** (`world.cpp`) resolves a loaded layer by any of its registered names
  (aliases included); **`World::RepublishLayerOccupancy()`** (a thin wrapper over
  `PublishDiagnostics()`) re-latches the occupancy overlay so rviz/diagnostics reflect the new map.
- **The `DynamicMap` world plugin** (`flatland_plugins/src/dynamic_map.cpp`, doc:
  `flatland_plugins/doc/dynamic_map.md`) is the worked consumer: a YAML-scripted, sim-time-triggered
  timeline of `fill` / `clear` / `translate` rect ops in world-metric coords, applied on the **fire
  edge only** (zero per-step cost once all events fired), with a sealed out-of-band JSON manifest
  (`manifest_path`) of the applied timeline — mirroring the `[[flatland-fault-injection]]` ground-truth
  contract. World→pixel transform: `col = (wx − origin.x)/res`, `row = rows − (wy − origin.y)/res`,
  clamped to the image. Tests: `test/dynamic_map_test.{cpp,test}`, world under `test/dynamic_map_tests/`.
- For **deterministic** runtime geometry, pin `simplify_map` (0 = none) for the run.

## When you change this
- Tune or extend `simplify_map`: keep the `0/1/2` arg semantics stable (launch files + tests rely on it).
- Validate against a real map: `roslaunch flatland_server server.launch use_rviz:=true` and confirm
  the collision outline (via `debug_visualization`) still matches the image.
- Benchmark before/after: `roslaunch flatland_server benchmark.launch simplify_map:=<0|1|2>`
  (add `use_perf:=true` for a perf profile). Test world: `flatland_server/test/benchmark_world/`.

## Gotchas
- Over-simplification can open gaps in thin walls — check fidelity, not just FPS.
- Chain loops are open vs. closed depending on contour; keep winding consistent so normals face the
  free space.
- Map image is untrusted input — handle missing/corrupt files with a clear error, don't crash.
- Map mutation runs on the `CV_32FC1` bitmap **before** `findContours`, so keep mutation magnitudes
  small (a few cm / few degrees) — over-mutating can disconnect corridors or delete thin walls just
  like over-simplification. Never emit the mutation manifest in-band; it is offline-only ground truth.
