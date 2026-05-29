---
name: flatland-map-layers
description: Understand Flatland's map/layer loading — turning an occupancy image into Box2D collision geometry via OpenCV findContours + simplify_map → chain loops. Invoke for map loading, layer config, or the recent map-simplification performance path.
---

# Map layers → Box2D collision geometry

A **layer** is a static map: an image plus a YAML descriptor, turned into Box2D edges the robot can
collide with. This is the recently optimized hot path (commits: OpenCV `findContours` replacing
custom vectorization, Box2D **chain loops** replacing per-segment edges, and `simplify_map`).

## File
- `flatland_server/src/layer.cpp` (+ `include/flatland_server/layer.h`) — owns the whole pipeline.

## Pipeline
1. Read the layer's image (occupancy bitmap) with **OpenCV**.
2. Extract obstacle outlines with `cv::findContours`.
3. **`simplify_map`** reduces contour vertex count (Douglas-Peucker-style). Controlled by the
   `simplify_map` arg: `0` = none, `1` = moderate, `2` = significant (see `launch/benchmark.launch`).
   Higher = fewer Box2D edges = faster stepping, at some shape fidelity cost (~10–20% speedup, and
   baseline-faster even at 0 per the commit notes).
4. Build Box2D **chain-loop** fixtures (`b2ChainShape` loops) from the simplified contours and attach
   them to the layer's static body, registering collision bits via `collision_filter_registry`.

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
