---
name: flatland-map-layers
description: Use when working on Flatland map layers — loading occupancy images into Box2D collision geometry, the OpenCV findContours → chain-loop conversion, or the simplify_map performance path. This is where recent optimization work (10-20% speedups) happened.
---

# Map layers → collision geometry

A layer turns an occupancy-grid image into Box2D collision geometry. All of this lives in
`flatland_server/src/layer.cpp`.

## The conversion pipeline (as implemented)
1. Load the layer's occupancy image (resolution + origin come from the layer YAML).
2. Extract obstacle outlines with OpenCV:
   `cv::findContours(obstacle_map_open, vectors_outline, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE)`
   (`layer.cpp:268`).
3. Build Box2D **chain loops** (`b2ChainShape`, `layer.cpp:227`) from each contour and attach
   them as fixtures. (This replaced the older line-segment approach — see commit
   "Swapped out vectorization for opencv::findContours, and box2d line segments for chain loops".)

## `simplify_map` (performance knob)
- Controlled by the `simplify_map` rosparam (`layer.cpp:254-256`), default `0`:
  - `0` = no simplification (now the baseline-fast path),
  - `1` = moderate simplification of polygon outlines (`layer.cpp:276`),
  - `>=2` = maximum simplification (`layer.cpp:260`).
- Added in commit "Added simplify_map feature, as well as benchmark"; gives 10-20% speedups.

## When you change this path
- Units: convert pixels → metres using the layer resolution/origin; don't leak pixel coords
  into body coordinates (see `flatland-physics-box2d`).
- Validate with the benchmark: `roslaunch flatland_server benchmark.launch` against
  `flatland_server/test/benchmark_world/`, before and after, and report the delta.
- Don't trade collision fidelity for speed silently — higher `simplify` levels drop detail.
- Box2D chain shapes are owned by the world; create them through fixtures, never `delete`.
