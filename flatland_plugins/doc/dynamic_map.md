# DynamicMap — mid-episode dynamic map changes

`DynamicMap` is a **world plugin** that mutates a named static layer's collision
geometry *during* an episode, so the static environment itself evolves within a
run — a shelf moves, a corridor closes — to test mapping/localization robustness
to non-static worlds. It mirrors the `FaultInjector` contract: a YAML-scripted,
time-triggered timeline of edits, applied deterministically, with a sealed
out-of-band ground-truth manifest.

## How it works

* On each step (`BeforePhysicsStep`) the plugin compares elapsed sim time
  (driven by the `Timekeeper`, never wall time) against each event's trigger.
* When an event fires it **read-modify-writes the target layer's in-memory
  occupancy bitmap** with OpenCV rectangle ops in world-metric coordinates, then
  calls `flatland_server::Layer::RebuildCollisionFromBitmap`, which destroys the
  layer's current Box2D fixtures and re-runs the contour → `b2ChainShape`
  pipeline against the new bitmap, and refreshes the cached `OccupancyGrid`.
* The latched per-layer occupancy overlay is then re-published so rviz / the
  diagnostic panel reflect the new map.

Geometry is swapped **only between physics steps and only on the fire edge** —
never inside a Box2D contact callback (destroying fixtures mid-solve is unsafe)
and never per step — so there is zero per-step cost once all events have fired.

## YAML schema

```yaml
plugins:
  - type: DynamicMap
    name: dynamic_map
    target_layer: "static"        # name of the layer whose bitmap is mutated
    manifest_path: "/tmp/flatland_dynamic_map_ground_truth.json"  # optional
    seed_key: "dynamic_map"       # optional, recorded in the manifest
    events:
      # "A corridor closes": fill a rectangular region with obstacle.
      - id: close_corridor
        trigger:
          time_s: 1.0             # sim-time onset threshold (elapsed seconds)
        op: fill                  # fill | clear | translate
        region: [4.0, 0.0, 1.0, 10.0]   # world [x, y, w, h] in meters
                                        # (x, y = lower-left corner)
      # "A shelf moves": clear a source rect and stamp it at a destination.
      - id: move_shelf
        trigger:
          time_s: 2.0
        op: translate
        region: [1.0, 1.0, 1.0, 1.0]    # source rectangle (world meters)
        dest: [7.0, 7.0]                # destination lower-left (world meters)
```

### Ops

| `op`        | Effect |
|-------------|--------|
| `fill`      | Mark the `region` rectangle occupied (an obstacle appears / a corridor closes). |
| `clear`     | Mark the `region` rectangle free (a corridor opens / an obstacle disappears). |
| `translate` | Snapshot the `region` rectangle, clear it (now free), and stamp it at `dest` (same width/height). Models a rigid object moving. |

Regions are given in **world meters** with the lower-left corner first, using the
same transform the layer's `LoadFromBitmap` / `BuildOccupancyGrid` assume
(`col = (x − origin.x) / resolution`, `row = rows − (y − origin.y) / resolution`).
Regions are clamped to the map bounds.

## Determinism & ground truth

The scripted timeline is fully deterministic by construction — the same world
and seed produce byte-identical geometry edits at the same sim times. The run
seed (from `flatland_server::RngManager`) is recorded for provenance. The applied
ops are sealed **out-of-band** to a JSON manifest (`manifest_path`) with each
event's resolved `applied_at_s`, op, and world region — so mapping/localization
evaluation knows exactly how the map evolved without that signal leaking into the
in-band sensor stream.

For deterministic collision geometry pin `simplify_map` (0 = none) for the run.

## Security note

`DynamicMap` adds **no new Lua/filesystem capability** beyond what already
exists. Like the `MapMutator` manifest dump, it only writes the optional
`manifest_path` sidecar. As always, world YAML is executable input (see the Lua
preprocessor) and should only be loaded from trusted sources.
