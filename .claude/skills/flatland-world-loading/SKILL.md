---
name: flatland-world-loading
description: Use when working on how Flatland loads worlds, models, or layers from YAML — the parsing pipeline, the Lua preprocessor, error reporting, or adding a new config field. Explains the world/model/layer YAML flow so you don't trace it from scratch.
---

# World / model / layer loading

Flatland builds a simulation from a tree of YAML files. The flow:

```
server.launch  →  world.yaml
  world.cpp        loads layers[] and models[]
    layer.cpp        each layer: occupancy image → Box2D collision geometry
    model.cpp        each model: bodies, joints, fixtures, and plugins[]
```

## Key files
- `flatland_server/src/yaml_preprocessor.cpp` — **runs first**: evaluates embedded **Lua**
  expressions in the YAML (string substitution / computed values) before parsing.
- `flatland_server/src/yaml_reader.cpp` (+ `include/flatland_server/yaml_reader.h`) — the
  typed accessor layer. Use its getters (with default/required semantics) instead of raw
  `YAML::Node` access; it attaches file + key context to errors.
- `flatland_server/include/flatland_server/exceptions.h` — `YAMLException` etc., thrown on
  missing/invalid fields. Throw these, don't silently default.
- `flatland_server/src/world.cpp` — top-level loader; owns the `b2World`, the layers, the models.
- `flatland_server/src/model.cpp`, `model_body.cpp`, `body.cpp`, `joint.cpp` — model subtree.
- `flatland_server/src/layer.cpp` — map layer (see `flatland-map-layers` skill).

## Adding a new config field
1. Read it via `YamlReader` in the relevant loader (`world`/`model`/`body`/`layer`).
2. Use a sensible default for backward compatibility, or throw `YAMLException` if required.
3. Add/extend a fixture world under `flatland_server/test/` (e.g. `load_world_tests/`) and a
   `load_world_test` assertion.

## Security note
World YAML can execute Lua via the preprocessor — loading an untrusted world is code
execution, not just parsing. See `.claude/rules/common/security.md`. Validate any
caller-supplied world/model path (`service_manager.cpp`) before loading.
