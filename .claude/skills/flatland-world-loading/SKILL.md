---
name: flatland-world-loading
description: Understand and modify how Flatland loads worlds, models, and layers from YAML — including the Lua preprocessor pass and YamlReader typed parsing. Invoke when touching config schema, world/model spawning, or YAML error handling.
---

# World / model / layer loading pipeline

A simulation is defined by a **world YAML** that references **layer** images (static map) and
**model** YAMLs; each model lists **plugins**. The load path:

```
world.yaml ──(Lua preprocess)──> parsed YAML ──> World ──> Layers + Models ──> Plugins
```

## Files
- `flatland_server/src/yaml_preprocessor.cpp` (+ `.h`) — **runs first**. Evaluates embedded **Lua**
  in the YAML: `$eval` expressions, env-var substitution. This is why a world file is effectively
  executable — treat untrusted worlds as code.
- `flatland_server/src/yaml_reader.cpp` (+ `include/.../yaml_reader.h`) — typed accessor layer
  (`YamlReader`, `Get<T>`, `SubNode`, defaults). All config reads should go through it; it throws
  `YAMLException` (`exceptions.h`) with file + field context.
- `flatland_server/src/world.cpp` — builds the `b2World`, then loads layers and models from the
  parsed world node.
- `flatland_server/src/model.cpp` — parses a model YAML: bodies, joints, plugins.
- `flatland_server/src/layer.cpp` — loads the map image for a layer (see `flatland-map-layers`).

## To add or change a config field
1. Find where the sibling fields are read (a `YamlReader` call in `world.cpp` / `model.cpp` /
   the relevant plugin's `OnInitialize`).
2. Add `reader.Get<T>("key", default)` — provide a default or throw `YAMLException` if required.
3. If it's preprocessable, make sure it survives the `yaml_preprocessor` pass.
4. Update an example world under `flatland_server/test/` and add/extend a `load_world_test` case.

## Reference worlds & tests
- `flatland_server/test/conestogo_office_test/world.yaml` — the default world `server.launch` loads.
- `flatland_server/test/load_world_tests/`, `load_world_test.cpp` / `.test` — loader coverage.
- `flatland_server/test/yaml_preprocessor/` — Lua-preprocessing coverage.

## Gotchas
- Don't read raw `YAML::Node` — you lose the file/field context in errors. Use `YamlReader`.
- Relative paths in a world resolve against the world directory; keep that contract for new loaders.
