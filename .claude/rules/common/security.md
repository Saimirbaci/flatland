# Security — Flatland

Flatland is a simulator, not a networked service — but it has one real, specific
attack surface worth guarding.

## World YAML can execute code
World files are run through `flatland_server/src/yaml_preprocessor.cpp`, which
evaluates **embedded Lua expressions** before parsing. Loading an untrusted world
file is therefore arbitrary code execution, not just data parsing.
- Treat any world/model/layer YAML from outside the repo as untrusted input.
- Don't expand the Lua surface (new globals, filesystem/network access from Lua)
  without a deliberate review — it widens this hole.
- When adding a service that loads a world/model by path (see
  `flatland_server/src/service_manager.cpp`), validate the path; don't blindly load
  caller-supplied filenames.

## Map / file paths
- Layer map images and YAML are resolved relative to the world file. Keep path
  handling in `yaml_reader`/`world.cpp`; don't construct paths from unvalidated input.

## Repo hygiene
- No secrets, tokens, or credentials belong in this repo — it's an open-source
  BSD-3 library (`LICENSE`). There is no auth/PII/payment data here; don't add any.
