# Coding Style — Flatland

C++11, ROS 1 (roscpp). These are enforced by CI (`scripts/ci_prebuild.sh`); a fresh
session must follow them or the build gate fails.

## Formatting
- Run `clang-format --style=file` on every `.h`/`.cpp` before committing. The repo
  ships a `.clang-format`; do not override it.
- CI checks formatting across all tracked C/C++ files except `thirdparty/`:
  ```bash
  git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" | xargs clang-format --style=file -i && git diff --exit-code
  ```
- `clang-tidy` also runs in CI (`scripts/parse_clang_tidy.py`, `scripts/clang_tidy_ignore.yaml`). Don't introduce new warnings.

## License header (mandatory)
Every source file opens with the Avidbots ASCII-art BSD-3-clause header. Copy it from
an existing file (e.g. `flatland_server/src/world.cpp`) and edit the `@name`, `@brief`,
`@author`, `@copyright` lines. A file without this header will not be accepted.

## Naming & structure
- Namespaces: `flatland_server::` for the engine, `flatland_plugins::` for plugins.
- Classes `CamelCase`; methods `CamelCase` (ROS/Avidbots convention in this repo, e.g.
  `OnInitialize`, `BeforePhysicsStep`); member variables `trailing_underscore_`.
- Headers live under `<package>/include/<package>/…`; use `#ifndef` include guards
  matching the path (see existing headers).
- Keep engine code in `flatland_server`; keep loadable behaviour in `flatland_plugins`.

## Don't touch
- `flatland_server/thirdparty/` (Box2D, ThreadPool, Tweeny) — vendored, excluded from
  format/tidy, never restyle or patch locally.
