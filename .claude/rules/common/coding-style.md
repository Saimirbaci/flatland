# Coding Style — Flatland

Applies to all C++ in `flatland_server`, `flatland_plugins`, `flatland_viz`, `flatland_msgs`.

## Formatting
- **clang-format `--style=file`** is authoritative — the repo ships a `.clang-format`. Never override it inline.
- Before committing, run the CI gate locally: `bash scripts/ci_prebuild.sh`. It checks every
  tracked `.c/.h/.cpp` file (excluding `thirdparty/`) and **fails CI** on any diff.
- One-shot reformat the way CI does:
  `git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" | xargs clang-format --style=file -i`
- clang-tidy also runs in CI; `scripts/clang_tidy_ignore.yaml` lists suppressions.

## License header (mandatory on every source file)
Every `.cpp` / `.h` begins with the Avidbots ASCII-art BSD header — copy the block at the top of
`flatland_server/src/body.cpp`. Fill in the per-file fields:
```
 * @copyright Copyright <year> Avidbots Corp.
 * @name   <filename>
 * @brief  <one line>
 * @author <name>
```
A new file without this header will not pass review.

## Naming & structure (ROS / roscpp idioms)
- Classes `PascalCase`; methods `PascalCase()` (matches existing `OnInitialize`, `GetModel`);
  member variables trailing-underscore `model_`, `body_`.
- Headers live in `<pkg>/include/<pkg>/` and use `#ifndef`/`#define` guards (see existing headers).
- Namespaces: engine code in `flatland_server`, plugins in `flatland_plugins`.
- Keep `.cpp`/`.h` paired under `src/` and `include/<pkg>/`.

## Excluded from all of the above
- `flatland_server/thirdparty/` (vendored Box2D, ThreadPool, Tweeny) — do not reformat, re-lint, or edit.
