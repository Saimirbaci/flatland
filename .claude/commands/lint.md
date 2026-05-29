Run Flatland's formatting + static-analysis gate locally, exactly as CI does
(`scripts/ci_prebuild.sh` runs this before the build in
`.github/workflows/industrial_ci_action.yml`).

1. **clang-format** — check, then fix in place:
   ```bash
   # show files needing changes
   git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" | xargs clang-format --style=file -output-replacements-xml | grep -c "<replacement "
   # apply the fixes
   git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" | xargs clang-format --style=file -i
   git diff --exit-code
   ```
   (CI uses the same `git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/"` set.)

2. **clang-tidy** — run tidy over the changed C/C++ files and parse with the repo's
   helper: `scripts/parse_clang_tidy.py`, honoring `scripts/clang_tidy_ignore.yaml`.

Never reformat anything under `flatland_server/thirdparty/`. Report what changed; if
clang-tidy finds new warnings, list them with `file:line`. Do not commit unless I ask.
