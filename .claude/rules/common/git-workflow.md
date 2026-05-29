# Git Workflow — Flatland

## Branch & PR flow
- Work on a feature branch, open a **PR against `master`**, merge after review. The git history is
  PR-merge based (e.g. `Merge pull request #105 from avidbots/1.4.1-fixes`,
  `#103 from avidbots/imu`). Feature branches are named after the feature (`imu`,
  `diff-drive-plugin-tf-broadcast-optional`).
- Keep one logical change per PR (a plugin, a perf optimization, a fix).

## Commit messages
- Short imperative subject describing the change, matching existing style:
  - `Added simplify_map feature, as well as benchmark.`
  - `Make diff drive plugin tf broadcasting a configurable option`
  - `Fixed cmake warning on ros build farm, bumped version to 1.4.1`
- No enforced conventional-commit prefix in history — be descriptive over formulaic.

## Versioning & changelogs (do this on a release PR)
Releases bump the version and update changelogs across packages:
- Update `<version>` in **each** `package.xml` (`flatland_server`, `flatland_plugins`,
  `flatland_viz`, `flatland_msgs`, `flatland`). Current: `1.5.0`.
- Update the per-package `CHANGELOG.rst` (e.g. `flatland_plugins/CHANGELOG.rst`).
- History shows dedicated bump commits/PRs (`version-bump-1.4.0`, `1.3.3`, `per-package-licenses`).

## Before pushing
- `bash scripts/ci_prebuild.sh` must pass (clang-format + clang-tidy).
- Run affected tests: `catkin run_tests <pkg> --no-deps && catkin_test_results`.
- Never commit anything under `thirdparty/` as if it were your change.
