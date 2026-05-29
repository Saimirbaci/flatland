# Git Workflow — Flatland

Observed from `git log` and the upstream `avidbots/flatland` history.

## Branch & PR flow
- Work on a feature branch; merge to `master` via pull request. The log is full of
  `Merge pull request #NNN from avidbots/<feature>` commits — single-purpose branches
  named after the feature (e.g. `imu`, `diff-drive-plugin-tf-broadcast-optional`).
- Keep one logical change per PR. Plugin additions, perf work, and version bumps are
  separate PRs in the history.

## Commit messages
- Short imperative or descriptive subject lines. No strict conventional-commit prefix
  is enforced, but be specific (e.g. "Make diff drive plugin tf broadcasting a
  configurable option", "Added simplify_map feature, as well as benchmark").

## Versioning & changelogs
- Releases bump the version in **every package's `package.xml`** together (e.g. the
  "version bumped to 1.4.0" / "1.3.3" commits) and update the per-package
  `CHANGELOG.rst`. Keep all five packages on the same version.
- A new ROS message/dependency must be declared in the relevant `package.xml`
  (`<depend>`/`<build_depend>`/`<exec_depend>`) and `CMakeLists.txt`.

## Before you push
- Build clean with `catkin build` and run the affected package's tests
  (`catkin run_tests <pkg> --no-deps` + `catkin_test_results`).
- Pass the format/tidy gate (see `coding-style.md` / `scripts/ci_prebuild.sh`).
