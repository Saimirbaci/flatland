Refresh this repo's Claude Code infrastructure (`CLAUDE.md` + `.claude/`) so it matches
the current state of the codebase. Self-contained — no network/friday.com needed.

## 1. Re-read what exists
Read `CLAUDE.md` and every file under `.claude/` (rules, agents, skills, commands) so
you know the current claims.

## 2. Re-survey the repo
- `ls -F` at root; confirm the package set (`flatland`, `flatland_msgs`,
  `flatland_server`, `flatland_plugins`, `flatland_viz`) — has one been added/removed/renamed?
- `git log --oneline -30` — what kind of work is happening now? New plugin? New subsystem?
- Check manifests/build for stack drift: `package.xml` deps, `CMakeLists.txt`, the CI
  workflow (`.github/workflows/industrial_ci_action.yml`), `scripts/ci_prebuild.sh`,
  ROS distro. Re-list `flatland_plugins/flatland_plugins.xml` vs `flatland_plugins/src/*.cpp`
  to catch new/removed plugins.
- Verify every file path cited in `CLAUDE.md` and the skills still exists.

## 3. Compare & decide
- **Drift** (renamed paths, new deps, changed commands, new plugins, ROS distro change):
  update the affected files in place.
- **New major subsystem** (e.g. a new package, a ROS 2 port, a new physics path): ASK me
  whether to add a new agent / skill / command for it before creating one.
- **Dead content** (a skill/command for something removed): propose deleting it.

## 4. Keep the conventions
- Cite only real paths (verify with `ls`/Read first). No generic boilerplate.
- Rules/agents 30-80 lines, commands 20-60, `CLAUDE.md` under 300.
- Keep the per-file Avidbots BSD header, clang-format, and pluginlib two-sided
  registration rules accurate.

## 5. Commit (do not push)
Stage the changes and commit with:
`chore(claude): update infrastructure for current state of the repo`
Then summarize what changed and why. Do not push or open a PR unless I ask.
