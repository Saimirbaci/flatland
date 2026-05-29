Refresh this repo's Claude Code infrastructure (`CLAUDE.md` + `.claude/`) to match the current state
of the codebase. Self-contained — no network/friday.com needed.

## 1. Re-read what exists
- Read `CLAUDE.md` and every file under `.claude/` (`rules/`, `agents/`, `skills/`, `commands/`).

## 2. Re-survey the repo
- `ls -F` at root; confirm the package set (`flatland`, `flatland_msgs`, `flatland_server`,
  `flatland_plugins`, `flatland_viz`) — note any added/removed package.
- `git log --oneline -30` — what kind of work is happening now? New plugins? New subsystem?
- Plugins: diff `flatland_plugins/src/*.cpp` against the `<class>` entries in
  `flatland_plugins/flatland_plugins.xml` and against the `flatland-plugin-authoring` skill's examples.
- Versions: `grep -h '<version>' */package.xml` — update the version cited in `CLAUDE.md`.
- CI: re-read `.github/workflows/industrial_ci_action.yml` and `scripts/ci_prebuild.sh` for changed commands/gates.
- Build/test/run: re-read `README.md` and `flatland_server/launch/*.launch` for changed commands/args.

## 3. Diff against the docs — look for drift
- New framework / dependency / language → add or update a rule under `.claude/rules/`.
- New major subsystem → propose a new skill and/or agent (ask the user before adding a persona).
- Renamed or deleted files → fix every citation (CLAUDE.md Key Files, agents, skills, commands).
- Removed dependency / dead command → delete the stale rule/skill/command.

## 4. Update in place
- Edit the affected files directly. Keep the size budgets: CLAUDE.md < 300 lines, rules/agents
  30–80 lines, commands 20–60 lines. No fabricated paths — verify each citation still exists.

## 5. Verify
- Re-confirm every path cited in `CLAUDE.md` exists (`ls` each).
- Confirm `.claude/commands/update-claude-infrastructure.md` (this file) still exists.

## 6. Commit (do NOT push)
```bash
git add CLAUDE.md .claude
git commit -m "chore(claude): update infrastructure for current state of the repo"
```
Report what changed and why; if you propose a new persona/skill, ask before committing it.
