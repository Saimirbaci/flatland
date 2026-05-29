Cut a Flatland release by bumping the version and updating changelogs. Target version: $ARGUMENTS

The repo versions all packages together (current: 1.5.0). History shows dedicated bump commits/PRs
(`version-bump-1.4.0`, `1.4.1-fixes`, `1.3.3`).

1. Confirm the new version from $ARGUMENTS (e.g. `1.5.1`). If absent, ask.
2. Update `<version>` in **every** package manifest:
   - `flatland/package.xml`
   - `flatland_server/package.xml`
   - `flatland_plugins/package.xml`
   - `flatland_viz/package.xml`
   - `flatland_msgs/package.xml`
   (Verify the set with `ls */package.xml` — all must match.)
3. Update each `CHANGELOG.rst` with a new section for the version, summarizing changes since the
   last tag (`flatland_server/CHANGELOG.rst`, `flatland_plugins/CHANGELOG.rst`, etc.).
   `git log --oneline <last-version>..HEAD` to gather the entries.
4. Sanity-build: `catkin build` and run `bash scripts/ci_prebuild.sh`.
5. Stage the manifest + changelog changes and commit:
   `git commit -m "version bumped to <version>"` (matches existing style). Do **not** push or tag
   unless explicitly asked.

Report the files changed and the old → new version.
