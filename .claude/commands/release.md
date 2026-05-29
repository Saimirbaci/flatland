Cut a Flatland release version bump (matches the "version bumped to 1.4.0" / "1.3.3"
commits in the history).

Ask me for the new version (e.g. `1.4.2`). Then:

1. Bump `<version>` in the `package.xml` of **all** packages so they stay in lockstep:
   `flatland/package.xml`, `flatland_msgs/package.xml`, `flatland_server/package.xml`,
   `flatland_plugins/package.xml`, `flatland_viz/package.xml`.
2. Update each package's `CHANGELOG.rst` with a new entry for the version, summarizing
   the changes since the last tag (use `git log <last-tag>..HEAD --oneline` to draft it).
3. Show me the diff for review.
4. Only if I confirm, commit with a message like `version bumped to <version>`
   (matching the repo's style). Do not push or tag unless I explicitly ask.

Keep all five packages on the same version number — never bump one in isolation.
