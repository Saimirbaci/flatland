# Security — Flatland

Flatland has no auth, network API, or PII surface. Its real risk is **untrusted input files**.

## World/model YAML is code, not data
- `flatland_server/src/yaml_preprocessor.cpp` runs an **embedded Lua interpreter** over world YAML
  before parsing (`$eval` expressions, env-var/`os` access). A malicious world file can therefore
  execute arbitrary Lua in the simulator process.
- **Only load worlds/models from trusted sources.** Treat a downloaded or user-supplied `.yaml`
  the way you would treat an executable script.
- When adding preprocessor features, do not expose new filesystem/process capabilities to Lua
  beyond what is already there without a deliberate review.

## File paths from config
- World YAML references map images and nested model YAML by path. Validate/normalize these paths;
  do not follow them outside the intended world directory when adding new loaders.
- Layer/map images are read with OpenCV in `layer.cpp` — guard against missing/corrupt files with
  `YAMLException`-style errors rather than crashing.

## Secrets
- There are no secrets, tokens, or credentials in this repo, and there should never be. Do not add
  `.env` files, API keys, or deploy credentials — nothing here needs them.
