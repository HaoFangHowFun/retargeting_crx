# General Agent Rules

- Default communication language is Chinese; keep code, commands, paths, and identifiers unchanged.
- Preserve current behavior first. Do not perform unrelated refactors.
- Read relevant code and call chains before editing.
- Prefer the flattest reasonable code structure; avoid unnecessary nesting, wrappers, and indirection.
- Keep code concise and direct without sacrificing clarity, correctness, or responsibility boundaries.
- Use the repository-local `.venv/bin/python` for all Python tests, checks, and scripts. Do not use conda. Bootstrap `.venv` with system Python when needed.
- For headless tests and environment installation, prefix Python commands with `env -u PYTHONPATH` to avoid inherited ROS packages. Keep the ROS environment only for explicit ROS work.
- Do not overwrite user changes. In a dirty worktree, touch only files related to the task.
- Explain the plan before major directory moves, data deletion, dependency upgrades, hardware behavior changes, or ROS behavior changes.
- Use `rg` / `rg --files` first for repository search.
- Do not run destructive commands such as `git reset --hard` or broad `rm` unless explicitly requested.
- Keep reports concise and actionable.
