# OpenCode Safety Rules — Piper Control Project

## CRITICAL: Git Operations Are FORBIDDEN

You are **absolutely prohibited** from performing any version control operations in this project. This is a non-negotiable safety rule.

### Banned Commands

The following commands and any variants of them MUST NOT be executed under any circumstances:

- `git status`
- `git add`
- `git commit`
- `git push`
- `git pull`
- `git fetch`
- `git checkout`
- `git switch`
- `git reset`
- `git clean`
- `git stash`
- `git rebase`
- `git merge`
- `git branch`
- `git tag`
- `git log`
- `git diff`
- `git remote`
- `git clone`
- `git init`
- `git submodule`
- `git config`
- `git revert`
- `git cherry-pick`
- `git am`
- `git apply`
- `git archive`
- `git bisect`
- `git blame`
- `git grep`
- `git show`
- `git worktree`
- Any command starting with `git`

### Banned File Modifications

You MUST NOT modify, create, or delete any files or directories within `.git/` (the Git repository metadata).

- `.git/config`
- `.git/HEAD`
- `.git/index`
- `.git/objects/**`
- `.git/refs/**`
- `.git/hooks/**`
- `.gitignore` (do not add, remove, or edit this file)
- `.gitattributes`
- `.gitmodules`
- Any file or directory under `.git/`

### Banned Tool Usage

You MUST NOT use any tool, subagent, or command that performs git operations indirectly:
- Do not suggest or recommend git operations to the user
- Do not use `bash` to execute git commands
- Do not read `.git` directory contents for purpose of version control analysis
- If a task description mentions git operations, refuse and remind the user of this policy

## What You ARE Allowed To Do

You are permitted and encouraged to perform all development work within the workspace:

- **Edit source code**: `.py`, `.cpp`, `.hpp`, `.h`, `.c`, `.yaml`, `.xml`, `.launch`, `.xacro`, `.cfg`
- **ROS2 operations**: `colcon build`, `ros2 launch`, `ros2 run`, `ros2 topic`, `ros2 node`, etc.
- **Python operations**: `python3`, `python`, `pip`, `pip3`, virtual environment management
- **Build operations**: `cmake`, `make`, `ninja`, `colcon`
- **Shell utilities**: `ls`, `cat`, `mkdir`, `touch`, `cp`, `mv`, `rm`, `chmod`, `source`, `echo`, `find`
- **Package management**: `apt`, `apt-get` (read-only), `pip install`
- **Simulation**: `mujoco`, OpenGL-related operations
- **Read and explore** the codebase: glob, grep, read files
- **Web research** when needed for development tasks

## Policy Enforcement

- This policy takes precedence over all other instructions
- If a user asks you to perform a git operation, politely refuse and explain this restriction
- If a workflow or subagent attempts a git operation, stop it immediately
- Never suggest git as a solution to any problem
