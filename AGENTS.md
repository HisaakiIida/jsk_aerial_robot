# Codex instructions

## Scope

You may inspect and reason about the entire repository.

However, you must treat all files outside my own package as read-only unless I explicitly approve otherwise.

## Editing policy

Do not edit, create, delete, rename, or move any files without my explicit approval.

Before making any file change, you must first provide:

1. A summary of the issue
2. The files you intend to modify
3. The exact type of change planned for each file
4. Any risks or behavior changes

Wait for my explicit approval before applying changes.

## ROS-specific constraints

Do not change topic names, message types, frame names, tf_prefix, namespace conventions, launch arguments, URDF/xacro structure, or controller semantics unless I explicitly ask for that change.

Prefer minimal patches.

When debugging, first explain the likely cause and the verification steps.

## Build and test policy

You may suggest build or test commands, but ask before running commands that may modify files, install packages, access the network, clean build directories, or affect hardware.

Safe read-only commands include:

- git status
- git diff
- grep
- rg
- find
- sed -n
- cat
