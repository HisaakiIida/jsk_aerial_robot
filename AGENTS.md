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

## Pre-edit preview policy

Before editing any file, you must show the planned edit locations.

For each planned edit, provide:

- File path
- Function, class, node, launch tag, YAML key, or xacro macro to be changed
- Approximate line number if available
- Purpose of the change
- Risk of the change
- A `code -g path:line` command for quickly opening the location in VS Code

Do not modify files until I explicitly approve the plan.

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
