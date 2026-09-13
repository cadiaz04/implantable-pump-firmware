# Workspace Audit — Instructions for Claude

I'm a Bioengineering senior building an implantable pump for my senior
design project. I'm new to VS Code and PlatformIO, and I've done some
scattered experimenting that may have left stray files, duplicate
projects, or inconsistent settings behind. I need help getting this
workspace into a clean, correct state.

**Please do a full audit before changing anything, explain what you find
in plain English (I don't know VS Code terminology), and ask me before
doing anything destructive or irreversible — deleting a folder, force
git operations, overwriting files, etc.**

## 1. Inventory the workspace

- List every top-level folder/project you can see in this workspace.
- For each one, tell me what it looks like it is: a real project, a
  leftover test/experiment, an empty scaffold, etc. Default/placeholder
  names (like "balls", "untitled", "project1") are a sign it was an
  unintentional test project — flag these.
- Tell me which folder you think is my *real* project, and why.

## 2. Check the real project against how it should be set up

My actual hardware:
- Board: a classic ESP32 DevKit (not an S3 or C3 variant)
- Actuator: a stepper motor driving a syringe pump

In the real project's `platformio.ini`, check:
- That a board definition matching a classic ESP32 DevKit is present
  and set as the default environment
- That `framework = arduino` is set
- That there isn't a conflicting or duplicate environment left over
  from experimentation

In the real project's folder structure, check that these exist and
look intact (not empty, not obviously broken):
- `include/config.h` — pin assignments and constants
- `src/drivers/` — stepper motor driver code
- `src/control/` — flow-rate control logic
- `src/main.cpp` — main firmware entry point
- `test/` — unit tests for the control logic
- `.gitignore`
- `README.md` and `CLAUDE.md`

## 3. Check git status

- Is this folder a git repository already (i.e., is there a `.git`
  folder)? If not, tell me — don't initialize one without asking.
- If it is a repo, is there at least one commit? Are there uncommitted
  changes sitting around from experimentation that I should know
  about before I do anything else?
- Is a GitHub remote configured, or is this still local-only?

## 4. Check the PlatformIO / VS Code setup itself

- Does the PlatformIO extension appear installed and working (i.e., can
  you see build/upload options for this project)?
- Is there anything in `.vscode/` that looks like leftover
  configuration from a different (test) project rather than this one?

## 5. Report back

Give me a short summary organized as:
- **What's here now** (plain English, 3-5 bullet points)
- **What's wrong or inconsistent, if anything**
- **What you'd recommend changing**, with each recommendation as a yes/no
  question I can answer, before you touch anything

Only after I respond to your recommendations should you make changes —
and even then, make one change at a time and tell me what you did.
