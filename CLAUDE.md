# CLAUDE.md

## Commit message format (mandatory)

Every commit subject line must be:

```
YYYYMMDD - description
```

- `YYYYMMDD` is the date of the commit, e.g. `20260928 - initial commit`.
- A space, hyphen, space separates the date from a short description.
- This is a standing rule: never commit with any other subject format. If a commit is made in
  the wrong format, reword it (amend/rebase and force-push if already pushed) straight away.

## Branch name format (mandatory)

Every branch name must be:

```
YYYYMMDD-description-separated-by-hyphens
```

- `YYYYMMDD` is the date the branch is created, e.g. `20260928-joint-sim-test`.
- The description is lowercase words joined by single hyphens, no spaces or underscores.
- This is a standing rule: never create a branch with any other name format. If one is created in
  the wrong format, rename it (and its remote counterpart if already pushed) straight away.

## Project

EE52043 Applied Robotics coursework - controlling a 4-axis SCARA arm (real robot over serial, or
the Unity simulator over TCP). See `README.md` for layout and setup.
