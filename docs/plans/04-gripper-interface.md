# 04 Gripper interface

**Status:** Draft · **Stage:** 1 · **Depends on:** 01

## Goal

Fix the shape every gripper design plugs into (driver interface, model profile, and step vocabulary)
before any physical gripper exists, so motion can be built and tested against a dummy one.

## Scope

**In:**
- The driver interface and the sim (no-op) driver.
- The gripper model and its TOML profile format, plus a dummy profile.
- The step vocabulary used by pick and place sequences.

**Out:**
- **Grasp strategy:** stage 2. It needs Detections and neighbours.
- **Real drivers:** stage 4. They depend on the actuation decision.
- **Opening width:** parked (pre-tightening idea).

## Design

### Driver interface

| Operation | Meaning |
|---|---|
| open | Open fully |
| grip | Close on the part |
| release | Let go (for most designs the same as open) |
| capabilities | What this design can do: `senses_grip`, `variable_opening` (parked, false for now) |
| is_holding | Only if `senses_grip`: whether a part is held |

Every call blocks until the actuation has finished (the model's actuation time), matching how robot
moves block until they have settled. Failures raise one error type.

**SimGripper:** logs each call and waits the actuation time. The wait can be switched off for unit
tests. Reports no capabilities.

### Gripper model (one TOML file per design)

| Field | Units | Used by |
|---|---|---|
| name | – | logs |
| driver | – | which driver to load (`sim`, later real ones) |
| tool_offset dx, dy | m | kinematics: grip point relative to the wrist axis |
| tool_length dz | m | kinematics: grip point below the wrist |
| symmetric_180 | bool | kinematics: whether yaw + 180° is allowed |
| wrist_limits | deg | kinematics: optional, if cables narrow the wrist range |
| approach_height | m above table | motion: where to pause above a part |
| grip_offset | m | motion: grip height relative to the body's centre height |
| footprint_open | m polygon | grasp strategy (stage 2): clearance check |
| max_part_width | m | grasp strategy: largest body it can take |
| actuation_time, settle_time | s | driver and motion waits |
| pick_sequence, place_sequence | list of steps | motion |

`grippers/dummy.toml` uses zero tool offset, symmetric, a small square footprint and the jaw sequences
below, with values marked `# placeholder`.

### Step vocabulary

Sequences are lists of these steps. Heights are named, not numbers, so one sequence works for any
part size or bin.

| Step | Effect |
|---|---|
| move_above(height) | Rise to travel height if lower, move x/y/yaw at travel height, descend to `height` |
| descend(height) | Vertical move down only |
| lift(height) | Vertical move up only |
| open, grip, release | Driver calls |
| wait(seconds or `settle`) | Pause |

Named heights:
- `travel`: from the workspace config.
- `approach`: from the gripper model.
- `grip`: the body's centre height (table + measured width/2) plus `grip_offset`.
- `drop`: from the bin.

Example jaw sequences for the dummy profile:
- **pick:** move_above(approach), open, descend(grip), grip, wait(settle), lift(travel)
- **place:** move_above(drop), release, wait(settle), lift(travel)

## Done when

- [ ] The dummy profile loads, and a profile with an unknown step or height name fails with a clear
      message.
- [ ] SimGripper runs open/grip/release and reports no capabilities.
- [ ] A second, made-up profile (e.g. vacuum order: descend before grip, no open) loads with no code
      changes. This proves the format is not tied to jaws.

## Open points

- Actuation path: firmware command or separate microcontroller (staff question). Only affects
  stage 4 drivers.
- Whether the gripper ever needs an explicit "home" or calibration step at startup.
