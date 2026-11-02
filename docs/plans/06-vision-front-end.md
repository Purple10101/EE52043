# 06 Vision front end

**Status:** Draft, first version built · **Stage:** 3 (brought forward, real arm only) · **Depends on:** 01, 03

## Goal

The front of the work cycle (steps 1–4 and the reach check of 8): take a snapshot of the pick plate,
find every resistor body, and turn each into a robot-frame target the arm can be sent to, refusing
any it can't reach before anything moves.

## Scope

**In:** live and replay camera, plate calibration, detection, the scene-to-robot mapping, the
body-height correction, Detection geometry, the IK reach check, and a dry run that prints the
`ScaraController` call for each body.

**Out (for now):** sending moves, Z (slide) heights, the pick sequence, classification, ranking
beyond "reachable first", the Unity simulator.

## Design

The pipeline from a raw frame to plate outlines is the robot_offline package (sibling repo,
`pip install -e`). Its rectification fits the camera to the plate each snapshot and maps image
pixels onto the plate's top **in the rig scene frame** (mm, origin at the robot base axis, z up),
the same frame as the arm and rig CAD. So the robot frame is one fixed transform away.

| Step | Where | What |
|---|---|---|
| Snapshot | `vision/camera.py` | `LiveCamera` (robot_offline's camera settings, stale frames flushed) or `ReplayCamera` (saved raw frames) |
| Calibrate | `vision/detector.py` → `rectify.calibrate` / `recalibrate` | about 1.5 s from scratch, 0.5 s near the last fit |
| Detect | `vision/detector.py` → `rectify.model_input`, `tiling.predict_frame` | outlines in rectified px |
| Map | `vision/detector.py`, `vision/rig.py` | rectified px → scene mm on the plate → body-height correction → robot m |
| Geometry | `world/types.py` `Detection.from_outlines` | centre, yaw, length, width (min-area rectangle), plausible (package sizes) |
| Reach | `kinematics/arm.py` | firmware IK mirrored, joint limits, wrist wrap, yaw + 180 for a symmetric gripper |
| Dry run | `task/plan_picks.py` | report, overlay and JSON per snapshot |
| Live window | `task/live_plan.py` | the same a few times a second on the camera (or saved frames), drawn on the top-down or camera view, with a panel of targets; reuses robot_offline's plate fitter, exposure control and window helpers |

**Scene to robot:** robot = R(180°) · scene / 1000 (`workspace.toml [scene_to_robot]`). From the CAD:
the arm lies along scene −y at zero, and the firmware IK of robot (0, 0.130) m is the CAD's pose over
the plate centre (base −46.12°, elbow 110.42°).

**Body height:** a body's axis is half its width above the plate, so the camera, 200 mm up at 36°,
sees it up to about 1.5 mm further away than it is. Each outline is moved toward the camera's foot by
height / camera height of its distance, using the lens position from the plate fit.

**Tool angle:** the jaws close across the body, so the tool angle is the body's yaw + 90°, less the
direction the jaws close along at tool angle 0 (`grippers/*.toml`).

## Keep-out zones (the camera tower)

**Moves are straight lines in joint space.** `move_js_scara` (`servo_controller.c`) runs every joint
on the same move time and double-S profile, with speed, acceleration and jerk in proportion to each
joint's distance (`calc_double_s_parameters`). So all joints start and stop together, and the arm
travels a straight line in (base, elbow, wrist, slide), not in x/y. A zone is therefore defined over
joint angles, and a move is safe only if the whole line between its end poses is clear.

| Part | Where | What |
|---|---|---|
| Map | `robot_control/config/keep_out/camera_tower.json` | blocked (base, elbow) cells on a 1° grid, elbow up to 141° |
| Map maker | `scripts/keep_out_map.py` (run in a venv with `cadquery-ocp`, `scipy`) | poses the arm STEP with the gripper STEP fitted at every grid cell, slide bottom / middle / top, wrist every 30°; blocked if any surface point comes within the clearance of the tower's surfaces (both column versions) |
| Viewer | `scripts/show_keep_out.py` | draws a map in joint space with the joint angles the plate and trays need |
| Which maps | `workspace.toml` `keep_out = ["camera_tower"]` | loaded by `config.KeepOutMap.from_config()` |
| Checks | `kinematics/keep_out.py` | `KeepOut.blocks(j0, j1)` (blocked if any grid point around it is), `first_hit(zones, a, b)` along a straight move, `route(zones, a, b, elbow_max)`: straight if clear, else fold the elbow, swing the base, unfold |
| Enforced | `Kinematics.ik` | a target inside a zone is refused ("in the camera tower keep-out ..."), so `plan_picks` and `live_plan` skip it |

The map covers every wrist angle and slide height, so it is conservative for any of them. Motion
(not built) must check every move with `first_hit` and use `route` when a move would cross a zone.

**The tower map (2026-10-31):** 10 mm clearance, gripper v3a.1 fitted: 4,290 of 25,702 cells
blocked, a slanted band from about base −8° (elbow up to about 105°) down to base 66° (elbow near 0).
Checks:

- Arm alone at 3 mm clearance, it matches the camera tower README's Fusion map
  (`camera_tower/analysis/collision_map_data.json`) cell for cell, at 1° instead of 2.5°. Its "base
  +10 to +60°, elbow above about 85°" is a simplification of the same band.
- Spot checks with exact solid intersections (OpenCascade): base 30°, elbow 50° hits the head, column
  and camera; base 0°/elbow 90°, base 60°/elbow 10° and the plate-centre pose don't touch.
- The gripper widens the band (the SG90 swings 51 mm out as the wrist turns): from 2,473 to 3,629
  cells at 3 mm.
- Bounding boxes were tried first and rejected: the camera head is L-shaped, so its box blocked
  poses that clear it in the CAD.

| Target | Reachable (joints) | Reachable and outside the tower zone |
|---|---|---|
| Pick plate | 69% | 69% |
| Tray Body1:3 | 72% | 72% |
| Tray Body1:4 | 90% | 41% |

Example: plate far corner (base −46.5°, elbow 73.7°) to the far side of tray Body1:3 (82.6°, 66.7°)
would cross the band at base −7°; `route` folds the elbow to 108°, swings, then unfolds.

Margin: 10 mm covers tower placement and small servo errors. It does not cover the base servo's
offset (up to 13° short, about 30 mm at full reach): fix that first. Re-run `scripts/keep_out_map.py`
after changing the tower, the gripper or the margin.

## Measurements from the CAD (2026-10-31)

From `Robot Resources/arm_and_rig_scene` (arm STEP, rig OBJ) and the v3a.1 gripper STEP, read with
OpenCascade.

| Quantity | Value |
|---|---|
| Plate top | x −50…50, y −180…−80, z −22 mm (scene). Near edge 80 mm from the base axis; the base's 150 mm square ends at y = −75 |
| Link 1, link 2 | 124.97 mm (base axis (0, 0) to elbow axis (90.1, −86.6)), 100.0 mm (to the Z shaft at (0, −130)) |
| Elbow, firmware | 0–120°: the servo's whole 2500–500 µs pulse range (`servo_controller_scara_init`) |
| Elbow, links | about 141° before link 1 and link 2 touch, at any base angle; no clash with the base column before that |
| Gripper offset | 0: socket and closed tips both on the Z-shaft axis ("mounts centred on the Z-shaft") |
| Fingertips | 64 mm below the Z shaft's bottom face |

Reach over the plate (0.5 mm grid, firmware IK, base ±90°):

| Elbow limit | Plate out of reach |
|---|---|
| 0–90° (`scara_control/scara_workspace.py`) | 77% |
| 0–120° (firmware) | 31% (the base side; the reach limit is 114.6 mm from the base axis) |
| 0–130° | 13% |
| 0–140° | about 0% |

With the elbow allowed to 141° and the gripper posed in the scene (10 mm grid, jaws open and shut,
slide at pick height, +20 mm and the top of the stroke), every resistor angle can be gripped at 88%
of the plate, and some angles everywhere. What blocks the rest:

- **The SG90 side and the elbow:** the gripper's servo end (51 mm from the axis) hits the disc under
  the elbow when it points toward the elbow. Of yaw and yaw + 180°, only one is safe; which one
  depends on the arm pose. Not modelled in the kinematics yet.
- **The robot base:** next to the base (plate y −80…−90), the gripper body overlaps the base square
  for some angles.
- **The cradle:** along the plate's x = −50 edge the open tips reach the cradle (Body1:2), taken here
  as its bounding box, so this is pessimistic.

## Open points

- **Check the scene-to-robot transform on the arm:** point the closed tips at the crosshair and the
  reachable plate corners, read back with `read_coordinates()`, and fit a correction into
  `[scene_to_robot]`. This also confirms the frame isn't mirrored.
- **The shoulder offset** (docs/hardware-api.md, 2026-10-05: base moves landing up to 13° short) is
  an assembly error the CAD can't show. It must be fixed before any target is trusted.
- **More elbow:** 120° loses 31% of the plate. The links allow 141°. Getting more means moving the
  elbow servo's travel (re-indexing the horn, 25T so 14.4° a tooth) and the firmware's limits and
  offset together; losing the 0–20° end costs nothing on the rig (the furthest bin point needs about 60°).
  `scara_control/scara_workspace.py` still caps the elbow at 90°, which would refuse the plate centre:
  raise it to the firmware's 120° before sending anything through it.
- **Jaw direction at tool angle 0** (`jaw_axis_at_zero_deg`) and **which slide end is up**
  (`hover_z_m`) are placeholders until measured on the arm.
- **Servo-side rule:** add the elbow keep-out to the wrist choice once the jaw direction is known.
