# Handoff: calibrating the arm with the camera

From the session of 2026-11-03, for whoever builds it. The plan, the facts it rests on (each with the
file or log it came from), and the safety rules learned on the arm so far. Nothing here is built yet.

## The goal in one paragraph

At every boot, before picking, the arm should prove it knows where the plate is, using the tower
camera as the judge. It touches the plate's centre crosshair with the gripper tip while the camera
watches, correcting until the camera sees the tip on the crosshair. It lines the jaws up with the
crosshair's lines, then visits points round the plate's outline the same way. Each visit pairs where
the camera saw the tip (plate mm) with where the arm thinks it is (potentiometer angles). Fitting the
pairs maps camera to arm directly, end to end. That replaces today's hand-taught 4.4 mm and catches
drift (a slipping coupling, a knocked plate) before it costs a pick.

## Why this and not hand teaching

- Hand teaching put the tip "over" a point by eye. The nominal scene-to-robot transform fits those
  points to **4.4 mm RMS** (2026-11-02, `logs/reach_points_20261102_180907_recal.json`). A 1/4 W
  resistor body is **2.4 mm** wide (`robot_control/config/workspace.toml [packages]`), so the error
  budget is tight.
- The camera already finds the plate in every frame (robot_offline's `rectify`: edges and crosshair).
  The camera's own view of the tip is the measurement that matters for picking, so measure that.
- One point fixes an offset. Points spread round the plate also fix rotation and scale. They can also
  absorb the base potentiometer's slight non-linearity: it reads **−1.6°** at its 0 mark while
  calibrated at ±90 (pot_check 2026-11-02 17:56).

## The pipeline being calibrated

```
camera pixel ─rectify (per frame)→ scene mm on the plate ─[scene_to_robot]→ robot x,y (m)
            ─IK (firmware / kinematics.arm)→ joint angles ─firmware + pots→ the real arm
```

`[scene_to_robot]` (`workspace.toml`, applied by `robot_control/vision/rig.py`) is the link this work
measures, and optionally a correction to the joint angles too (see "The fit").

## Facts to build on

### The rig (scene frame: mm, origin at the base axis, z up)

| What | Value | Source |
|---|---|---|
| Plate top | z −22; corners (±50, −80) and (±50, −180); corners rounded r 5 | robot_offline `configs/dataset.toml [rectify]` |
| Crosshair | groove 1 mm wide, 3 mm deep, at **(−0.48, −130.37)**, measured on the printed plate (spread 0.02 mm) | same |
| Camera lens | design (−145, −145, 178), 36.1° from vertical, aimed at the plate centre; the plate fit refines it per frame (`cal.fit["camera_mm"]`) | `Robot Resources/camera_tower/README.md`; `robot_control/vision/detector.py camera_mm()` |
| Scene to robot | rotation 180°, translation 0 (confirmed to 4.4 mm RMS) | `workspace.toml`; this repo's PR #3 |
| Bins (real rig) | `bins_real.toml`: trays turned 2026-11-02, slot directions per slot | |

### The camera

- Arducam IMX708 USB2, native 4608 × 2592 MJPG. Opened by robot_offline's `pre/camera.open_camera`
  (`robot_control/vision/camera.py LiveCamera`): autofocus off, exposure and white balance locked.
- `rectify.calibrate(frame)` / `rectify.recalibrate(frame, previous)` find the plate (raise
  `PlateNotFound`). `rectify.to_scene_mm(points_px, cal)` takes full-frame pixels to scene mm **on the
  plate's top surface**.
- **Parallax:** the camera looks 36° off vertical, so a point h mm above the plate appears about
  0.73·h mm further from the camera than it is. `rig.correct_height(points_mm, camera_mm, surface_z,
  height)` corrects it. At plate height the error is zero, which is why the plan touches the plate.

### The arm and firmware

- Firmware: the course's Moodle build (`firmware/`), protocol in `hardware_api/scara_motion_controller_api.py`.
  It enforces **no joint limits**, and its settling step pushes the pulse with **no limit**. Keep to
  `robot_real.toml`: base −90..67.5, elbow 0..139.
- **`SCARA_AUTO_CALIBRATE` must run after every RESET.** It maps servo pulses to potentiometer angles,
  in RAM. It sweeps the base and elbow end to end and ends at base 0°, elbow 90°.
- Base moves past about 30° land short unless settling is the course's **1 s**. The base pulse map
  falls about 8° short at 60° (`scripts/base_reach.py`). The firmware stops correcting within
  **0.5°** of target, about 1 mm at the plate: that's the arm's resolution in this loop.
- `SCARA_MOVE_JOINT(axis, …)` pulses only that joint. `SCARA_MOVE_JOINTS` and `SCARA_MOVE_COORD` pulse
  all four, and after a RESET a limp wrist or slide **snaps** to its first pulse. Prefer single-joint
  moves until the wrist and slide are calibrated.
- Base and elbow have potentiometers (`SCARA_READ_ANGLE` 0/1). The wrist and slide have none:
  `READ_COORD`'s z and wrist are just the last commanded values.
- Patterns that worked: `scripts/reach_test.py` (teach by hand, then replay one joint at a time,
  read back, record as it goes), `scripts/base_reach.py` (command past a short map, never past a safe
  bound, stop when a bigger command stops moving the joint).

### The gripper (`robot_control/config/grippers/parallelogram_v4.toml`)

- Slim v4: jaws open 0-12 mm, so a 1/4 W body leaves 4.8 mm a side for all positioning error.
- Tip 64 mm below the Z shaft's bottom face, on the shaft axis (tool offset 0, 0).
- `jaw_axis_at_zero_deg = 0.0` is a **placeholder**: which way the jaws close at wrist 0° is unmeasured.
- The SG90 sits behind the linkage, 36.3 mm from the axis, square to the jaws.

## Prerequisites (before any of the steps below)

1. **The slide (z), calibrated.** Find which end is up (unconfirmed: `workspace.toml hover_z_m`). Find
   the slide joint angle that puts the fingertips at the plate top (z −22) and 1–2 mm above it, and a
   safe hover. Slide mapping: `servo_init(axis_3, 500, 2300, 1500, 90, −60)`, z_min 0.095 → 90,
   z_max 0.15 → −60 (`firmware/Core/Src/servo_controller.c`, `scara.c calc_inverse_kinematics`).
2. **The wrist, roughly.** Its real travel (the firmware claims ±180°, unlikely for a hobby servo:
   measure it), and a safe range that keeps the SG90 side away from the elbow.
3. **A marker on the fingertip** that the camera finds reliably in every pose, such as a small bright
   dot or an ArUco tag on the camera-facing side. Check it isn't hidden by the gripper body from a
   view 36° off vertical, at both jaw angles.
4. **Camera tower on, and AUTO_CALIBRATE checked against it.** The tower has been off for every arm
   test so far. Its CAD keep-out zone (`config/keep_out/camera_tower.json`, rebuilt for gripper v4)
   covers base 1–24° at elbow 90. AUTO_CALIBRATE ends clear of it (base 0, elbow 90), but its elbow
   sweep at base 0 **crosses it from elbow 76 to 88**. The CAD zone is conservative (10 mm margin,
   every slide height and wrist angle). But the first AUTO_CALIBRATE with the tower mounted must be
   watched with a hand on the supply, with the slide up. Bin B4 is inside the zone too.

## The plan

### 1. Fix the plate in the image (arm parked)

Park the arm off the plate, take a frame, and run `rectify.calibrate`. Keep that `Calibration` for the
whole session: the camera and plate don't move, and the arm hides plate edges when it's over them.
Re-fit only to check nothing moved, at the end and with the arm parked again.

### 2. Find the tip in a frame

`find_tip(frame, cal) → scene mm`: the marker's pixel position, through `rectify.to_scene_mm`, then
`correct_height` for the marker's height above the plate (0 when touching). Test it with the arm held
by hand (servos off) over known points: the crosshair, the corners. Compare with the potentiometers
through the nominal transform. Agreement within a few mm means the detector is right.

### 3. Touch the crosshair, under the camera's guidance

1. Go above the crosshair at a safe hover, by the current calibration (nominal transform and IK).
   Base, then elbow, as single-joint moves.
2. **Correct in x, y:** find the tip, compute the error to the crosshair in scene mm, and convert to
   robot. Move by a fraction of it (gain about 0.7), capped at about 5 mm a step. Repeat until within
   about 1 mm or 8 tries, and stop if it isn't converging. Hovering adds parallax, so use
   `correct_height` with the hover height.
3. **Come down:** the slide to 1–2 mm above the plate (prerequisite 1), then in small steps to
   touching. **There is no force sensing:** driving the printed tips into the plate stalls the servo
   or breaks the tips. Stop at the measured plate height. Never command below it.
4. **Correct again at touch height,** where there's no parallax. Record the potentiometer angles and
   the tip's scene mm. That's one pair.
5. Lift to hover.

### 4. The wrist: line the jaws up with the crosshair's lines

The crosshair's two grooves are known directions in the scene. Over the crosshair at low height, turn
the wrist until the jaws (or a line marked across them) lie along one groove, judged by the camera:
the angle between the jaw line and the groove in the rectified image. Then do the other groove. That
gives `jaw_axis_at_zero_deg` and a check on the wrist's scale (the two grooves are 90° apart). Keep to
the wrist's safe range (prerequisite 2).

### 5. Round the plate's outline

Targets in plate mm, inset from the edge (the corners are rounded r 5, and the jaws must stay over
the plate): about 8 mm in from each corner, the mid-points of each edge, and the crosshair again. The
near edge (y −80) is close to the elbow limit: its middle needed the elbow at about 137.7° on
2026-11-02, so inset more there or skip it. Each target is as in step 3, giving 9–10 pairs. This is
the "9-point calibration" vision-guided robots use.

### 6. The fit

Pairs: (scene mm seen by the camera, base and elbow potentiometer angles). Two models, in order:

1. **Rigid 2-D transform** (rotation and translation, 3 numbers). This is what `workspace.toml
   [scene_to_robot]` holds, so it drops straight in: robot = R·scene/1000 + t, with robot from
   forward kinematics of the angles (link lengths 0.125, 0.100 m, `robot_real.toml`).
2. **Joint corrections** (base offset and scale, elbow offset, maybe link lengths) fitted with the
   rigid transform. Use this if the rigid fit's residuals show a pattern with the base angle, as the
   hand-taught points did (the base potentiometer). It needs a place in the code to apply it: the IK
   output before sending, and the potentiometer reading after.

Report the residual for every point. Accept if RMS ≤ 1.5 mm and worst ≤ 2.5 mm (suggested). Then
**check open-loop:** go to the crosshair and one corner from the new calibration alone, camera only
watching. Store the result with its date and residuals, for example
`robot_control/config/camera_robot.json` or an updated `workspace.toml`.

### 7. At every boot

1. PROGRAM, RESET, `SCARA_INITIALISE`, `SCARA_AUTO_CALIBRATE` (watched the first times, prerequisite 4).
2. The wrist and slide to a known safe pose, slowly.
3. Arm parked, plate fit (step 1).
4. **Quick check:** the crosshair and the four inset corners (steps 3 and 5, about 5 points). If every
   point is within the acceptance threshold of the stored calibration, carry on. If not, run the full
   outline (step 5) and refit (step 6), and say which point moved.
5. Pick, with a snapshot after each grasp. If the resistor is still on the plate, retry, maybe at
   the other jaw angle.

## Safety rules learned on the arm (2026-11-02)

- Servo supply: a USB-C PD trigger with **no current limit**. DIP switch on 5 V; unplugging is the
  e-stop; a hand on it for every powered test (memory: servo power supply).
- After a RESET the wrist and slide are limp and snap to their first pulse.
- Nothing stops a command past a joint's travel or into the elbow's stop (141.1°): settling keeps
  pushing. Bound every command; back off when the joint stops following.
- The DEMO switch on DEMO at a reset runs the course demo, which moves every joint.
- The base and elbow potentiometers read up to about ±0.5° of noise; median several readings.

## Open questions

- Where exactly the jaws' grip centre is when closed on a 2.4 mm body, against the marker: is
  "tip on the crosshair" the same point the jaws close on?
- Is the 0.5° settling deadband (about 1 mm) good enough, or is a final nudge (re-sending the move)
  needed for picks?
- Does the plate fit (`rectify.recalibrate`) still work with the arm in view, for checks between picks?
- Does the gripper hide the tip marker at some poses (the far corners, low hover)?

## Files to read first

`robot_control/vision/rig.py`, `robot_control/vision/detector.py`, `robot_control/vision/camera.py`,
robot_offline `src/robot_offline/pre/rectify.py`, `robot_control/config/{workspace,robot_real,bins_real}.toml`,
`robot_control/config/grippers/parallelogram_v3.toml`, `scripts/reach_test.py`, `scripts/base_reach.py`,
`hardware_api/scara_motion_controller_api.py`, `course_files/lab_scripts/RobotSetup.md`.
