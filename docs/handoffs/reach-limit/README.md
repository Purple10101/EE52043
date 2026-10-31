# Handoff: the arm's reach limit over the pick plate

From the session of 2026-10-31 (branch `20261031-vision-frontend`, not committed). For the session
investigating whether, and how, the reach limit can be extended. Everything below names the file it
came from; "measured" means read from the CAD with the scripts in `scripts/`, "computed" means worked
out from those numbers, "inferred" means not checked against a source.

## The problem in one paragraph

With the firmware's elbow limit of 120°, the arm can't get closer than 114.6 mm to its base axis, so
**31% of the pick plate (the side nearest the arm) is out of reach**. The links themselves only touch
at about 141°, and 140° would cover the whole plate. The 120° comes from how the firmware maps the
elbow servo's pulse range, not from a collision. `scara_control/scara_workspace.py` (Alex's
controller) is stricter still, at 90° (77% of the plate out of reach, including its centre).

## Frames (read this first)

- **Scene frame:** mm, origin at the robot base axis, z up. The frame of the arm and rig CAD, and of
  robot_offline's rectified plate mm. Source: `C:\Users\joshp\Y4S1\Robot Resources\arm_and_rig_scene\README.txt`
  ("All three files share one frame: z up, origin at the centre of the robot base").
- **Robot frame:** the firmware's, metres, angle 0 along +y, positive toward +x
  (`course_files/scara/servo_motion_controller/Core/Src/scara.c`, `calc_inverse_kinematics` at line
  139 and forward kinematics above it).
- **Robot = scene turned 180°.** Evidence: the arm lies along scene −y at zero
  (`Robot Resources\camera_tower\README.md`, "Arm clearance": "the sim's servo frame is the rig frame,
  and the arm lies straight along −y at zero ... joint angles turn the links by −q about +z"), and the
  firmware IK of robot (0, 0.130) m gives base −46.13°, elbow 110.43°, matching the CAD pose over the
  plate centre (scene (0, −130)) in `arm_and_rig_scene\README.txt` ("base -46.12 deg, elbow 110.42
  deg"). Not yet checked on the real arm (a mirror would also match the magnitudes).

## What was measured and where

| Fact | Value | Source |
|---|---|---|
| Pick plate top | x −50…50, y −180…−80, z −22 mm | `arm_and_rig_scene\EduSCARA_resistorrig.obj`, body `Body1:1` (OBJ is in cm), `scripts/rig.py`. Matches `robot_offline\configs\dataset.toml [rectify] plate_corners_mm` |
| Plate near edge | 80 mm from the base axis; far corners 187 mm | computed |
| Robot base | 150 × 150 mm square, x/y ±75, z −50…15 (parts `base_0`, `base_1`); column to z 135 | arm STEP below, `scripts/steptree.py` |
| Other rig bodies | cradle `Body1:2` x −130…−55, y −180…−135, top z −17; trays `Body1:3` x −180…−80, y −25…75 and `Body1:4` y −130…−30, top z −40; corner block `Body1:5` (camera tower foot) x/y −180…−135 | rig OBJ |
| Base axis, elbow axis, Z shaft | (0, 0), (90.1, −86.6), (0, −130) in the STEP pose | bearing bores (vertical cylinders) in `arm_and_rig_scene\EduSCARA arm - no gripper (plate reach pose).step`, parts `link1_0`, `link2_0/1`, `link2_4`, `zpos_0`; `scripts/axes.py` |
| Link 1, link 2 | 124.97 mm, 100.0 mm | computed from the axes; firmware uses 0.125 / 0.100 (`scara_control/scara_controller.py` init defaults) |
| Elbow servo horn | 25-tooth spline ("SERVO ARM HORN 25T v1") | part name in the arm STEP; 14.4° per tooth is inferred |
| Elbow at which the links touch | 141.0° (first overlap `link1_0` × `link2_1`, 1.9 mm³), the same at base −90, −46.12, 0, 45, 90; no clash with the base column before that | `scripts/elbow.py`: link 2 rotated about the elbow axis in 2° then 0.5° steps, solid intersection volumes with OpenCascade. Arm only, no gripper, Z at the STEP pose. Two overlaps present at every angle (elbow pot `pot_1` in `link2_1`'s bore, a screw in `servo_1`) are joint internals and were excluded |
| Gripper offset from the wrist axis | 0 (centred) | `Robot Resources\gripper\v3_final\CAD\Gripper v3a.1 - Bolt-on Tips.step`: socket bore r 9.1 and four M2 holes on r 7 at (0, 0), matching the shaft's spigot r 9.0 and hole circle r 7 (`scripts/axes2.py`); closed tips meet at x = 0. Also `gripper\v3_final\Docs and evidence\Print and assembly guide.md` line 151: "this gripper mounts centred on the Z-shaft" |
| Fingertips below the shaft's bottom face | 64 mm | gripper STEP (tips at z 55, socket floor at z −9, gripper frame) |
| Gripper footprint around the axis | x −22…+51 mm (Base, pinion, SG90 on the +x side, along the jaw-closing direction), y −12…+36 (SG90) | gripper STEP part bounds |
| Arm's elbow underside | `link2_0`: a disc of radius 50 around the elbow axis, z 37…42 | arm STEP |
| Slide stroke | shaft bottom z ≈ 37…87 (tips touch the plate at 42) | inferred: sim slide 25…75 mm, and `gripper\v3_final\Docs and evidence\plate reach evidence.png` ("Z (s4) = 30 mm: tips touch the plate (z −22)"); the STEP pose has the shaft bottom at z 40 |

## Where the 120° comes from

- **Firmware the SCARA path uses:** `course_files/scara/servo_motion_controller/Core/Src/servo_controller.c:141-144`,
  `servo_controller_scara_init` (called from `scara.c:41`):
  `servo_init(_axis_1, 2500, 500, 1500, -90, 30, 90, 0, 1)`, i.e. min/max/zero pulse 2500/500/1500 µs,
  servo angle −90…30°, offset 90°. Joint angle = servo angle + 90, so the joint range is 0…120° and
  120° is already the 500 µs end of the pulse range.
- The other init in the same file (`servo_controller_init`, lines 32-35) has `-90, 45` (joint 0…135).
  It looks unused by the SCARA path; check.
- **Pulse mapping:** `servo.c`, `servo_set_angle` (around line 100): below the zero the slope is
  (1500 − 2500) / 90 = −11.1 µs/°, above it (500 − 1500) / 30 = −33.3 µs/°. The two halves differ by
  3×, so the "angle" scale is not linear in pulse width. Whether joint 120° really is where the servo
  sits at 500 µs depends on the servo and on the potentiometer feedback (PID gains in
  `initialize_robot`); unverified. The servo model is unknown.
- **The firmware on the lab board** is a modified build whose source exists only in `git stash@{0}`
  (`firmware/Core/Src/*`, "firmware safety, calibration, RealRobot, range-of-motion test"). Same
  `servo_init` values (`git show stash@{0}:firmware/Core/Src/servo_controller.c`, lines 141-144); its
  `servo.c` adds `servo_joint_in_range`, so it **refuses** moves outside the limits. Raising the limit
  means editing that source and reflashing. `docs/hardware-api.md` ("Which firmware is on the board")
  describes it.
- **Other places that state limits:** `docs/architecture.md` ("J1 0…120° (real: −90…30° servo range
  with a 90° offset)", reach "roughly 115–225 mm"); `docs/plans/03-kinematics.md` (same);
  `scara_control/scara_workspace.py` `WorkspaceLimits.j1 = (0.0, 90.0)` with reach 160–225 mm; this
  branch's `robot_control/config/robot_real.toml` (`j1_deg = [0.0, 120.0]`). The camera tower's sweep
  used the Unity sim's limits, elbow 0…120 (`camera_tower\README.md`, "Arm clearance").

## Results

**Plate out of reach by elbow limit** (computed, `scripts/reach.py`: 0.5 mm grid over the plate with
its 5 mm corner radius, the firmware's IK in the robot frame, base ±90°):

| Elbow limit | Out of reach | Note |
|---|---|---|
| 0–90° | 77.4% | Alex's `scara_workspace.py` |
| 0–120° | 30.8% | firmware now; reach limit 114.6 mm. Agrees with `camera_tower\README.md` ("About 30% of the pick plate ... out of reach") |
| 0–130° | 13.4% | |
| 0–134.4° | 5.9% | the window moved one spline tooth (14.4…134.4) |
| 0–140° | 0.1% | |
| 0–150° | 0% | but the links touch at 141° |

Losing the bottom of the range costs little: elbow 14.4° still reaches 223 mm. The farthest rig point
I checked (tray `Body1:3`'s corner, 195 mm) needs elbow ≥ about 60°, but I did not check it against
the base limit; `camera_tower\README.md` says 27% of tray `Body1:3` is out of reach.

**With the gripper, allowing elbow ≤ 141°** (`scripts/platereach.py`: 10 mm grid of plate cells; the
arm STEP posed by IK; the gripper STEP bolted under the shaft; jaw direction every 30° over 360°;
shaft bottom at z 42 (tips on the plate), 62 and 87; jaws shut, and "open" approximated by moving the
carriers and tips 12.5 mm outward; obstacles: the arm's base, link 1, link 2 and the rig bodies as
bounding boxes, the pick plate excluded):

- Joint limits alone: 100% of cells reachable.
- Every resistor angle grippable (jaw or jaw + 180° clear): **88%** of cells; some angle: 100%.
- Problem cells: the middle of the near edge (scene y −85, x −15…25: only 1–4 of 6 angles) and the
  x = −45 edge next to the cradle (1–4 of 6).

Map printed by the script (rows scene x, far edge at top; columns scene y −180 left to −80 right, the
base side; `o` all 6 resistor angles OK, digit how many of 6, `*` needs elbow 120–141°):

```
  x=  45.0 o  o  o  o  o  o  o  o* o* o*
  x=  35.0 o  o  o  o  o  o  o  o* o* o*
  x=  25.0 o  o  o  o  o  o  o  o* o* 4*
  x=  15.0 o  o  o  o  o  o  o  o* o* 2*
  x=   5.0 o  o  o  o  o  o  o  o* o* 1*
  x=  -5.0 o  o  o  o  o  o  o  o* o* 1*
  x= -15.0 o  o  o  o  o  o  o  o* o* 2*
  x= -25.0 o  o  o  o  o  o  o  o* o* 4*
  x= -35.0 o  o  o  o  o  o  o  o* o* o*
  x= -45.0 2  1  1  1  3  4  o  o* o* o*
```

What blocked (pairs counted over all poses tried): gripper `Base` / `SG90` × `link2_0` (the elbow
disc) most; gripper `Base` / jaw carrier × robot base (`weight_1250`, `base_1`, `base_2`) near the near
edge; open tips × cradle along x = −45 (the cradle as its bounding box, so pessimistic).

**Consequence for the wrist:** the SG90 end must not point toward the elbow, or it hits the elbow
disc. Of yaw and yaw + 180°, only one is safe for a given pose. Not yet in `robot_control/kinematics`,
and the jaw direction at tool angle 0 is still unmeasured (`robot_control/config/grippers/parallelogram_v3.toml`).

## Caveats

- CAD only. The real arm can differ: the base servo already has a ~24° offset between servo zero and
  potentiometer zero, landing up to 13° short (`docs/hardware-api.md`, "Known issues", from the
  2026-10-05 tests; `logs/range_of_motion_20261005_*.txt` in this repo are that test's logs).
- The link-contact sweep had no gripper and Z at one height; the gripper check used three heights.
- Rig obstacles are bounding boxes. The camera tower itself was not loaded (only a box over its foot,
  up to z 260); the tower's own sweep says the plate isn't affected (`camera_tower\README.md`).
- The 141° sweep is about collisions only; servo travel, cable routing and the elbow pot's range were
  not checked.

## Options to look into

1. **Measure the real elbow first:** with servo power off, how far does the elbow bend by hand before
   the links touch? With power on, slowly command toward 120° and read `read_joint_angle(1)` (the
   potentiometer) to see where 120° really lands.
2. **Shift the elbow's window:** re-seat the horn one tooth (14.4°) so the same pulse range covers
   about 14…134°, and update the firmware's elbow `offset_angle` / limits (stash source), the pot
   calibration, `robot_real.toml`, and `scara_workspace.py`. 5.9% of the plate would stay out of reach.
3. **Firmware only:** if the servo turns further than the firmware assumes at 500 µs, a wider pulse
   range or a corrected angle scale might reach 135–140° without touching the hardware. Needs the
   servo model and a bench test.
4. **Move the plate:** it's 80 mm from the base axis; moving it about 35 mm away from the base would
   put it all beyond 114.6 mm (check the far side against the 225 mm reach, the trays and the tower).
5. **Accept it:** keep resistors out of the near 30% (mark the line on the plate). The frontend
   already refuses those targets.

## Scripts (`scripts/`)

Copied from the session's scratch folder. Quick scripts, not project code. They need OpenCascade's
Python bindings, kept out of the project venvs:

```
py -3.11 -m venv cadenv
cadenv\Scripts\python -m pip install cadquery-ocp numpy
cadenv\Scripts\python steptree.py "<arm STEP>" arm_parts      # lists parts, saves each as .brep + index.txt
cadenv\Scripts\python steptree.py "<gripper STEP>" grip_parts
cadenv\Scripts\python axes.py arm_parts link1_0 link2_0 zpos_0   # vertical bores per part
cadenv\Scripts\python elbow.py arm_parts "-90,0,90"             # NB: its loop was narrowed to 138-142.5 deg in 0.5 steps
cadenv\Scripts\python platereach.py .  10                        # needs arm_parts/ and grip_parts/ in the given folder
python reach.py                                                  # plain Python: plate reach by elbow limit
python rig.py "<rig OBJ>"                                        # rig body bounds in mm
```

OCP 8 differences from older pythonocc examples: `TDF_LabelSequence` is
`OCP.OCP.collections.Sequence_TDF_Label`, `TopoDS.Face(...)` (no `_s`), and `Bnd_Box.Get()` doesn't
convert (use `CornerMin()` / `CornerMax()`).

## Related code on this branch

- `robot_control/kinematics/arm.py`: the firmware IK mirror; `Kinematics.reach()` gives the limits;
  refusals name the elbow angle.
- `robot_control/config/robot_real.toml`: the joint limits the frontend uses.
- `docs/plans/06-vision-front-end.md`: the same measurements, summarised.
- `python -m robot_control.task.live_plan` draws the 114.6 mm reach limit on the live view.
