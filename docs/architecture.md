# Architecture

Software structure for the resistor sorter: a fixed camera finds through-hole resistors on the table,
and the SCARA arm picks them up one at a time and drops each into the right bin.

This file is the source of truth in the repo. It was agreed through review of the
[architecture page](https://claude.ai/artifact/CDciGTU6csatooetXW5sMR) (private; shared on request).
Status markers: **Agreed** (settled), **Proposed** (needs sign-off), **Parked** (noted, not planned),
**Open** (undecided or needs staff input).

## Decisions

| Status | Decision |
|---|---|
| Agreed | **The problem is planar.** Resistors lie on a flat table at a known height. Perception produces 2D poses in the robot's x/y frame via a pixel-to-table mapping, with no 3D reconstruction. |
| Agreed | **Re-scan after every pick.** The arm parks out of view, the camera takes a fresh snapshot, and the next target comes from that. Disturbances are tolerated and each grasp is verified for free. |
| Agreed | **Each resistor is stored as the outline of its body.** Leads are ignored. Orientation comes from the outline. |
| Agreed | **Outlines are kept in both frames:** pixels (for classification and overlays) and robot metres (for motion). |
| Agreed | **Derived geometry is computed once, in one place:** centre, yaw, length, width and plausibility, when a Detection is created. |
| Agreed | **Nice-to-have: no hard-coded body sizes.** Size-dependent rules read the size from the Detection, so different packages (1/4 W, 1/2 W) can work. Drop this and assume one size if it becomes very hard. |
| Agreed | **Works for any gripper design.** A new gripper means a new driver and a config profile, nothing else. |
| Agreed | **Classification is required but built later**, behind a fixed interface. No code assumes band reading will work. The method is undecided; a neural network is not assumed. |
| Agreed | **Python kinematics mirrors the firmware rather than driving the arm.** IK already exists in `scara.c` and `sim_scara_controller.cs`. Our copy checks reach and limits before anything is sent, and applies the gripper tool offset. |
| Proposed | **No tracker module.** Classification reruns every scan. The only state carried between scans is PickMemory (and BinAllocation, below). |
| Proposed | **Dynamic bin allocation.** See [Bins](#bins). |

## Modules

```
robot_control/
  world/        data types only, depends on nothing
  config/       TOML settings: robot backends, workspace, bins, gripper profiles
  hardware/     Robot interface: SimRobot (TCP), RealRobot (serial), RecordingRobot (tests)
  kinematics/   IK/FK mirror, limits, reach, tool transform. Pure maths
  gripper/      driver interface, GripperModel (config), grasp strategy
  motion/       runs pick/place step sequences at safe heights
  vision/       Camera (live/replay), Detector, Calibration
  classify/     Classifier interface + Stub
  planning/     target ranking, bin allocation
  task/         Supervisor: the work cycle, PickMemory, failure handling
```

Layers depend only downward. The Supervisor makes every call and passes results on, so no module
imports another sideways (vision never imports classify).

## Data flow in one cycle

| From → To | Data | Contents |
|---|---|---|
| Camera → vision | Frame | One still image, captured after the arm has parked |
| vision → planning | Detection[] | Per resistor: outline (px and robot m), centre, yaw, length, width, plausible |
| vision → classify | Crop | Per detection: pixels inside its pixel outline, rest masked |
| classify → planning | Classification | Value in Ω or unknown, confidence, raw bands if read |
| Supervisor → planning | PickMemory | Last target centre, attempt count, skipped positions |
| planning → gripper | Target | Chosen Detection, neighbours' outlines, destination Bin |
| gripper → kinematics | GraspCandidate[] | Ranked tool poses (x, y, yaw, z), each clear of neighbours |
| kinematics → motion | Grasp pose | First reachable candidate, yaw flipped 180° if that keeps the wrist in range |
| motion → Robot | Wrist pose | Each step's tool pose converted for `SCARA_MOVE_COORD` |
| motion → Gripper driver | Command | Open, grip, release, in the gripper model's sequence order |

`config/` supplies static values to every module.

## The work cycle

1. Park arm out of view
2. Capture snapshot
3. Detect body outlines
4. Map to robot frame, derive pose
5. Check last pick against PickMemory
6. Classify every detection
7. Rank all detections
8. Take the first with a clear, reachable grasp
9. Run pick sequence
10. Run place sequence at bin
11. Back to 1 until the table is empty

Steps 7 and 8 form a loop: planning ranks (skipped positions excluded, then higher confidence and more
isolated first), and the Supervisor walks the list asking the grasp strategy and kinematics for a valid
grasp. Planning never calls them itself.

### Failure handling (Proposed)

Behaviour inside the Supervisor and planning, not structure. It relies on PickMemory, the Detection
plausibility flag and the optional grip-sensing capability, all designed elsewhere.

| Situation | Detected by | Response |
|---|---|---|
| Grasp failed | A detection within a few mm of the last target on the next scan (plus grip sensing if available) | Retry with the next candidate. After N attempts, add to PickMemory's skip list |
| Resistor disturbed | Next scan | Nothing special |
| Target unreachable | Kinematics check before any move | Skip and flag, never send |
| No clear grasp | Footprint overlaps a neighbour at every candidate | Try the next resistor in the ranking |
| Value unknown or low confidence | Classifier confidence | Re-image or reject bin, per policy |
| Odd shape | Plausibility check | Ignore and log the snapshot |

## Data model

All robot-frame lengths are metres, all angles degrees.

- **Detection**: snapshot_id, outline_px, outline_robot, then derived once: centre (x, y), yaw (mod 180),
  length, width, plausible.
- **Classification**: value (Ω or unknown), confidence (0–1, method-agnostic: colour-match margin,
  valid standard value, agreement between frames; not necessarily a neural network), bands.
- **Snapshot**: id, image, capture time, Detections.
- **GraspCandidate**: tool pose (x, y, yaw, z), score, clearance margin.
- **PickMemory**: last_target (x, y), attempts, skipped positions.
- **Bin** and **BinAllocation**: see below.

### Bins

**Proposed.** Split into fixed and run-time parts:

- **Bin** (config): id, drop point, drop height, reject flag.
- **BinAllocation** (run state): which value each bin holds. Saved to a small JSON file as it changes,
  so a restart mid-run keeps the mapping.

Rules: only a confident reading may claim an empty bin; a bin is claimed when the resistor is dropped,
not when it is targeted; one bin is kept back for unknown values and overflow. Policies are swappable:
first-come, or allocate everything after the first scan (rarest values go to overflow, bins can be
ordered by value).

## Gripper abstraction

| Part | Kind | Holds |
|---|---|---|
| Driver | Code | Open, grip, release; declares capabilities such as grip sensing. Sim no-op version |
| Model | Config | Tool offset (dx, dy, dz), grip depth, approach height, open footprint, 180° symmetry, max part width, timing, wrist limits if cables restrict it, pick and place step sequences |
| Grasp strategy | Code, small | Outline + model → ranked candidates. Default: body centre along the long axis, plus yaw+180°, small yaw and axial tweaks. One default strategy driven by the model; a second only if a design needs it |

Pick sequences are data built from a fixed step vocabulary (move above, descend, open, grip, release,
wait, lift), so jaws and vacuum differ only in config.

### What the gripper changes in kinematics and hardware

- **Length below the wrist** converts tool height to slide position.
- **Off-axis grip point** makes the wrist position depend on yaw. An on-axis design avoids this.
- **Symmetry** decides whether yaw + 180° is available to stay inside wrist limits.
- **Cables** may narrow the usable wrist range.
- **Mass** affects settling, PID gains and move times.
- **The Z slide travels only about 5 cm.** Gripper length must let the tip reach table height plus
  half the body diameter at the bottom of the stroke, and clear bin rims at the top. Measure the table
  height relative to the slide before finalising the CAD.
- **Actuation:** neither API has a gripper command and the firmware uses all four PWM channels.
  Either add a firmware command or use a separate microcontroller (Open).

## Kinematics and hardware facts

From the course sources (`scara.c`, `servo_controller.c`, `sim_scara_controller*.cs`):

- Links 0.125 m and 0.100 m. One elbow configuration (θ2 = π − φ3, always ≥ 0).
- Joint limits match between sim and real: J0 −90…90°, J1 0…120° (real: −90…30° servo range with a
  90° offset), J2 −180…180°. Reachable band roughly 115–225 mm from the base.
- **Neither backend protects itself.** The real firmware does no range checks: unreachable targets
  give NaN angles, and `servo_set_angle` does not clamp, so out-of-range commands drive servos past
  their limits. The sim logs an error and moves anyway. Our kinematics check is the only guard.
- **Z ranges are inconsistent.** Sim joint moves check 0.025–0.075 m, sim coordinate moves check
  0.01–0.1 m and clamp to the `SCARA_INITIALISE` z_min/z_max. The real firmware maps z linearly from
  z_min/z_max onto the slide servo angle without clamping. Course example values: real 0.095–0.15 m.

| | Simulator | Real arm |
|---|---|---|
| Transport | TCP 127.0.0.1:9000 | Serial, 115200 baud |
| Read angle / coords | Not available | Available |
| Auto-calibrate | Not available | Available |
| Failure flag | Always success, not checked by the API | Raises on failure |

## Testing seams

- **Fake perception:** made-up detections so the full loop runs in the sim.
- **Replay camera:** folder of real photos for vision work without the arm.
- **Sim gripper driver:** no-op so sequences run.
- **RecordingRobot:** records commands instead of moving, for unit tests of motion.
- **Unit tests** for kinematics, outline geometry, PickMemory checks and allocation.

## Build order

1. hardware, kinematics, motion (with world, config and a dummy gripper model): arm moves to commanded
   tool poses in the sim, with limit checks.
2. Supervisor with fake perception and planning: full cycle in the sim.
3. Camera, calibration, detection.
4. Gripper drivers and profiles.
5. Classifier.

Stages 3 and 4 can run in parallel. Plans live in [plans/](plans/README.md).

## Parked

- **Pre-tightening:** close the gripper partway before descending when the open footprint would hit a
  neighbour. Candidates would carry an opening width; the driver's open takes a width (capability).

## Open questions (for staff)

- How many bins, how many distinct values, and is the value-to-bin mapping given?
- Is the camera fixed relative to the robot base or the table?
- May we modify the firmware to add a gripper command?
- Table size and position relative to the base.
- 4-band only, or 5-band too? One power rating or several?
