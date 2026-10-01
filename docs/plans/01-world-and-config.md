# 01 World and config

**Status:** Draft · **Stage:** 1 · **Depends on:** nothing

## Goal

Give every other module the same data types and the same validated settings, so nothing defines its
own copy of a pose or a joint limit.

## Scope

**In:**
- `world/`: pose types, Bin, Detection with derived geometry, and the remaining record types as
  empty definitions so later stages have fixed shapes to fill.
- `config/`: TOML settings files and a loader that validates them at startup.

**Out:**
- Pixel-to-robot mapping (vision, stage 3).
- BinAllocation persistence (stage 2).
- Cross-module checks such as "every bin is reachable". These need kinematics, so they run in a
  startup check owned by `task/`, not in the loader.

## Design: world/

Plain immutable records (frozen dataclasses). No behaviour except deriving geometry.

| Type | Fields | Built in stage 1? |
|---|---|---|
| Pose2D | x, y (m), yaw (deg) | Yes |
| ToolPose | x, y (m), yaw (deg), height above table (m) | Yes |
| JointState | j0, j1, j2 (deg), slide (m) | Yes |
| Bin | id, drop x/y, drop height, is_reject | Yes |
| Detection | snapshot_id, outline_px, outline_robot, then derived: centre, yaw, length, width, plausible | Yes |
| Snapshot | id, image, capture time, detections | Shape only |
| Classification | value (Ω or unknown), confidence, bands | Shape only |
| GraspCandidate | tool pose, score, clearance margin | Shape only |
| PickMemory | last_target, attempts, skipped | Shape only |

### Detection geometry

Detection is the only place geometry is derived. It is created from the two outlines plus the list of
known package sizes, and works out everything once:

- **Outline storage:** N×2 numpy arrays, pixels and metres.
- **Rectangle fit:** `cv2.minAreaRect` on `outline_robot` (as float32) gives centre and side lengths.
  `length` is the longer side, `width` the shorter.
- **Yaw:** compute it from the rectangle's corner points (`cv2.boxPoints`), as the direction of the
  long side in the robot-frame angle convention, normalised to (−90°, 90°]. Do not use the angle
  `minAreaRect` returns: its convention changed between OpenCV versions.
- **Plausible:** true if length and width are within tolerance of any configured package.

Detection does not know how pixels map to metres. Vision passes both outlines in.

## Design: config/

TOML files read with `tomllib` (standard library in Python 3.11, already in use), so no new
dependency. The loader turns each file into frozen settings objects and fails at startup with a clear
message naming the file and key if anything is missing or inconsistent.

| File | Contents |
|---|---|
| `robot_sim.toml` | host, port, reply timeout; link lengths; joint limits; slide range 0.025–0.075 m; settling time; PID values (sent, ignored by the sim); slide direction; capabilities |
| `robot_real.toml` | serial port, baud, timeout; the same robot fields; slide range (course example 0.095–0.15 m, confirm); slide servo angles at z_min/z_max (90° and −60° per `servo_controller.c`) |
| `workspace.toml` | table reference height, travel height, home joints, park pose, default move durations, bins, resistor packages (size and tolerance) |
| `grippers/<name>.toml` | one gripper model per file (fields in plan 04) |

The backend is chosen at run time (`--backend sim|real`, default sim), not by editing files.

Validation the loader does on its own: required keys and types, min < max for every limit, link
lengths positive, slide range inside the backend's hard range, at least one bin, exactly zero or one
reject bin.

Starting package values (typical datasheet sizes, measure the real parts): 1/4 W about 6.3 × 2.4 mm,
1/2 W about 9.0 × 3.2 mm.

## Done when

- [ ] Detection derives centre, yaw, length and width correctly for rectangles at 0°, 45°, 90° and
      −30°, and yaw is always in (−90°, 90°].
- [ ] A 1/4 W-sized outline is plausible. A 15 × 15 mm blob and a 2 × 1 mm speck are not.
- [ ] Both robot files and the workspace file load. A file with a missing key or min > max fails with
      a message naming the file and key.
- [ ] Unit tests for all of the above run with no hardware.

## Open points

- Table size, bin positions and park pose need real measurements. Use placeholder values marked
  `# placeholder` until then.
- Slide direction on the real arm (does a larger z raise the tool?). In the sim, larger z is retracted
  (higher).
