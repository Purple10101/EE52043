# 02 Hardware

**Status:** Draft · **Stage:** 1 · **Depends on:** 01

## Goal

One `Robot` interface that hides every difference between the simulator and the real arm, so nothing
above `hardware/` knows or cares which one is connected.

## Scope

**In:** the Robot interface, SimRobot, RealRobot, and RecordingRobot (for tests).

**Out:**
- Gripper actuation (`gripper/` drivers, plan 04).
- Reachability checks (kinematics, plan 03).
- The course API files in `hardware_api/`: these are wrapped, never edited.

## Design

### The interface

| Operation | Meaning |
|---|---|
| initialise | Send `SCARA_INITIALISE` with values from config. Real arm: optionally auto-calibrate |
| move_coord(wrist pose, duration) | `SCARA_MOVE_COORD`. Pose is x, y, yaw (deg), slide (m) |
| move_joints(joint state, duration) | `SCARA_MOVE_JOINTS` |
| move_slide(slide, duration) | `SCARA_MOVE_JOINT` on axis 3 only, for vertical-only moves |
| last_commanded | The last pose and joint state sent. Both backends track this, because the sim cannot read back |
| read_joints / read_pose | Real arm only, reported as a capability. Sim reports it cannot |
| close | Release the socket or serial port |

All calls block until the backend replies (the sim replies only after the move and settling time).
Any failure raises one error type (HardwareError) with a clear reason.

### Units are consistent at this interface

The backends disagree on what axis 3 means in joint moves:

- **Sim:** axis 3 is the slide position in metres (course example `SCARA_MOVE_JOINT(3, 0.075, 1)`).
- **Real:** axis 3 is the slide servo angle in degrees (course example `SCARA_MOVE_JOINT(3, -60, 1)`).

The interface always takes metres. RealRobot converts to servo degrees using the firmware's own linear
mapping: z_min ↔ 90°, z_max ↔ −60° (`servo_controller.c:144`, `scara.c:153`). Coordinate moves take
metres on both backends.

### Last-line checks in every adapter

Neither backend protects itself. The real firmware has no range checks and `servo_set_angle` does
not clamp. The sim logs an error and moves anyway. Kinematics is the main guard, but each adapter
also refuses, before sending:
- any non-finite value (NaN, inf)
- any joint command outside the configured joint limits
- any slide value outside the configured slide range

Full reachability of coordinate moves stays in kinematics, so hardware does not depend on it.

### SimRobot

- Wraps `sim_scara_motion_controller`.
- Sets a reply timeout on the socket (the course API waits forever otherwise; the joint test used
  30 s).
- The sim always replies success and the course API does not check the flag, so a timeout is the only
  failure it can report.
- Connection refused → HardwareError telling the user to start the sim, open `main_scene`, press Play
  and start the server.

### RealRobot

- Wraps `scara_motion_controller`.
- Serial port and timeout from config. The course API already raises on a bad flag or timeout;
  convert both to HardwareError.
- Read-back available. Use it to report actual vs commanded position in logs.

### RecordingRobot

Implements the interface by appending every command to a list and updating `last_commanded`. It never
moves anything. Motion tests use it to check command order and heights.

### Course API printing

Both course APIs print on every command. Leave this on during stage 1, since it helps debugging.
Revisit (redirect to logging) when the Supervisor's own logs arrive.

## Config it needs

`robot_sim.toml` and `robot_real.toml` from plan 01.

## Done when

- [ ] With the sim running: initialise, move to home joints, move_coord to three reachable poses, and
      move_slide up and down, with no out-of-bounds errors in the Unity console.
- [ ] With the sim not running: a clear HardwareError, not a traceback from the socket.
- [ ] Each adapter refuses a NaN, an out-of-range joint and an out-of-range slide without sending
      anything (unit-tested via RecordingRobot and a fake transport).
- [ ] RealRobot's metres-to-servo-degrees conversion is unit-tested at z_min, z_max and the midpoint.
- [ ] Real arm, when available: the same moves as the sim test, at slow durations, with someone at
      the power switch.

## Open points

- Does the real arm need `SCARA_AUTO_CALIBRATE` every power-up? The course script has it commented
  out.
- Slide range and direction on the real arm need measuring (shared with plan 01).
