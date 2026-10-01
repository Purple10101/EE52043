# 05 Motion

**Status:** Draft · **Stage:** 1 · **Depends on:** 01, 02, 03, 04

## Goal

Turn "pick at this tool pose" or "place in this bin" into a safe series of robot and gripper commands,
and refuse the whole operation before moving if any part of it is impossible.

## Scope

**In:** running pick and place sequences, safe-height travel, home and park, and startup recovery.

**Out:**
- Choosing targets or grasps (planning and grasp strategy, stage 2).
- Retries (Supervisor). Motion reports what happened; it does not decide what to do next.

## Design

### Validate everything first

Running a sequence has two phases:

1. **Expand:** turn every step into concrete commands (tool poses with heights resolved, driver calls,
   waits), then run every pose through kinematics: tool transform, yaw choice, IK, limits.
2. **Execute:** only if every pose is valid. Otherwise return a refusal naming the step and reason,
   having sent nothing.

So the arm never stops halfway through a pick because a later pose turned out to be unreachable.

### Safe travel rules

- **Never change height and x/y in the same command.** A single `SCARA_MOVE_COORD` interpolates in
  joint space, so the tool's path between two poses is a curve, not a straight line. Keeping vertical
  and horizontal moves separate means the path only ever curves at travel height, above everything.
- **All horizontal travel happens at the travel height** from config.
- **Vertical moves use `move_slide`,** so x/y and yaw cannot drift during them.

### Durations

Stage 1 uses fixed durations per move type from config: travel, vertical, and slow (near the table).
Later this can scale with joint distance, using `last_commanded` as the start point.

### Home, park and startup

- **home:** go to the configured home joints via travel height.
- **park:** move to the configured park pose, out of the camera's view. The Supervisor calls this
  before every snapshot.
- **startup:** the arm's position is unknown at power-up, and the sim cannot report it. So the first
  command is always a vertical-only move to travel height, then home.

### Result

Every operation returns one of:
- **completed**
- **refused** (step and reason, nothing sent)
- **hardware error** (step reached, error)

The Supervisor decides what to do with it.

## Config it needs

Travel height, home joints, park pose and move durations (workspace); named heights and sequences
(gripper model); plus everything kinematics needs.

## Done when

Unit tests, using RecordingRobot and SimGripper:
- [ ] A pick at a reachable pose produces the commands in order, every x/y move happens at travel
      height, and no command changes height and x/y together.
- [ ] A pick whose descend step is unreachable is refused, with nothing recorded.
- [ ] Startup's first recorded command is a vertical-only move.

In the simulator:
- [ ] Startup, home, then pick at three table poses and place each in a different bin with the dummy
      gripper, with no out-of-bounds errors in the Unity console.
- [ ] Park moves the arm to the configured pose.

## Open points

- Travel height has to clear the bin rims, which needs the real bin dimensions.
- Whether real-arm moves need slower durations near the table than the sim. Expect yes.
