# 03 Kinematics

**Status:** Draft · **Stage:** 1 · **Depends on:** 01

## Goal

Predict exactly what the firmware will do with a target, and refuse any target that would be
unreachable or break a joint limit, before anything is sent. Also convert where the gripper should be
into where the wrist must go.

The firmware does its own IK on every `SCARA_MOVE_COORD`. This module does not replace it; it mirrors
it so we can check the result first.

## Scope

**In:** forward kinematics (FK), inverse kinematics (IK), joint-limit and reach checks, choosing yaw or
yaw + 180°, wrist-angle wrapping, the tool transform, and slide height conversion.

**Out:** anything with I/O. This module is pure maths and never talks to hardware.

## How the IK works

Same maths as `calc_inverse_kinematics` in `scara.c:139` and the sim. For a target (x, y):

1. **Distance to target:** r = √(x² + y²).
2. **Elbow angle (law of cosines):** the two links and the line to the target form a triangle with
   sides l1, l2, r. The angle between the links is φ3 = acos((r² − l1² − l2²) / (−2·l1·l2)). The elbow
   joint is how far the arm bends from straight: θ2 = 180° − φ3.
3. **Bearing of the target:** φ2 = atan2(x, y), measured from +y toward +x.
4. **Shoulder offset (law of cosines again):** the angle at the shoulder between link 1 and the line to
   the target is φ1 = acos((l2² − r² − l1²) / (−2·r·l1)). The shoulder joint is θ1 = φ2 − φ1.
5. **Wrist:** the tool's yaw is the sum of the three rotations, so θ3 = yaw − (θ1 + θ2).

There is only one elbow solution: θ2 is always ≥ 0, so the elbow always bends the same way.

**Reachable** means the acos arguments lie in [−1, 1] (that is, |l1 − l2| ≤ r ≤ l1 + l2) and every
joint is inside its limits. With J1 limited to 0–120°, the real reachable band is about 115–225 mm from
the base, further narrowed by J0's ±90°.

FK is the reverse and is used for checks and tests: x = l1·sin θ1 + l2·sin(θ1 + θ2),
y = l1·cos θ1 + l2·cos(θ1 + θ2), yaw = θ1 + θ2 + θ3.

## Design

### IK result

IK returns either the joint state, or a refusal with a reason that names the problem (out of reach,
too close, J1 above limit, slide below range...). It never returns NaN. Motion logs the reason.

### Yaw choice and wrist wrapping

The firmware computes θ3 = yaw − (θ1 + θ2) and does not wrap it. Sending yaw = 170° when θ1 + θ2 = −30°
asks for θ3 = 200°, outside ±180°. So before sending, kinematics:

1. Wraps the requested yaw by ±360° (the same physical orientation) so θ3 lands in range.
2. If the gripper model is 180°-symmetric, also tries yaw + 180°.
3. Picks whichever valid option needs the smaller wrist rotation from the last commanded wrist angle.
4. Refuses if neither fits (only possible when cable limits narrow the wrist range).

The yaw sent to the firmware is the adjusted one, so the firmware computes a θ3 we already checked.

### Tool transform

The gripper model gives the grip point's offset from the wrist axis: dx, dy in the tool frame and dz
(length below the wrist).

- **Position:** wrist x/y = tool x/y minus (dx, dy) rotated by the tool yaw. For an on-axis gripper
  (dx = dy = 0) the wrist and tool positions are the same.
- **Order:** the transform runs before IK, and again for each yaw option, because the wrist position
  depends on yaw when the gripper is off-axis.

### Slide height

Tool poses use height above the table. Conversion to a slide position uses the backend's slide
direction, a measured reference (the slide value at which the gripper tip touches the table), and the
gripper's dz. The result is then checked against the slide range.

### Reach helper

A function that says whether a table point is reachable at any yaw. Used by the startup config check
(every bin and the table corners reachable) and later by planning.

## Config it needs

Link lengths, joint limits, slide range, slide direction and table-contact reference (plan 01); tool
offset, symmetry and any narrower wrist limits (gripper model, plan 04).

## Done when

- [ ] FK(0°, 90°, 0°) gives x = 0.100, y = 0.125, yaw = 90°.
- [ ] IK(0.100, 0.125) gives θ1 = 0°, θ2 = 90° (to 1e-6).
- [ ] FK(IK(p)) ≈ p across a grid of reachable points and yaws.
- [ ] (0.30, 0.00) is refused as out of reach. (0.05, 0.05) is refused because θ2 ≈ 146° > 120°.
- [ ] The yaw = 170°, θ1 + θ2 = −30° case is wrapped into range, not refused.
- [ ] An off-axis tool offset at yaw 0° and 90° moves the wrist target by the expected amount.
- [ ] Results agree with the sim: send three poses via SimRobot and compare where the arm ends up with
      FK of the joints we predicted (by eye in Unity is enough for stage 1).

## Open points

- The table-contact slide reference has to be measured for each backend and gripper.
- Whether to also mirror the sim's z clamp, or simply refuse anything outside the range. Proposed:
  refuse, never clamp silently.
