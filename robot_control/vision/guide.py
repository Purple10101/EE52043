"""One step of guiding the tip onto a target with the camera, pure maths.

The camera says how far the tip is from the target; the arm moves by a fraction of that, however
wrong its own idea of where it is, so the error shrinks whatever the calibration so far.
"""

import math

import numpy as np

from robot_control.vision.arm_fit import JointCorrection
from robot_control.vision.rig import scene_to_robot

GAIN = 0.7                  # of the error, each step: the arm overshoots with more
MAX_STEP_MM = 5.0
# The tip is off the wrist's axis (a circle fitted to it as the wrist turned, 2026-11-05): TIP_OFFSET_MM
# from the axis, in scene direction TIP_ANGLE_DEG (from +x, anticlockwise) at wrist WRIST_REF_DEG with
# base + elbow at TOOL_REF_DEG. It turns clockwise with base, elbow and wrist, the wrist by WRIST_SCALE.
TIP_OFFSET_MM = 6.8
TIP_ANGLE_DEG = 187.6
WRIST_REF_DEG = 70.0
TOOL_REF_DEG = 64.8
WRIST_SCALE = 1.14          # by eye against the crosshair, 2026-11-06: 71 deg turned about 80, then 80 turned about 92
# The jaws close along JAW_ANGLE_DEG at wrist JAW_WRIST_DEG with base + elbow at JAW_TOOL_DEG: the
# gripper's shoulder square to the crosshair's y groove, base -49.7, elbow 112.1, wrist 78.2.
JAW_ANGLE_DEG = 90.0
JAW_WRIST_DEG = 78.2
JAW_TOOL_DEG = 62.4
SOLVE_STEPS = 6


def tip_angle(base, elbow, wrist):
    """Scene direction (deg) of the tip from the wrist's axis at this pose."""
    return TIP_ANGLE_DEG - WRIST_SCALE * (wrist - WRIST_REF_DEG) - (base + elbow - TOOL_REF_DEG)


def tip_offset_mm(base, elbow, wrist):
    """Scene mm (x, y) from the wrist's axis to the tip at this pose."""
    a = math.radians(tip_angle(base, elbow, wrist))
    return np.array([TIP_OFFSET_MM * math.cos(a), TIP_OFFSET_MM * math.sin(a)])


def pots_for(axis, kinematics, rig, correction):
    """(base, elbow) potentiometer angles, deg, that put the wrist's axis at scene mm axis: the true
    angles from the inverse kinematics, undoing correction (true from potentiometer). The firmware
    steers by the potentiometers, so these are the angles to send."""
    x, y = scene_to_robot([axis], rig)[0]
    result = kinematics.ik(x, y, 0.0, kinematics.arm.z[0])
    if not result.ok:
        raise ValueError(result.reason)
    return result.joints.j0 / correction.base_scale, result.joints.j1 - correction.elbow_offset_deg


def plan_point(tip_mm, lens_mm, kinematics, rig, correction=JointCorrection()):
    """(base, elbow, wrist) deg that put the tip at scene mm tip_mm with the jaws face-on to the camera:
    the tip then lies from the axis toward the camera's foot. Raises ValueError if out of reach."""
    tip = np.asarray(tip_mm, float)
    toward = np.asarray(lens_mm[:2], float) - tip
    toward /= np.linalg.norm(toward)
    base, elbow = pots_for(tip - TIP_OFFSET_MM * toward, kinematics, rig, correction)
    wanted = math.degrees(math.atan2(toward[1], toward[0]))
    turn = (tip_angle(base, elbow, WRIST_REF_DEG) - wanted + 180.0) % 360.0 - 180.0
    return base, elbow, WRIST_REF_DEG + turn / WRIST_SCALE


def jaw_angle(base, elbow, wrist):
    """Scene direction (deg, from +x anticlockwise) the jaws close along at this pose. They turn with the
    tip; measured against the crosshair's grooves, 2026-11-05."""
    return JAW_ANGLE_DEG - WRIST_SCALE * (wrist - JAW_WRIST_DEG) - (base + elbow - JAW_TOOL_DEG)


def plan_jaws(tip_mm, body_deg, wrist_range, kinematics, rig, start_wrist=WRIST_REF_DEG, correction=JointCorrection(),
              at_axis=False):
    """(base, elbow, wrist) deg that keep the tip at scene mm tip_mm with the jaws closing across a body
    lying along body_deg. Turning the wrist swings the tip, so the arm moves to cancel it; the jaws are
    the same turned 180 deg, so of the wrist angles in wrist_range the nearest start_wrist is used.
    at_axis: the point is for the jaws' centre, the axis they turn about, not the tip, so nothing
    swings. Raises ValueError if there is none."""
    tip = np.asarray(tip_mm, float)

    def solve(wrist):
        base = elbow = None
        for _ in range(SOLVE_STEPS):
            offset = tip_offset_mm(base, elbow, wrist) if base is not None and not at_axis else np.zeros(2)
            base, elbow = pots_for(tip - offset, kinematics, rig, correction)
            wrist += ((jaw_angle(base, elbow, wrist) - body_deg - 90.0 + 90.0) % 180.0 - 90.0) / WRIST_SCALE
        return base, elbow, wrist

    half_turn = 180.0 / WRIST_SCALE
    found = [solve(start_wrist + k * half_turn) for k in (-1, 0, 1)]
    found = [f for f in found if wrist_range[0] <= f[2] <= wrist_range[1]]
    if not found:
        raise ValueError(f"no wrist angle in {wrist_range[0]:g}..{wrist_range[1]:g} lines the jaws up")
    return min(found, key=lambda f: abs(f[2] - start_wrist))


def axis_mm(tip_mm, base, elbow, wrist):
    """The wrist's axis (scene mm), from the tip seen at scene mm tip_mm at this pose."""
    return np.asarray(tip_mm, float) - tip_offset_mm(base, elbow, wrist)


def step_joints(tip_mm, target_mm, base, elbow, kinematics, rig, gain=GAIN, max_step_mm=MAX_STEP_MM):
    """(base, elbow) deg that bring the tip, seen at scene mm tip_mm, toward target_mm from the pose
    (base, elbow). Raises ValueError if the arm can't go there."""
    a, b = scene_to_robot([tip_mm, target_mm], rig)
    step = gain * (b - a)
    size = 1000.0 * np.linalg.norm(step)
    if size > max_step_mm:
        step *= max_step_mm / size
    x, y, tool = kinematics.fk(base, elbow)
    result = kinematics.ik(x + step[0], y + step[1], tool, kinematics.arm.z[0])
    if not result.ok:
        raise ValueError(result.reason)
    return result.joints.j0, result.joints.j1
