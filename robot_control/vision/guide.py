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
# The tip is off the wrist's axis: TIP_OFFSET_MM from the axis, in scene direction TIP_ANGLE_DEG (from +x,
# anticlockwise) at wrist WRIST_REF_DEG with base + elbow at TOOL_REF_DEG. It turns clockwise with base, elbow
# and wrist, the wrist by WRIST_SCALE. The offset is the gripper v4 CAD's (Robot Resources/gripper/v4/CAD,
# confirmed by the user 2026-11-12): the closed fingertips are 12 mm wide and centred on the mount's axis, and
# the V the camera finds is their end nearest the camera, 6.0 mm out. A circle fitted to the tip as the wrist
# turned (2026-11-05) gave 6.8, through a guessed tip height. The direction is that circle's.
# Gripper v5 (2026-11-18): the same 6.0 mm by its CAD (points 12 mm deep, centred on the axis; the tip the camera
# finds, jaws.py, is the end nearest it). The direction from the wrist calibration below: at wrist -15, base +
# elbow 66.94, the jaws close along y and the spring-slot face is square to the camera, so the tip is toward -x.
# v4's were 187.6 at wrist 70, tool 64.8.
TIP_OFFSET_MM = 6.0
TIP_ANGLE_DEG = 180.0
WRIST_REF_DEG = -15.0
TOOL_REF_DEG = 66.94
WRIST_SCALE = 1.0           # gripper v5, by the user's eye against the crosshair, 2026-11-18: 90 firmware deg (-15 to 75)
                            # turned the jaws from the y groove to the x groove, 79 was short. 1.14 with v4 (2026-11-06)
# The jaws close along JAW_ANGLE_DEG at wrist JAW_WRIST_DEG with base + elbow at JAW_TOOL_DEG. Gripper v5,
# 2026-11-18, slide down at 59, by the user's eye: jaws along the crosshair's y groove (and face-on to the
# camera) at base -43.65, elbow 110.59, wrist -15, reached going up from -60. v4: wrist 78.2 at tool 62.4.
JAW_ANGLE_DEG = 90.0
# While the camera measures the gripper (camera_calibrate.py, the aim check) the jaws close along this scene
# direction: square to the plate, along its y edges (the user's choice, 2026-11-18).
CALIBRATION_JAW_DEG = 90.0
JAW_WRIST_DEG = -15.0
JAW_TOOL_DEG = 66.94
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


def plan_square(tip_mm, jaw_deg, lens_mm, kinematics, rig, correction=JointCorrection()):
    """(base, elbow, wrist) deg that put the tip at scene mm tip_mm with the jaws closing along scene
    direction jaw_deg (90: square to the plate's x edges), the face the camera finds them by (the tip's side)
    on the camera's side of the jaw line. Raises ValueError if out of reach."""
    tip = np.asarray(tip_mm, float)
    toward = np.asarray(lens_mm[:2], float) - tip
    wanted = jaw_deg + 90.0         # the tip lies square to the jaw line: of the two ways, the camera's
    if math.cos(math.radians(wanted)) * toward[0] + math.sin(math.radians(wanted)) * toward[1] < 0:
        wanted -= 180.0
    a = math.radians(wanted)
    base, elbow = pots_for(tip - TIP_OFFSET_MM * np.array([math.cos(a), math.sin(a)]), kinematics, rig, correction)
    turn = (tip_angle(base, elbow, WRIST_REF_DEG) - wanted + 180.0) % 360.0 - 180.0
    return base, elbow, WRIST_REF_DEG + turn / WRIST_SCALE


def jaw_angle(base, elbow, wrist):
    """Scene direction (deg, from +x anticlockwise) the jaws close along at this pose. They turn with the
    tip; measured against the crosshair's grooves, 2026-11-05."""
    return JAW_ANGLE_DEG - WRIST_SCALE * (wrist - JAW_WRIST_DEG) - (base + elbow - JAW_TOOL_DEG)


def plan_jaws(tip_mm, body_deg, wrist_range, kinematics, rig, start_wrist=WRIST_REF_DEG, correction=JointCorrection(),
              at_axis=False, jaw_offset=None):
    """(base, elbow, wrist) deg that keep the tip at scene mm tip_mm with the jaws closing across a body
    lying along body_deg. Turning the wrist swings the tip, so the arm moves to cancel it; the jaws are
    the same turned 180 deg, so of the wrist angles in wrist_range the nearest start_wrist is used.
    at_axis: the point is for the jaws' centre, the axis they turn about, not the tip, so nothing
    swings. jaw_offset: (base, elbow, wrist) -> scene mm from the shaft's axis to the jaws' centre (a taught
    pick calibration, vision/pick_fit.py); when given, the jaws' centre is put on the point by it instead.
    Raises ValueError if there is none."""
    tip = np.asarray(tip_mm, float)

    def offset_at(base, elbow, wrist):
        if base is None:
            return np.zeros(2)
        if jaw_offset is not None:
            return np.asarray(jaw_offset(base, elbow, wrist), float)
        return np.zeros(2) if at_axis else tip_offset_mm(base, elbow, wrist)

    def solve(wrist):
        base = elbow = None
        for _ in range(SOLVE_STEPS):
            offset = offset_at(base, elbow, wrist)
            base, elbow = pots_for(tip - offset, kinematics, rig, correction)
            wrist += ((jaw_angle(base, elbow, wrist) - body_deg - 90.0 + 90.0) % 180.0 - 90.0) / WRIST_SCALE
        return base, elbow, wrist

    half_turn = 180.0 / WRIST_SCALE
    found = [solve(start_wrist + k * half_turn) for k in (-1, 0, 1)]
    found = [f for f in found if wrist_range[0] <= f[2] <= wrist_range[1]]
    if not found:
        raise ValueError(f"no wrist angle in {wrist_range[0]:g}..{wrist_range[1]:g} lines the jaws up")
    return min(found, key=lambda f: abs(f[2] - start_wrist))


def wrist_across(base, elbow, body_deg, wrist_range, start_wrist=WRIST_REF_DEG):
    """The wrist angle (deg) that closes the jaws across a body lying along body_deg, with the arm at
    (base, elbow): of those in wrist_range, the nearest start_wrist. Raises ValueError if there is none."""
    wrist = start_wrist + ((jaw_angle(base, elbow, start_wrist) - body_deg - 90.0 + 90.0) % 180.0 - 90.0) / WRIST_SCALE
    found = [wrist + k * 180.0 / WRIST_SCALE for k in (-1, 0, 1)]
    found = [w for w in found if wrist_range[0] <= w <= wrist_range[1]]
    if not found:
        raise ValueError(f"no wrist angle in {wrist_range[0]:g}..{wrist_range[1]:g} lines the jaws up")
    return min(found, key=lambda w: abs(w - start_wrist))


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
