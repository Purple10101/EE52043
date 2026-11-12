"""Whether the arm, at a pose, hides any of the plate from the camera.

The camera looks down at the plate from the tower at an angle, so a part of the arm hides the plate if the
line from the camera through it meets the plate. Each link is treated as a box (its width, and the heights
its parts span in the digital twin's meshes) and points on it are projected from the camera onto the plate's
top; if any lands on the plate (plus a margin), the arm is in the way.

Why it matters: the run used to park on the far side of the plate before every look, which swung the arm
from the trays all the way across and back each cycle. Worked out for the taught slots (2026-11-11): at
A1-A4 the arm hides none of the plate, slide up or down in the slot; at B1-B4 the upper arm lies across it.
So after a drop into an A slot the camera can look straight away (task.Supervisor.clear_view).
"""

import numpy as np

from robot_control.vision.rig import robot_to_scene

PLATE_TOP_Z = -22.0                         # scene mm
PLATE_MM = ((-50.0, 50.0), (-180.0, -80.0))
MARGIN_MM = 10.0                            # the arm's outline here is approximate
# (width mm, lowest z, highest z) of each link, from the twin's meshes (twin_manifest.json bounds).
LINK1 = (26.0, 35.0, 108.0)                 # the upper arm, base axis to elbow
LINK2 = (18.0, 35.0, 234.0)                 # the forearm, with the slide's column at its end
GRIPPER = (30.0, 70.0)                      # the gripper hanging at the wrist: width, height above its tips
TIPS_Z = (-18.0, 15.0)                      # the tips' height: down in a bin, and with the slide fully up


def joints_mm(base_deg, elbow_deg, kinematics, rig):
    """(base axis, elbow, wrist) in scene mm, by the firmware's forward kinematics."""
    l1, l2 = kinematics.arm.link1, kinematics.arm.link2
    a, b = np.radians(base_deg), np.radians(base_deg + elbow_deg)
    elbow = (l1 * np.sin(a), l1 * np.cos(a))
    wrist = (elbow[0] + l2 * np.sin(b), elbow[1] + l2 * np.cos(b))
    return robot_to_scene([(0.0, 0.0), elbow, wrist], rig)


def lands_on_plate(point, lens_mm, margin_mm=MARGIN_MM):
    """Whether the camera's line through point (scene mm, x y z) meets the plate's top, plus a margin."""
    camera = np.asarray(lens_mm, float)
    if point[2] >= camera[2]:
        return False
    t = (camera[2] - PLATE_TOP_Z) / (camera[2] - point[2])
    x, y = camera[:2] + (np.asarray(point[:2]) - camera[:2]) * t
    (x0, x1), (y0, y1) = PLATE_MM
    return x0 - margin_mm <= x <= x1 + margin_mm and y0 - margin_mm <= y <= y1 + margin_mm


def arm_hides_plate(base_deg, elbow_deg, lens_mm, kinematics, rig, margin_mm=MARGIN_MM):
    """Whether any part of the arm at (base, elbow) hides the plate from a camera at lens_mm, whatever the
    slide's height (the gripper is checked both down in a bin and fully up)."""
    base, elbow, wrist = joints_mm(base_deg, elbow_deg, kinematics, rig)
    boxes = [(base, elbow, LINK1[0], LINK1[1], LINK1[2]), (elbow, wrist, LINK2[0], LINK2[1], LINK2[2])]
    boxes += [(wrist, wrist, GRIPPER[0], tip, tip + GRIPPER[1]) for tip in TIPS_Z]
    for a, c, width, z0, z1 in boxes:
        a, c = np.asarray(a, float), np.asarray(c, float)
        d = c - a
        across = np.array([-d[1], d[0]]) / (np.linalg.norm(d) or 1.0)
        if not np.linalg.norm(d):
            across = np.array([1.0, 0.0])
        for s in np.linspace(0.0, 1.0, 25):
            for w in np.linspace(-width / 2, width / 2, 5):
                x, y = a + s * d + w * across
                for z in np.linspace(z0, z1, 6):
                    if lands_on_plate((x, y, z), lens_mm, margin_mm):
                        return True
        if not np.linalg.norm(d):                   # the gripper: a square, not a line
            for w in np.linspace(-width / 2, width / 2, 5):
                for z in np.linspace(z0, z1, 6):
                    if lands_on_plate((a[0], a[1] + w, z), lens_mm, margin_mm):
                        return True
    return False
