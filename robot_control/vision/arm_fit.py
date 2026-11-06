"""Fits where the camera saw the tip to where the potentiometers say the arm is, pure maths.

Each pair is the tip in scene mm (from the camera) and the base and elbow angles (deg) at that moment.
fit_rigid gives the [scene_to_robot] that suits them best. fit_joints also corrects the angles, for
the residuals a rigid fit leaves.

No base offset is fitted: turning the base by a constant turns the arm's whole reach about the base
axis, which the rotation and translation already cover, so the two can't be told apart.
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from robot_control.config import CONFIG_DIR, RigSettings
from robot_control.vision.rig import scene_to_robot

CALIBRATION = CONFIG_DIR / "camera_calibration.json"
STEPS = 30                  # Gauss-Newton steps at most: it settles in a few
SMALL_STEP = 1e-6           # for the numerical Jacobian
DONE = 1e-9                 # a step this small ends the fit


@dataclass(frozen=True)
class JointCorrection:
    """True angles from potentiometer ones: base · base_scale, elbow + elbow_offset_deg."""

    base_scale: float = 1.0
    elbow_offset_deg: float = 0.0

    def apply(self, joints):
        j = np.asarray(joints, float).reshape(-1, 2)
        return np.column_stack([j[:, 0] * self.base_scale, j[:, 1] + self.elbow_offset_deg])


@dataclass(frozen=True)
class ArmFit:
    rig: RigSettings
    joints: JointCorrection
    errors_mm: np.ndarray   # N × 2, robot frame: where the camera puts each tip less where the arm does

    @property
    def distances_mm(self):
        return np.hypot(*self.errors_mm.T)

    @property
    def rms_mm(self):
        return float(np.sqrt(np.mean(self.distances_mm ** 2)))

    @property
    def worst_mm(self):
        return float(self.distances_mm.max())


def save_calibration(fit, note, path=CALIBRATION):
    """Writes an ArmFit, with note (a dict: its date and how good it is) for the record."""
    Path(path).write_text(json.dumps({
        "rotation_deg": fit.rig.rotation_deg, "translation_m": list(fit.rig.translation_m),
        "base_scale": fit.joints.base_scale, "elbow_offset_deg": fit.joints.elbow_offset_deg, **note}, indent=2) + "\n")


def load_calibration(path=CALIBRATION):
    """(RigSettings, JointCorrection) as saved by save_calibration."""
    d = json.loads(Path(path).read_text())
    return (RigSettings(float(d["rotation_deg"]), tuple(float(v) for v in d["translation_m"])),
            JointCorrection(float(d["base_scale"]), float(d["elbow_offset_deg"])))


def fit_circle(points):
    """(centre (x, y), radius, residuals) of the circle through points (N × 2), by least squares.
    The tip's path as the wrist turns: the centre is the wrist's axis, the radius the tip's offset."""
    p = np.asarray(points, float).reshape(-1, 2)
    if len(p) < 3:
        raise ValueError("a circle needs at least 3 points")
    mean = p.mean(0)
    q = p - mean
    a, b, c = np.linalg.lstsq(np.column_stack([2 * q, np.ones(len(q))]), (q ** 2).sum(1), rcond=None)[0]
    radius = math.sqrt(c + a * a + b * b)
    return mean + (a, b), radius, np.hypot(*(q - (a, b)).T) - radius


def arm_xy(joints, kinematics, correction=JointCorrection()):
    """Robot m (N × 2) for potentiometer angles (N × 2, base and elbow, deg)."""
    return np.array([kinematics.fk(b, e)[:2] for b, e in correction.apply(joints)])


def errors_mm(scene_mm, joints, kinematics, rig, correction=JointCorrection()):
    """N × 2: each tip as the camera saw it, less where the arm says it is, in the robot frame."""
    return 1000.0 * (scene_to_robot(scene_mm, rig) - arm_xy(joints, kinematics, correction))


def _rigid(scene_mm, robot_m):
    """The RigSettings taking scene mm to robot m with the least squared error."""
    a = np.asarray(scene_mm, float).reshape(-1, 2) / 1000.0
    b = np.asarray(robot_m, float).reshape(-1, 2)
    p, q = a - a.mean(0), b - b.mean(0)
    angle = math.atan2(np.sum(p[:, 0] * q[:, 1] - p[:, 1] * q[:, 0]), np.sum(p * q))
    c, s = math.cos(angle), math.sin(angle)
    t = b.mean(0) - np.array([[c, -s], [s, c]]) @ a.mean(0)
    return RigSettings(math.degrees(angle), (float(t[0]), float(t[1])))


def fit_rigid(scene_mm, joints, kinematics):
    """Rotation and translation only: an ArmFit that drops straight into [scene_to_robot]."""
    if len(scene_mm) < 2:
        raise ValueError("a rigid fit needs at least 2 pairs")
    rig = _rigid(scene_mm, arm_xy(joints, kinematics))
    return ArmFit(rig, JointCorrection(), errors_mm(scene_mm, joints, kinematics, rig))


def fit_joints(scene_mm, joints, kinematics):
    """Rotation, translation, the base's scale and the elbow's offset, together."""
    if len(scene_mm) < 3:
        raise ValueError("fitting the joints needs at least 3 pairs")

    def unpack(p):
        correction = JointCorrection(1.0 + p[0], p[1])
        return _rigid(scene_mm, arm_xy(joints, kinematics, correction)), correction

    def residuals(p):
        rig, correction = unpack(p)
        return errors_mm(scene_mm, joints, kinematics, rig, correction).ravel()

    p = np.zeros(2)
    for _ in range(STEPS):
        r = residuals(p)
        jacobian = np.column_stack([(residuals(p + SMALL_STEP * np.eye(2)[i]) - r) / SMALL_STEP for i in range(2)])
        step = np.linalg.lstsq(jacobian, -r, rcond=None)[0]
        p = p + step
        if np.abs(step).max() < DONE:
            break
    rig, correction = unpack(p)
    return ArmFit(rig, correction, errors_mm(scene_mm, joints, kinematics, rig, correction))
