"""The taught pick calibration: where the jaws really are, fitted from resistors the arm was put over by hand.

The arm aims by its CAD geometry (the rig from workspace.toml, the potentiometers' angles as they are, the gripper
a line straight down from the slide's shaft). That was right over 5 of 6 resistors (2026-11-11,
experiments/2026-11-11_tray_pick_test/), but not everywhere: at wrist angles near -30 the jaws' centre sat
along the body from it. So the correction is taught on the thing a pick needs, not through the camera's view of
the tip: scripts/teach_picks.py lowers the open jaws over each resistor on the plate, the user nudges them until
they straddle it and records the pair:

    the resistor's centre as the camera finds it (scene mm)   against   the potentiometers there, and the wrist.

Three models are fitted to the pairs, each on top of the CAD geometry, and the simplest that holds up when each
pair is left out in turn is kept:

    elbow          the elbow's zero only: the reach too long or short everywhere (1 number; the first 4 pairs,
                   2026-11-11, all wanted the jaws about 1 mm nearer the base)
    shift          the rig's offset only: the arm's base moved against the plate (2)
    shift+elbow    both (3)
    rigid          the offset and the rig's rotation (3)
    rigid+jaws     and the jaws' centre off the shaft's axis, turning with the gripper (5): the jaws' centre in
                   the scene is the axis plus jaw_mm turned by -(base + elbow + WRIST_SCALE · wrist)

A model is only kept if it can be checked by leaving each pair out (enough pairs for that).

Saved to config/pick_calibration.json; sort_resistors.build_arm aims by it when it is there.
"""

import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from robot_control.config import RigSettings
from robot_control.vision.arm_fit import CONFIG_DIR, JointCorrection, arm_xy
from robot_control.vision.guide import WRIST_SCALE
from robot_control.vision.rig import robot_to_scene

PICK_CALIBRATION = CONFIG_DIR / "pick_calibration.json"
MODELS = ("elbow", "shift", "shift+elbow", "rigid", "rigid+jaws")          # fewest numbers first
MIN_PAIRS = {"elbow": 1, "shift": 1, "shift+elbow": 2, "rigid": 2, "rigid+jaws": 3}   # to determine its numbers
WORTH_MM = 0.2              # a model with more numbers is kept only if its left-out error is this much smaller
STEPS = 40
SMALL = 1e-4


@dataclass(frozen=True)
class PickCalibration:
    model: str
    rotation_deg: float
    translation_m: tuple            # as RigSettings: robot = R(rotation) · scene / 1000 + translation
    jaw_mm: tuple                   # the jaws' centre from the shaft's axis, at base + elbow + scale · wrist = 0
    elbow_offset_deg: float = 0.0   # true elbow = potentiometer elbow + this (as arm_fit.JointCorrection)
    rms_mm: float = float("nan")
    left_out_rms_mm: float = float("nan")
    pairs: str = ""
    n: int = 0
    date: str = ""

    @property
    def rig(self):
        return RigSettings(self.rotation_deg, tuple(self.translation_m))

    @property
    def correction(self):
        return JointCorrection(1.0, self.elbow_offset_deg)

    def jaw_offset(self, base, elbow, wrist):
        """Scene mm from the shaft's axis to the jaws' centre at this pose (guide.plan_jaws's jaw_offset)."""
        return jaw_offset(self.jaw_mm, base, elbow, wrist)

    def describe(self):
        return (f"aim: taught {self.date} on {self.n} resistors ({self.model}), {self.rms_mm:.1f} mm rms, "
                f"{self.left_out_rms_mm:.1f} left out")

    def save(self, path=PICK_CALIBRATION):
        d = asdict(self)
        d["translation_m"], d["jaw_mm"] = list(self.translation_m), list(self.jaw_mm)
        Path(path).write_text(json.dumps(d, indent=2) + "\n")


def load(path=PICK_CALIBRATION):
    """The saved PickCalibration, or None if there isn't one."""
    if not Path(path).exists():
        return None
    d = json.loads(Path(path).read_text())
    d["translation_m"], d["jaw_mm"] = tuple(d["translation_m"]), tuple(d["jaw_mm"])
    return PickCalibration(**d)


def jaw_offset(jaw_mm, base, elbow, wrist):
    a = math.radians(-(base + elbow + WRIST_SCALE * wrist))
    c, s = math.cos(a), math.sin(a)
    return np.array([c * jaw_mm[0] - s * jaw_mm[1], s * jaw_mm[0] + c * jaw_mm[1]])


def _unpack(model, p, cad):
    """(rotation deg, translation mm, jaw mm, elbow offset deg) from a model's numbers."""
    t = tuple(1000.0 * v for v in cad.translation_m)
    return {"elbow": lambda: (cad.rotation_deg, t, (0.0, 0.0), p[0]),
            "shift": lambda: (cad.rotation_deg, (p[0], p[1]), (0.0, 0.0), 0.0),
            "shift+elbow": lambda: (cad.rotation_deg, (p[0], p[1]), (0.0, 0.0), p[2]),
            "rigid": lambda: (p[0], (p[1], p[2]), (0.0, 0.0), 0.0),
            "rigid+jaws": lambda: (p[0], (p[1], p[2]), (p[3], p[4]), 0.0)}[model]()


def _start(model, cad):
    t = [1000.0 * v for v in cad.translation_m]
    return np.array({"elbow": [0.0], "shift": t, "shift+elbow": [*t, 0.0], "rigid": [cad.rotation_deg, *t],
                     "rigid+jaws": [cad.rotation_deg, *t, 0.0, 0.0]}[model])


def predict(rotation_deg, translation_mm, jaw_mm, joints, wrists, kinematics, elbow_offset_deg=0.0):
    """Scene mm (N × 2) of the jaws' centre at each pose (potentiometer joints N × 2, wrists N)."""
    rig = RigSettings(rotation_deg, tuple(v / 1000.0 for v in translation_mm))
    axes = robot_to_scene(arm_xy(joints, kinematics, JointCorrection(1.0, elbow_offset_deg)), rig)
    return np.array([a + jaw_offset(jaw_mm, b, e, w) for a, (b, e), w in zip(axes, joints, wrists)])


def _fit_params(model, centres, joints, wrists, kinematics, cad):
    def residuals(p):
        rot, t, jaw, elbow = _unpack(model, p, cad)
        return (predict(rot, t, jaw, joints, wrists, kinematics, elbow) - centres).ravel()

    p = _start(model, cad).astype(float)
    damping = 1e-3
    for _ in range(STEPS):
        r = residuals(p)
        jac = np.column_stack([(residuals(p + SMALL * np.eye(len(p))[i]) - r) / SMALL for i in range(len(p))])
        step = np.linalg.solve(jac.T @ jac + damping * np.eye(len(p)), -jac.T @ r)
        p = p + step
        if np.abs(step).max() < 1e-7:
            break
    return p, residuals(p).reshape(-1, 2)


def fit(model, centres, joints, wrists, kinematics, cad=None):
    """(PickCalibration, errors N × 2 mm) of one model to the pairs."""
    cad = cad or RigSettings.from_config()
    centres, joints, wrists = (np.asarray(v, float) for v in (centres, joints, wrists))
    p, errors = _fit_params(model, centres, joints.reshape(-1, 2), wrists, kinematics, cad)
    rot, t, jaw, elbow = _unpack(model, p, cad)
    rms = float(np.sqrt((errors ** 2).sum(1).mean()))
    cal = PickCalibration(model, float(rot), tuple(float(v) / 1000.0 for v in t), tuple(float(v) for v in jaw),
                          float(elbow), rms_mm=rms, n=len(centres), date=time.strftime("%Y-%m-%d %H:%M"))
    return cal, errors


def left_out_rms(model, centres, joints, wrists, kinematics, cad=None):
    """Each pair's error when the model is fitted without it, rms mm (nan if too few pairs)."""
    n = len(centres)
    if n - 1 < MIN_PAIRS[model] + 1:          # fitted without one pair, it must still be more than determined
        return float("nan")
    out = []
    for i in range(n):
        keep = np.arange(n) != i
        cal, _ = fit(model, np.asarray(centres)[keep], np.asarray(joints)[keep], np.asarray(wrists)[keep], kinematics, cad)
        guess = predict(cal.rotation_deg, [1000 * v for v in cal.translation_m], cal.jaw_mm,
                        np.asarray(joints)[i:i + 1], np.asarray(wrists)[i:i + 1], kinematics, cal.elbow_offset_deg)[0]
        out.append(float(np.hypot(*(guess - np.asarray(centres)[i]))))
    return float(np.sqrt(np.mean(np.square(out))))


def cad_errors(centres, joints, wrists, kinematics, cad=None):
    """Where the CAD geometry puts the jaws' centre, less where the resistor was, N × 2 mm."""
    cad = cad or RigSettings.from_config()
    return predict(cad.rotation_deg, [1000 * v for v in cad.translation_m], (0.0, 0.0), np.asarray(joints, float),
                   np.asarray(wrists, float), kinematics) - np.asarray(centres, float)


def choose(centres, joints, wrists, kinematics, pairs_name="", cad=None, say=print):
    """Fits every model the pairs allow, says how each did, and returns the PickCalibration to keep: the one with
    the smallest left-out error, a simpler one unless the bigger beats it by WORTH_MM. A model with too few pairs
    to be checked by leaving one out is never kept."""
    cad_e = cad_errors(centres, joints, wrists, kinematics, cad)
    say(f"the CAD geometry: {np.sqrt((cad_e ** 2).sum(1).mean()):.2f} mm rms over {len(centres)} pairs; each: "
        + " ".join(f"{d:.1f}" for d in np.hypot(*cad_e.T)))
    best = None
    for model in MODELS:
        if len(centres) < MIN_PAIRS[model]:
            continue
        cal, errors = fit(model, centres, joints, wrists, kinematics, cad)
        cal = PickCalibration(**{**asdict(cal), "left_out_rms_mm": left_out_rms(model, centres, joints, wrists,
                                                                                 kinematics, cad), "pairs": pairs_name})
        say(f"{model:10s}: {cal.rms_mm:.2f} mm rms, {cal.left_out_rms_mm:.2f} left out; rotation {cal.rotation_deg:.2f} deg, "
            f"offset ({1000 * cal.translation_m[0]:+.2f}, {1000 * cal.translation_m[1]:+.2f}) mm, "
            f"jaws ({cal.jaw_mm[0]:+.2f}, {cal.jaw_mm[1]:+.2f}) mm, elbow {cal.elbow_offset_deg:+.2f} deg")
        score = cal.left_out_rms_mm
        if math.isnan(score):
            continue
        if best is None or score < best[1] - WORTH_MM:
            best = (cal, score)
    return best[0] if best else None
