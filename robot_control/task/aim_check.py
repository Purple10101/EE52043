"""AimCheck: does the arm still go where the camera says? Measured, and the steady part taken out.

The arm aims from the camera's picture through the arm-to-plate calibration (camera_calibration.json:
scene mm to the robot frame, and the joints' corrections). Carrying the rig shifts the arm's base against
the plate and camera, which travel together on the tower, so the calibration then aims everything off by
the same few mm. On 2026-11-11 that made a pick miss: experiments/2026-11-11_pick_error_pipeline/
(2.8 mm, nearly all one steady shift; 1.1 mm rms left once the shift was taken out).

The check, a minute or two:
1. The jaws closed, the tip put down at each of CHECK_POINTS in turn, face-on to the camera, at the pick
   height (Arm.touch: the five points camera_calibrate's check uses, which its fit never saw).
2. At each, the camera finds the tip in an averaged picture (Eyes.find_tip) and the potentiometers are read.
   A point where the tip can't be found is skipped.
3. Each pair gives the error: where the calibration says the wrist axis is, against where the camera saw
   it. Their mean is the shift; what is left after taking it out is what a shift can't fix.
4. If the shift is worth taking out (SHIFT_WORTH_MM), the calibration's translation is moved by it
   (arm_fit.fit_shift) and saved, the old file kept in logs/. If what is left is more than LEFT_OK_MM, the
   error isn't just a shift: the full calibration is needed (README, "Calibrating the system").

Every pair goes to logs/aim_check_<time>.csv in camera_calibrate's columns, so `camera_calibrate.py fit`
can use them too.
"""

import csv
import json
import math
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from robot_control.motion.arm import ArmError
from robot_control.vision.arm_fit import CALIBRATION, errors_mm, fit_shift, save_calibration
from robot_control.vision.guide import axis_mm

ROOT = Path(__file__).resolve().parents[2]
CHECK_POINTS = [(20.0, -150.0), (-20.0, -150.0), (20.0, -122.0), (-20.0, -122.0), (0.0, -142.0)]   # camera_calibrate's
PICK_HEIGHT_MM = 1.0        # the tip above the plate at the pick height (camera_calibrate, by eye)
MIN_POINTS = 3              # fewer tips found than this: no verdict
SHIFT_WORTH_MM = 0.5        # a shift smaller than this is left alone
LEFT_OK_MM = 2.0            # rms left after the shift: more than this and the calibration needs redoing
FIELDS = ["target_x", "target_y", "tip_x", "tip_y", "base", "elbow", "wrist", "slide", "height"]


@dataclass
class AimReport:
    """What a check found."""
    points: list                # the targets visited, scene mm
    tips: list                  # where the camera saw the tip at each (None where it couldn't)
    errors_mm: np.ndarray       # N × 2, scene mm: where the camera saw the wrist axis less where the calibration
                                # put it, i.e. how far off the arm landed, for the points with a tip
    shift_mm: np.ndarray        # their mean, scene mm
    rms_before_mm: float
    rms_after_mm: float         # once the shift is taken out
    saved: bool = False
    verdict: str = ""

    @property
    def found(self):
        return sum(t is not None for t in self.tips)


class AimCheck:
    def __init__(self, eyes, arm, path=CALIBRATION, logs=ROOT / "logs"):
        """eyes: vision.Eyes. arm: motion.Arm (its navigator's rig and correction are what picks aim by)."""
        self.eyes, self.arm, self.path, self.logs = eyes, arm, Path(path), Path(logs)

    def run(self, say=print):
        """Visits the points, measures, and takes the shift out if it's worth it. Returns the AimReport."""
        from robot_control.vision.tip import TipNotFound
        nav = self.arm.navigator
        lens = self.eyes.lens_mm()
        pairs_path = self.logs / f"aim_check_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        tips, axes, joints = [], [], []
        with pairs_path.open("w", newline="") as f:
            out = csv.writer(f)
            out.writerow(FIELDS)
            for i, point in enumerate(CHECK_POINTS, 1):
                say(f"aim check {i} of {len(CHECK_POINTS)}: the tip down at ({point[0]:g}, {point[1]:g})")
                base, elbow, wrist = self.arm.touch(point, lens)
                try:
                    tip = self.eyes.find_tip(point, PICK_HEIGHT_MM)
                except TipNotFound as why:
                    say(f"aim check {i}: tip not found ({why}), skipped")
                    tips.append(None)
                    continue
                finally:
                    self.arm.lift()
                tips.append(tuple(float(v) for v in tip))
                axes.append(axis_mm(tip, base, elbow, wrist))
                joints.append((base, elbow))
                out.writerow([point[0], point[1], round(tip[0], 2), round(tip[1], 2), round(base, 2), round(elbow, 2),
                              round(wrist, 2), self.arm.bench.OVER_PLATE_SLIDE, PICK_HEIGHT_MM])
                f.flush()
        self.arm.park()
        return self.judge(CHECK_POINTS, tips, np.array(axes), np.array(joints), nav, pairs_path, say)

    def judge(self, points, tips, axes, joints, nav, pairs_path, say=print):
        """The verdict from the pairs; saves the shifted calibration and gives it to the navigator if it's worth it."""
        if len(axes) < MIN_POINTS:
            report = AimReport(points, tips, np.zeros((0, 2)), np.zeros(2), float("nan"), float("nan"),
                               verdict=f"only {len(axes)} of {len(points)} tips found: no verdict. Is the gripper "
                                       "face-on to the camera with its jaws closed, the plate clear?")
            say(report.verdict)
            return report
        before = errors_mm(axes, joints, nav.kinematics, nav.rig, nav.correction)
        shifted = fit_shift(axes, joints, nav.kinematics, nav.rig, nav.correction)
        rms = lambda e: float(np.sqrt((np.asarray(e) ** 2).sum(1).mean()))
        scene = to_scene(before, nav.rig)
        report = AimReport(points, tips, scene, scene.mean(0), rms(before), rms(shifted.errors_mm))
        shift = float(np.hypot(*report.shift_mm))
        direction = f"(dx {report.shift_mm[0]:+.1f}, dy {report.shift_mm[1]:+.1f} on the plate)"
        if shift < SHIFT_WORTH_MM:
            report.verdict = f"aim good: off {report.rms_before_mm:.1f} mm rms, no steady shift worth taking out"
        else:
            self.save(shifted, report, pairs_path)
            nav.rig = shifted.rig           # every pick from now on aims by it
            report.saved = True
            report.verdict = (f"aim was off {report.rms_before_mm:.1f} mm rms, a steady {shift:.1f} mm {direction}; "
                              f"taken out and saved, {report.rms_after_mm:.1f} mm left")
        if report.rms_after_mm > LEFT_OK_MM:
            report.verdict += (f". {report.rms_after_mm:.1f} mm is more than a shift can fix: redo the full "
                               "calibration (README, Calibrating the system)")
        say(report.verdict)
        return report

    def save(self, fit, report, pairs_path):
        """The shifted calibration, written where every tool reads it; the old one kept in logs/."""
        old = json.loads(self.path.read_text()) if self.path.exists() else {}
        if self.path.exists():
            shutil.copy(self.path, self.logs / f"camera_calibration_replaced_{time.strftime('%Y%m%d_%H%M%S')}.json")
        save_calibration(fit, {
            "date": time.strftime("%Y-%m-%d"),
            "pairs": old.get("pairs", ""),
            "rms_mm": old.get("rms_mm"), "worst_mm": old.get("worst_mm"), "left_out_rms_mm": old.get("left_out_rms_mm"),
            "shifted": {"by_mm": [round(float(v), 2) for v in report.shift_mm], "on": time.strftime("%Y-%m-%d %H:%M"),
                        "pairs": pairs_path.name, "rms_before_mm": round(report.rms_before_mm, 2),
                        "rms_after_mm": round(report.rms_after_mm, 2), "fitted_on": old.get("date")},
        }, self.path)


def to_scene(errors_robot_mm, rig):
    """Error vectors (N × 2 mm) from the robot frame to the scene's, as robot_to_scene turns points."""
    a = math.radians(rig.rotation_deg)
    c, s = math.cos(a), math.sin(a)
    return np.asarray(errors_robot_mm, float).reshape(-1, 2) @ np.array([[c, -s], [s, c]])


CAD_LINE = "aim: the arm's CAD geometry (no camera calibration)"


def camera_aimed(arm):
    """Whether the arm aims by the camera calibration (sort_resistors.build_arm's camera_calibration) rather than
    its CAD geometry. Only then is there an aim for AimCheck to check."""
    return getattr(getattr(arm, "navigator", None), "geometry", "camera") == "camera"


def describe_for(arm):
    """describe(), the taught pick calibration's line, or CAD_LINE: whichever the arm aims by."""
    if camera_aimed(arm):
        return describe()
    if getattr(arm.navigator, "geometry", "") == "taught":
        from robot_control.vision import pick_fit
        taught = pick_fit.load()
        return taught.describe() if taught else CAD_LINE
    return CAD_LINE


def describe(path=CALIBRATION):
    """One line on the arm-to-plate calibration in use: when it was fitted, and when its aim was last checked."""
    if not Path(path).exists():
        return "aim: NO CALIBRATION: do the full one (README, Calibrating the system)"
    d = json.loads(Path(path).read_text())
    shifted = d.get("shifted")
    if shifted:
        return (f"aim: checked {shifted['on']}, off {shifted['rms_before_mm']} mm, now {shifted['rms_after_mm']} "
                f"(fitted {shifted.get('fitted_on') or d.get('date')})")
    return f"aim: fitted {d.get('date')}, not checked since. Moved the rig? CHECK AIM"
