"""Teach the arm where the resistors are: by hand, once, on the resistors lying on the plate. Saved for every run.

    python scripts/teach_picks.py                      # teach on the resistors on the plate, then fit and save
    python scripts/teach_picks.py --fit logs/pick_teach_<time>.csv           # fit again from saved pairs
    python scripts/teach_picks.py --fit logs/pick_teach_<time>.csv --dry     # ... and only print

The arm parks and the camera finds the resistors. Then, one resistor at a time, the open jaws come down over it
the way a pick would (across the body, at the pick's wrist angle) and stop at the bottom without closing. Look
from straight above and nudge them until the body sits in the middle of the jaws, then ENTER. In the window:

    w / s   (or up / down)     move the jaws along the body
    a / d   (or left / right)  move them across it
    [ / ]                      a smaller / bigger nudge (0.25, 0.5, 1, 2 mm)
    ENTER                      the jaws are centred: record it, on to the next
    n                          skip this one
    q / Esc                    done: park, fit and save

Each nudge lifts the jaws to the approach height, moves, and comes down again, so nothing on the plate is touched.
Every pair (the camera's centre of the resistor against the potentiometers and wrist where the jaws were centred)
goes to logs/pick_teach_<time>.csv as it's recorded. At the end vision/pick_fit.py fits the correction on top of
the CAD geometry and saves it to robot_control/config/pick_calibration.json (the old one kept in logs/). The
sorting windows aim by it from then on. Teach again only if the rig is moved or picks start missing.

Before: the arm started and parked (CALIBRATE in the sorting window, then close it), the gripper board plugged in,
resistors spread over the plate (4 or more, near and far, left and right, lying different ways), a hand on the
servo supply's plug.
"""

import argparse
import csv
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav  # noqa: E402
import slide_wrist as bench  # noqa: E402
import sort_resistors  # noqa: E402

from robot_control.config import ArmSettings  # noqa: E402
from robot_control.kinematics.arm import Kinematics  # noqa: E402
from robot_control.vision import pick_fit  # noqa: E402
from robot_control.vision.eyes import Eyes  # noqa: E402
from robot_control.vision.guide import plan_jaws  # noqa: E402

FIELDS = ["centre_x", "centre_y", "body_deg", "aim_x", "aim_y", "along_mm", "across_mm", "base", "elbow", "wrist"]
STEPS_MM = [0.25, 0.5, 1.0, 2.0]
NUDGE_S = 1.0
WINDOW = "teach picks"
KEYS = {ord("w"): (1, 0), 2490368: (1, 0), ord("s"): (-1, 0), 2621440: (-1, 0),
        ord("d"): (0, 1), 2555904: (0, 1), ord("a"): (0, -1), 2424832: (0, -1)}


def say(text):
    print(text, flush=True)


class Teacher:
    """Puts the open jaws over one resistor after another, moves them by the user's nudges, records the pairs."""

    def __init__(self, arm, resistors, writer, out):
        self.arm, self.nav = arm, arm.navigator
        self.resistors = resistors
        self.writer, self.out = writer, out
        self.i = 0
        self.along = self.across = 0.0
        self.step = 1
        self.message = ""
        self.pairs = []
        self.down = False

    @property
    def resistor(self):
        return self.resistors[self.i] if self.i < len(self.resistors) else None

    def axes(self):
        a = np.radians(self.resistor.direction_deg)
        return np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])

    def aim(self):
        u, v = self.axes()
        return np.asarray(self.resistor.centre_mm, float) + self.along * u + self.across * v

    def arrive(self):
        """Over the current resistor as a pick goes, then down to the bottom with the jaws open."""
        self.along = self.across = 0.0
        self.down = False
        r = self.resistor
        say(f"\nresistor {self.i + 1} of {len(self.resistors)}: ({r.centre_mm[0]:.1f}, {r.centre_mm[1]:.1f}), "
            f"long axis {r.direction_deg:.0f} deg")
        try:
            if self.nav.go(r.centre_mm, r.direction_deg) is None:
                self.message = f"can't reach it ({self.nav.status}): n to skip"
                return
            self.nav.drive("slide", resistor_nav.PICK_SLIDE, resistor_nav.SLIDE_MOVE_S)
            self.down = True
            self.message = "down, jaws open: is the body in the middle of them?"
        except SystemExit as why:
            self.message = f"refused: {why}. n to skip"

    def nudge(self, d_along, d_across):
        if not self.down:
            return
        size = STEPS_MM[self.step]
        self.along += d_along * size
        self.across += d_across * size
        aim = self.aim()
        try:
            base, elbow, wrist = plan_jaws(aim, self.resistor.direction_deg, bench.SAFE["wrist"], self.nav.kinematics,
                                           self.nav.rig, self.nav.wrist_centre, self.nav.correction,
                                           at_axis=self.nav.aim_axis, jaw_offset=self.nav.jaw_offset)
            self.nav.drive("slide", resistor_nav.NAV_SLIDE, resistor_nav.SLIDE_MOVE_S)
            bench.glide(self.nav.scara, base, elbow, wrist, resistor_nav.NAV_SLIDE, NUDGE_S)
            self.nav.drive("slide", resistor_nav.PICK_SLIDE, resistor_nav.SLIDE_MOVE_S)
            self.message = f"moved {self.along:+.2f} along, {self.across:+.2f} across (mm)"
        except (SystemExit, ValueError) as why:
            self.along -= d_along * size
            self.across -= d_across * size
            self.message = f"refused: {why}"

    def record(self):
        if not self.down:
            return
        base, elbow = (float(v) for v in bench.read_joints(self.nav.scara))
        wrist = float(bench.load_state()["wrist"])
        r, aim = self.resistor, self.aim()
        row = [r.centre_mm[0], r.centre_mm[1], r.direction_deg, aim[0], aim[1], self.along, self.across, base, elbow, wrist]
        self.writer.writerow([round(float(v), 3) for v in row])
        self.out.flush()
        self.pairs.append(row)
        say(f"  recorded: centred {self.along:+.2f} along, {self.across:+.2f} across; base {base:.2f}, elbow {elbow:.2f}, "
            f"wrist {wrist:.1f}")
        self.next()

    def next(self):
        self.nav.lift()
        self.i += 1
        if self.resistor is not None:
            self.arrive()

    def draw(self):
        image = np.zeros((300, 900, 3), np.uint8)
        r = self.resistor
        lines = [f"{len(self.pairs)} recorded" + (f", resistor {self.i + 1} of {len(self.resistors)}: "
                                                    f"({r.centre_mm[0]:.1f}, {r.centre_mm[1]:.1f}) mm" if r else ": all done, q"),
                 f"nudged {self.along:+.2f} mm along the body, {self.across:+.2f} across; nudge size {STEPS_MM[self.step]} mm",
                 self.message, "",
                 "w/s or up/down: along the body     a/d or left/right: across it     [ ]: nudge size",
                 "ENTER: centred, record     n: skip     q / Esc: done, fit and save"]
        for k, line in enumerate(lines):
            cv2.putText(image, line, (14, 34 + 42 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, image)

    def run(self):
        cv2.namedWindow(WINDOW)
        cv2.moveWindow(WINDOW, 60, 60)
        self.arrive()
        while True:
            self.draw()
            key = cv2.waitKeyEx(50)
            if key in (ord("q"), 27):
                return
            if self.resistor is None:
                continue
            if key in KEYS:
                self.nudge(*KEYS[key])
            elif key == ord("["):
                self.step = max(0, self.step - 1)
            elif key == ord("]"):
                self.step = min(len(STEPS_MM) - 1, self.step + 1)
            elif key in (13, 10):
                self.record()
            elif key == ord("n"):
                say("  skipped")
                self.next()


def fit_and_save(pairs, pairs_name, dry=False):
    p = np.asarray(pairs, float)
    if len(p) < 1:
        say("no pairs: nothing to fit")
        return
    kinematics = Kinematics(ArmSettings.from_config())
    cal = pick_fit.choose(p[:, 0:2], p[:, 7:9], p[:, 9], kinematics, pairs_name, say=say)
    if cal is None:
        say("too few pairs to fit")
        return
    say(f"\nkept: {cal.model}. {cal.describe()}")
    if dry:
        say("--dry: not saved")
        return
    if pick_fit.PICK_CALIBRATION.exists():
        shutil.copy(pick_fit.PICK_CALIBRATION, ROOT / "logs" / f"pick_calibration_replaced_{time.strftime('%Y%m%d_%H%M%S')}.json")
    cal.save()
    say(f"saved {pick_fit.PICK_CALIBRATION.relative_to(ROOT)}: the sorting windows aim by it from their next start")


def read_pairs(path):
    with open(path, newline="") as f:
        return [[float(r[k]) for k in FIELDS] for r in csv.DictReader(f)]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--fit", help="fit from a saved pairs file instead of teaching")
    p.add_argument("--dry", action="store_true", help="fit and print, but don't save")
    args = p.parse_args()
    if args.fit:
        fit_and_save(read_pairs(args.fit), Path(args.fit).name, args.dry)
        return
    if bench.load_state() is None:
        sys.exit("Not started: CALIBRATE in the sorting window first (then close it).")
    arm, _ = sort_resistors.build_arm(None, resistor_nav.load_bins(), camera_calibration=False)
    say(f"aiming by: {arm.navigator.geometry}")
    eyes = Eyes(closeups=False, say=say)
    path = ROOT / "logs" / f"pick_teach_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        arm.park()
        view = eyes.look(sharp=True)
        eyes.close()
        if not view.resistors:
            say("no resistors found on the plate")
            return
        say(f"{len(view.resistors)} resistors found; pairs to {path.relative_to(ROOT)}")
        with path.open("w", newline="") as out:
            writer = csv.writer(out)
            writer.writerow(FIELDS)
            teacher = Teacher(arm, view.resistors, writer, out)
            try:
                teacher.run()
            finally:
                cv2.destroyAllWindows()
                arm.navigator.lift()
                say("\nparking")
                arm.park()
        fit_and_save(teacher.pairs, path.name, args.dry)
    finally:
        try:
            eyes.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
