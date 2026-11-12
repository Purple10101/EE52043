"""The sorting program with the procedure fixed: two buttons, always done the same way.

    CALIBRATE   on an EMPTY plate, after the arm has been set by hand: StandardCalibration, below.
    START       on a FULL plate, once CALIBRATE has passed in this window: the sorting run.
    STOP        ends a run once the arm is parked again.
    q / Esc     quit, once nothing is running.

The manual window (task.app.SortingApp, scripts/sort_resistors.py) leaves the order to whoever is at the
rig: whether to start the arm, check the aim, where the plate is found. Here the order is fixed, each
step is checked, and sorting can't begin until all of them have passed:

1. The arm started: the start sequence and the elbow check (Arm.start_up). Every CALIBRATE, so the arm is
   known to be calibrated whatever happened to the Nucleo's power before.
2. The plate found and confirmed empty, with the arm parked out of view (Eyes.count_bodies): the plate is
   found on a clear plate, never with a lead over its crosshair, and the aim check has nothing to hit.
3. The aim checked (task.AimCheck): the tip down at 5 points, the steady shift taken out and saved.
   It passes if no more than LEFT_OK_MM is left; otherwise the full calibration is needed.

Then START reads every resistor's value once (a focus sweep, then a close-up of each: vision.Eyes.read_values)
and sorts until nothing is left that it can sort (task.Supervisor). Pressing START again carries
on with the same bins, as in the manual window; CALIBRATE again is needed only if something changed
(the Nucleo lost power, the rig was moved or knocked).
"""

import time
from dataclasses import dataclass, field

from robot_control.task import aim_check
from robot_control.task.aim_check import AimCheck, camera_aimed, describe_for
from robot_control.task.app import SortingApp

BUTTONS = ["CALIBRATE", "START", "STOP"]
BEFORE_CALIBRATE = ("Plate EMPTY. After a power-on: servo supply OFF, by hand slide fully up, elbow folded 100+ deg, "
                    "gripper turned from the tower; supply ON. Then press CALIBRATE.")
AFTER_CALIBRATE = "Calibrated. Lay the resistors out on the plate, keeping the crosshair clear, and press START."


class CalibrationFailed(Exception):
    """A step of the standard calibration didn't pass: the reason says what to do."""


@dataclass
class CalibrationResult:
    """What a passed calibration found, for the window and the log."""
    finished: str                       # when, "HH:MM"
    elbow: object                       # motion.ElbowCheck
    aim: object                         # task.AimReport, or None if the arm aims by its CAD geometry
    steps: list = field(default_factory=list)       # what each step said, in order

    def summary(self):
        if self.aim is None:
            return f"calibrated {self.finished}: {self.elbow}, aim by the CAD geometry"
        return (f"calibrated {self.finished}: {self.elbow}, aim {self.aim.rms_after_mm:.1f} mm "
                f"(was {self.aim.rms_before_mm:.1f})")


class StandardCalibration:
    """The fixed calibration: arm, plate, aim, each checked before the next."""

    def __init__(self, arm, eyes, log, say=print, check_aim=True):
        """check_aim: the third step, for an arm aiming by the camera calibration. Without it (the CAD geometry)
        there are two."""
        self.arm, self.eyes, self.log, self.say = arm, eyes, log, say
        self.aims = check_aim
        self.of = 3 if check_aim else 2

    def run(self):
        """Every step in order. Returns a CalibrationResult; raises CalibrationFailed at the first that doesn't pass."""
        steps = []
        elbow = self.start_arm()
        steps.append(str(elbow))
        steps.append(self.check_plate_empty())
        aim = self.check_aim() if self.aims else None
        steps.append(aim.verdict if aim else "aim: the CAD geometry, nothing to check")
        result = CalibrationResult(time.strftime("%H:%M"), elbow, aim, steps)
        self.log.event(0, "calibrated", steps=steps)
        return result

    def start_arm(self):
        self.say(f"1/{self.of} starting the arm: the base and elbow sweep end to end, then the elbow check")
        elbow = self.arm.start_up(say=self.say)
        if not elbow.ok:
            raise CalibrationFailed(f"{elbow}. Free the cables, set the arm by hand again and CALIBRATE again")
        return elbow

    def check_plate_empty(self):
        self.say(f"2/{self.of} finding the plate and checking it is empty")
        found = self.eyes.count_bodies()
        if found:
            raise CalibrationFailed(f"the plate isn't empty: {found} resistor{'s' * (found != 1)} seen. CALIBRATE needs an "
                                    "empty plate (the aim check puts the tip down on it): clear it and CALIBRATE again")
        return "plate found and empty"

    def check_aim(self):
        self.say("3/3 checking the aim: the tip down at 5 points")
        report = AimCheck(self.eyes, self.arm).run(say=self.say)
        if report.found < aim_check.MIN_POINTS:
            raise CalibrationFailed(f"aim check: {report.verdict}")
        if report.rms_after_mm > aim_check.LEFT_OK_MM:
            raise CalibrationFailed(f"aim check: {report.verdict}")
        return report


class AutoSortingApp(SortingApp):
    """SortingApp with the procedure fixed (see above): CALIBRATE, then START."""

    def __init__(self, supervisor, window):
        self.calibration = None             # the last CalibrationResult that passed, in this window
        super().__init__(supervisor, window)
        self.window.say(BEFORE_CALIBRATE)

    def run(self):
        while True:
            pressed = self.window.poll(self.enabled())
            if pressed == "QUIT":
                if not self.busy:
                    return
                self.quitting = True
                self.stop()
            elif pressed == "CALIBRATE":
                self.launch("CALIBRATE", self.calibrate_standard)
            elif pressed == "START":
                self.launch("START", self.sort)
            elif pressed == "STOP":
                self.stop()
            if self.quitting and not self.busy:
                return

    def enabled(self):
        """CALIBRATE whenever nothing is running; START only once a calibration has passed; STOP during a run."""
        if self.busy:
            return {"STOP"} if self.job_name == "START" else set()
        if self.arm is None:
            return set()
        return {"CALIBRATE"} | ({"START"} if self.calibration is not None else set())

    def calibrate_standard(self):
        self.calibration = None
        try:
            result = StandardCalibration(self.arm, self.supervisor.eyes, self.supervisor.log, self.supervisor.say,
                                         check_aim=camera_aimed(self.arm)).run()
        except CalibrationFailed as why:
            self.supervisor.log.event(0, "calibration failed", why=str(why))
            self.window.show_arm(self.arm_line())
            self.supervisor.say(f"CALIBRATION FAILED: {why}")
            return
        self.calibration = result
        self.window.show_arm(f"arm: {result.summary()}")
        self.window.show_aim(describe_for(self.arm))
        self.supervisor.say(AFTER_CALIBRATE)

    def sort(self):
        ended = self.supervisor.run()
        self.supervisor.say(f"{ended}. Lay out more and press START, or q to quit")
