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
4. With the ohmmeter on (config/ohmmeter.toml): it answers, and reads an open circuit with nothing on its pads.
   Checked first, as it moves nothing: a meter that isn't up fails CALIBRATE before the arm sweeps.

With the ohmmeter, START measures the resistors the band reader isn't sure of: each is picked, its leads
pressed on the meter's pads still in the jaws (config/ohmmeter_station.toml), and binned by what the meter
reads; one the meter can't place goes in the reject bin.

Then START reads every resistor's value once (a focus sweep, then a close-up of each: vision.Eyes.read_values)
and sorts until nothing is left that it can sort (task.Supervisor). Pressing START again carries
on with the same bins, as in the manual window; CALIBRATE again is needed only if something changed
(the Nucleo lost power, the rig was moved or knocked).
"""

import time
from dataclasses import dataclass, field

from robot_control.hardware.ohmmeter import OhmmeterError
from robot_control.task import aim_check
from robot_control.task.aim_check import AimCheck, camera_aimed, describe_for
from robot_control.task.app import SortingApp

BUTTONS = ["CALIBRATE", "START", "STOP"]
BEFORE_CALIBRATE = ("Plate EMPTY. After a power-on: servo supply OFF, by hand slide fully up, elbow folded 100+ deg, "
                    "gripper turned from the tower; supply ON. Then press CALIBRATE.")
AFTER_CALIBRATE = "Calibrated. Lay the resistors out on the plate, keeping the crosshair clear, and press START."
SKIPPED_CALIBRATE = ("Calibration skipped: the arm must already be started since the Nucleo's last power-on, and "
                     "the rig not moved since the aim was checked. Lay the resistors out, keeping the crosshair "
                     "clear, and press START (or CALIBRATE on an empty plate first).")
SKIPPED = object()          # AutoSortingApp.calibration when run with skip_calibration: START offered without one


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
    """The fixed calibration: the meter (if any), arm, plate, aim, each checked before the next."""

    def __init__(self, arm, eyes, log, say=print, check_aim=True, meter=None):
        """check_aim: the aim step, for an arm aiming by the camera calibration. Without it (the CAD geometry)
        there is one step fewer. meter: the ohmmeter (hardware.ohmmeter), or None when it is off."""
        self.arm, self.eyes, self.log, self.say = arm, eyes, log, say
        self.aims, self.meter = check_aim, meter
        self.step = 0

    @property
    def of(self):
        return 2 + bool(self.aims) + (self.meter is not None)

    def announce(self, text):
        self.step += 1
        self.say(f"{self.step}/{self.of} {text}")

    def run(self):
        """Every step in order. Returns a CalibrationResult; raises CalibrationFailed at the first that doesn't pass."""
        self.step = 0
        steps = []
        if self.meter is not None:
            steps.append(self.check_meter())
        elbow = self.start_arm()
        steps.append(str(elbow))
        steps.append(self.check_plate_empty())
        aim = self.check_aim() if self.aims else None
        steps.append(aim.verdict if aim else "aim: the CAD geometry, nothing to check")
        result = CalibrationResult(time.strftime("%H:%M"), elbow, aim, steps)
        self.log.event(0, "calibrated", steps=steps)
        return result

    def check_meter(self):
        self.announce("checking the ohmmeter: it must answer, and read open with nothing on its pads")
        try:
            m = self.meter.measure(settle=False)
        except OhmmeterError as why:
            raise CalibrationFailed(f"the ohmmeter isn't answering ({why}): check its USB cable, or run with "
                                    "--no-ohmmeter") from None
        if not m.is_open:
            got = "a short" if m.ohms == 0 else "no reading" if m.ohms is None else f"{m.ohms:.0f} ohm"
            raise CalibrationFailed(f"the ohmmeter reads {got} with nothing meant to be on its pads: clear them "
                                    "(a resistor or lead left there?) and CALIBRATE again")
        return "ohmmeter answering, pads clear"

    def start_arm(self):
        self.announce("starting the arm: the base and elbow sweep end to end, then the elbow check")
        elbow = self.arm.start_up(say=self.say)
        if not elbow.ok:
            raise CalibrationFailed(f"{elbow}. Free the cables, set the arm by hand again and CALIBRATE again")
        return elbow

    def check_plate_empty(self):
        self.announce("finding the plate and checking it is empty")
        found = self.eyes.count_bodies()
        if found:
            raise CalibrationFailed(f"the plate isn't empty: {found} resistor{'s' * (found != 1)} seen. CALIBRATE needs an "
                                    "empty plate (the aim check puts the tip down on it): clear it and CALIBRATE again")
        return "plate found and empty"

    def check_aim(self):
        self.announce("checking the aim: the tip down at 5 points")
        report = AimCheck(self.eyes, self.arm).run(say=self.say)
        if report.found < aim_check.MIN_POINTS:
            raise CalibrationFailed(f"aim check: {report.verdict}")
        if report.rms_after_mm > aim_check.LEFT_OK_MM:
            raise CalibrationFailed(f"aim check: {report.verdict}")
        return report


class AutoSortingApp(SortingApp):
    """SortingApp with the procedure fixed (see above): CALIBRATE, then START."""

    def __init__(self, supervisor, window, skip_calibration=False):
        """skip_calibration: START offered straight away, taking the arm as already started (slide_wrist.py start
        or an earlier CALIBRATE, since the Nucleo's last power-on) and the aim as it is in camera_calibration.json.
        Nothing here can tell if the Nucleo has lost power since: then the arm lands well off."""
        self.calibration = SKIPPED if skip_calibration else None    # the last CalibrationResult that passed here
        super().__init__(supervisor, window)
        self.window.say(SKIPPED_CALIBRATE if skip_calibration else BEFORE_CALIBRATE)

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
                                         check_aim=camera_aimed(self.arm), meter=self.supervisor.ohmmeter).run()
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
