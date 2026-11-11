"""SortingApp: the sorting program's buttons, each running one job in the background.

    CALIBRATE ARM   after the Nucleo is powered on: the start sequence and the elbow check (Arm.start_up).
                    Press it once to see what to set by hand first, and again to go.
    CHECK AIM       does the arm still go where the camera says? The tip down at 5 points, the steady shift
                    taken out of the calibration and saved (task.AimCheck). After the rig has moved, and
                    at the start of a session.
    LOOK            one look and its plan; the arm doesn't move (it must be parked).
    START           the sorting run (Supervisor.run), until nothing is left that it can sort.
    STOP            ends the run once the arm is parked again: never mid-move, never holding a resistor.
    q / Esc         quit, once nothing is running (while something is, it asks it to stop first).

One job at a time, in its own thread, so the window keeps drawing while the arm moves. The bins'
values and the run's memory belong to the Supervisor, so pressing START again carries on with the
same bins.
"""

import threading
import time

from robot_control.task.aim_check import AimCheck, describe

CONFIRM_S = 20.0            # a second press of CALIBRATE ARM within this long goes ahead
BEFORE_START = ("FIRST, with the servo supply OFF, by hand: slide fully up, elbow folded to 100 deg or more, "
                "gripper turned away from the tower, plate clear. Then supply ON and press CALIBRATE ARM again.")


class SortingApp:
    def __init__(self, supervisor, window):
        """supervisor: task.Supervisor (its arm None to look only). window: task.RunWindow."""
        self.supervisor, self.window = supervisor, window
        self.arm = supervisor.arm
        self.job, self.job_name = None, None
        self.confirm_until = 0.0
        self.quitting = False
        self.window.show_arm(self.arm_line())
        self.window.show_aim(describe() if self.arm else "")
        self.supervisor.update_trays()
        self.window.say("Ready. If the arm has been powered off since it was last started, press CALIBRATE ARM. "
                        "Then LOOK to check the plan, and START to sort."
                        if self.arm else "Looking only (no arm): press LOOK")

    def run(self):
        """The window's loop, until quit. Returns once any job has finished."""
        while True:
            pressed = self.window.poll(self.enabled())
            if pressed == "QUIT":
                if not self.busy:
                    return
                self.quitting = True
                self.stop()
            elif pressed == "CALIBRATE ARM":
                self.calibrate()
            elif pressed == "CHECK AIM":
                self.launch("CHECK AIM", self.check_aim)
            elif pressed == "LOOK":
                self.launch("LOOK", self.supervisor.look_once)
            elif pressed == "START":
                self.launch("START", self.supervisor.run)
            elif pressed == "STOP":
                self.stop()
            if self.quitting and not self.busy:
                return

    @property
    def busy(self):
        return self.job is not None and self.job.is_alive()

    def enabled(self):
        """The buttons that make sense now."""
        if self.busy:
            return {"STOP"} if self.job_name == "START" else set()
        return {"LOOK"} | ({"CALIBRATE ARM", "CHECK AIM", "START"} if self.arm else set())

    def calibrate(self):
        """The first press shows what to set by hand; a second within CONFIRM_S runs the start-up."""
        if time.monotonic() > self.confirm_until:
            self.confirm_until = time.monotonic() + CONFIRM_S
            self.window.say(BEFORE_START)
            print(BEFORE_START, flush=True)
            return
        self.confirm_until = 0.0

        def start_up():
            check = self.arm.start_up(say=self.supervisor.say)
            self.window.show_arm(self.arm_line(check))
            self.supervisor.say(f"arm started and parked: {check}" + ("" if check.ok else ". Press CALIBRATE ARM to redo it"))

        self.launch("CALIBRATE ARM", start_up)

    def check_aim(self):
        """AimCheck: the tip down at 5 points, the shift measured and, if worth it, taken out and saved."""
        AimCheck(self.supervisor.eyes, self.arm).run(say=self.supervisor.say)
        self.window.show_aim(describe())

    def stop(self):
        if self.job_name == "START" and self.busy:
            self.supervisor.request_stop()
            self.window.say("stopping once the arm is parked again...")

    def launch(self, name, job):
        """Runs job in the background; an error ends it with the reason on the status line."""
        def work():
            try:
                job()
            except Exception as why:        # shown, not swallowed: the job is over either way
                self.supervisor.say(f"{name} stopped: {type(why).__name__}: {why}")
                raise
        self.job_name = name
        self.job = threading.Thread(target=work, name=name, daemon=True)
        self.job.start()

    def arm_line(self, check=None):
        if self.arm is None:
            return "arm: not connected (looking only)"
        started = self.arm.last_started
        if not started:
            return "arm: NOT STARTED: press CALIBRATE ARM"
        return f"arm: started {started}" + (f", {check}" if check else "")
