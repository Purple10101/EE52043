"""Supervisor: runs the sorting cycle from start to finish. The top of the program.

One cycle:

    look      the arm parked, the camera sees the plate: every resistor, where it lies, its value
    judge     was the last pick a success? (its spot is empty now)
    plan      which resistor next, and which bin it goes in
    pick      over it, down, grip, up
    measure   (with the ohmmeter) its leads pressed on the meter's pads, still held, and its bin chosen by that reading
    drop      over its bin, down, open, up
    park      out of the camera's view, ready to look again

and again, until nothing is left on the plate that the run can sort. Looking again after every pick
means nothing has to be tracked between picks: a resistor knocked aside is simply somewhere new, and a
missed pick shows up as a resistor still where the jaws went.

Each look builds on the ones before (Eyes' BodyMemory): a body is followed by where it lies, its reading
averaged over its last few looks and its close-up. So the run never ends while a body it has seen is
missing from the latest look: it looks again (CONFIRM_LOOKS) first, as a body can be missed by one look.

The Supervisor decides nothing itself and moves nothing itself: Eyes see, the Planner decides, the Arm
moves, the window and the log report. It only puts them in order and decides when to stop. Its BinPlan
and PickMemory last as long as it does, so a run stopped and started again carries on with the same bins.
"""

import threading

from robot_control.hardware.ohmmeter import OhmmeterError
from robot_control.motion.arm import ArmError
from robot_offline.post.bands.learned import ohms_text
from robot_control.planning.memory import Attempt
from robot_control.vision.occlusion import arm_hides_plate
from robot_control.task.run_log import describe_reading
from robot_control.vision.eyes import EyesError

MAX_CYCLES = 100            # a run never makes more picks than this
MAX_FAILS_IN_A_ROW = 3      # picks in a row that came to nothing: something is wrong (calibration?), so stop
DROP_TRIES = 2
CONFIRM_LOOKS = 2           # looks again this many times before ending while a body seen before is missing
# Where the arm waits for a look when it isn't already clear of the camera's view: over tray slot A2 (its taught
# angles, bins_real.toml), which hides none of the plate (vision/occlusion.py). Much nearer the trays than the
# park on the far side of the plate, which the run used to swing to and from every cycle.
LOOK_POSE = (56.3, 116.4)


class Supervisor:
    def __init__(self, eyes, arm, planner, log, window=None, ohmmeter=None):
        """eyes: vision.Eyes. arm: motion.Arm, or None to look and plan only. planner: planning.Planner.
        log: task.RunLog. window: task.RunWindow, or None to show nothing. ohmmeter: hardware.ohmmeter's
        SerialOhmmeter (or FakeOhmmeter), needed when the planner measures, else None."""
        if planner.measure and ohmmeter is None and arm is not None:
            raise ValueError("the planner measures resistors, but there is no ohmmeter")
        self.eyes, self.arm, self.planner, self.log = eyes, arm, planner, log
        self.ohmmeter = ohmmeter
        self.window = window or QuietWindow()
        self.fails_in_a_row = 0
        self.stop_asked = threading.Event()     # set from the window's thread; read between picks

    @property
    def memory(self):
        return self.planner.memory

    @property
    def bins(self):
        return self.planner.bins

    def request_stop(self):
        """Stop once the arm is parked again: never mid-move, never holding a resistor."""
        self.stop_asked.set()

    def run(self):
        """Sorts until nothing is left that can be sorted, or something stops it. Returns why it ended."""
        self.stop_asked.clear()
        self.log.start_run()
        self.fails_in_a_row = 0
        try:
            self.say("parking before the first look")
            self.arm.park()
            self.read_values()
            for cycle in range(1, MAX_CYCLES + 1):
                ended = self.cycle(cycle)
                if ended:
                    break
            else:
                ended = f"stopped after {MAX_CYCLES} cycles"
            if not self.arm.holding:
                self.arm.park()             # ready for the next START or CALIBRATE
        except (ArmError, EyesError, OhmmeterError) as why:
            ended = f"STOPPED: {why}"
        self.log.event(0, "end", why=ended, sorted=len(self.memory.sorted), bins=self.bins.assigned)
        self.update_trays()
        self.say(ended)
        return ended

    def cycle(self, n):
        """One look and, if there is something to sort, one pick. Returns why the run should end, or None."""
        view = self.eyes.look()
        outcome = self.judge(n, view)
        plan = self.planner.plan(view)
        self.show(n, view, plan, outcome)
        for _ in range(CONFIRM_LOOKS):
            if plan.next is not None or not view.missing:
                break
            self.say(f"nothing to pick, but {len(view.missing)} resistor{'s' * (len(view.missing) != 1)} seen before "
                     "isn't in this look: looking again")
            self.log.event(n, "looking again", missing=[[round(v, 1) for v in c] for c in view.missing])
            self.arm.park()                 # from the park, which is known to hide nothing, in case the arm did
            view = self.eyes.look()
            plan = self.planner.plan(view)
            self.show(n, view, plan, None)
        if plan.next is None:
            return f"done: nothing left to sort ({self.summary(view)})"
        if self.fails_in_a_row >= MAX_FAILS_IN_A_ROW:
            return f"STOPPED: {self.fails_in_a_row} picks in a row came to nothing. Check the calibration"
        if self.stop_asked.is_set():
            return "stopped: you asked"
        self.pick_and_drop(n, plan.next)
        self.clear_view()
        self.update_trays()
        return None

    def clear_view(self):
        """Out of the camera's view for the next look, the shortest way: where the arm is, if it hides none of
        the plate (after a drop into an A slot); else over A2. Not the park on the far side of the plate."""
        base, elbow = self.arm.joints()
        kinematics, rig = self.arm.navigator.kinematics, self.arm.navigator.rig
        if not arm_hides_plate(base, elbow, self.eyes.lens_mm(), kinematics, rig):
            self.say("looking from where the arm is: it's clear of the camera's view")
            return
        self.say("moving clear of the camera's view")
        self.arm.move_to(*LOOK_POSE)

    def judge(self, n, view):
        """How the last pick went, as this look shows it."""
        outcome = self.memory.judge(view)
        if outcome is not None:
            self.fails_in_a_row = 0 if outcome == "sorted" else self.fails_in_a_row + 1     # "missed (pushed away)" too
            self.log.event(n, "last pick " + outcome)
        self.update_trays()
        return outcome

    def pick_and_drop(self, n, pick):
        r = pick.resistor
        heading = pick.bin_id or "the ohmmeter"
        self.update_trays(target=pick.bin_id)
        self.say(f"picking #{r.number}, read {describe_reading(r)}, for {heading}"
                 + (" (measured first)" if pick.measure and pick.bin_id else ""))
        self.log.event(n, "pick", resistor=r.number, body=r.body_id, ohm=r.value_ohm, confidence=round(r.confidence, 3),
                       colours=" ".join(r.colours), closeup=r.closeup, bin=pick.bin_id, measure=pick.measure,
                       x_mm=round(r.centre_mm[0], 1), y_mm=round(r.centre_mm[1], 1))
        value, bin_id = (None, self.bins.unclassified) if pick.bin_id is None else (r.value_ohm, pick.bin_id)
        if not self.arm.pick(r):
            self.fails_in_a_row += 1
            self.memory.give_up(r.centre_mm, f"the arm couldn't pick it ({self.arm.message})")
            self.log.event(n, "pick refused", why=self.arm.message)
            if self.arm.holding:            # it stopped after the jaws closed: whatever they hold must go
                self.drop(n, bin_id, value)
            return
        meter_failed = None
        if pick.measure:
            try:
                value, bin_id = self.measure(n, r)
            except OhmmeterError as why:    # drop what it holds in the reject bin first, then stop the run
                self.log.event(n, "measure failed", why=str(why))
                value, bin_id, meter_failed = None, self.bins.unclassified, why
        self.memory.attempted(Attempt(r.centre_mm, value, bin_id))
        self.eyes.picked(r.centre_mm)       # still a body next look if anything is found there: a failed pick
        self.drop(n, bin_id, value)
        if meter_failed is not None:
            raise OhmmeterError(f"the ohmmeter stopped answering: {meter_failed}")

    def measure(self, n, r):
        """The resistor in the jaws, read by the ohmmeter: carried to its station (the meter's settings.station),
        its leads pressed on the pads without letting go, measured, lifted. Returns (value, bin): its E24 value
        and the bin that value goes in, or (None, the reject bin) when the meter can't place it or the arm
        couldn't get there. Raises OhmmeterError if the meter doesn't answer."""
        reject = self.bins.unclassified
        station = getattr(getattr(self.ohmmeter, "settings", None), "station", None)
        self.say(f"measuring #{r.number}")
        if station is not None and not self.arm.press_on_meter(station):
            self.log.event(n, "measured", resistor=r.number, ohms=None, samples=[], nominal=None, read=r.value_ohm,
                           agrees=None, bin=reject, why=f"couldn't reach the ohmmeter: {self.arm.message}")
            self.say(f"couldn't reach the ohmmeter ({self.arm.message}): #{r.number} into {reject}")
            return None, reject
        try:
            m = self.ohmmeter.measure()
        finally:
            if station is not None:
                self.arm.lift()
        value, why = m.nominal_ohm, m.why
        bin_id = self.bins.bin_for(value) if value is not None else reject
        if value is not None and bin_id == reject:
            why = f"{ohms_text(value)} is outside the sorter's range"
        elif bin_id is None:
            why, bin_id = f"no bin free for {ohms_text(value)}", reject
        if bin_id == reject:
            value = None
        read = r.value_ohm
        agrees = None if read is None or m.nominal_ohm is None else read == m.nominal_ohm
        self.log.event(n, "measured", resistor=r.number, ohms=None if m.ohms is None else round(m.ohms, 2),
                       samples=[round(v, 2) for v in m.samples], nominal=m.nominal_ohm, read=read, agrees=agrees,
                       bin=bin_id, why=why)
        if value is None:
            self.say(f"the ohmmeter can't place #{r.number} ({why}): into {bin_id}")
        elif agrees is False:
            self.say(f"the ohmmeter says {ohms_text(value)} where the bands said {ohms_text(read)}: "
                     f"going by the meter, into {bin_id}")
        else:
            self.say(f"measured {ohms_text(value)} ({m.ohms:.1f} ohm): into {bin_id}")
        return value, bin_id

    def drop(self, n, bin_id, value):
        """Into bin_id; value (ohms) keeps that bin from then on, unless it is the unclassified bin. Raises
        ArmError if it can't let go there."""
        self.say(f"taking it to {bin_id}")
        for _ in range(DROP_TRIES):
            if self.arm.drop(bin_id):
                if bin_id != self.bins.unclassified:
                    self.bins.assign(value)
                self.log.event(n, "dropped", bin=bin_id)
                self.update_trays(target=bin_id)
                return
            self.log.event(n, "drop failed", why=self.arm.message)
        raise ArmError(f"holding a resistor it couldn't drop in {bin_id}: {self.arm.message}")

    def read_values(self):
        """The start-of-run read (Eyes.read_values): every resistor's value, once, after a focus calibration.
        Each look after it only finds the resistors and matches them to these readings."""
        self.say("reading every resistor's value (once per run)")
        view = self.eyes.read_values()
        read = [r for r in view.resistors if r.is_read]
        self.log.event(0, "values read", resistors=len(view.resistors), read=len(read),
                       values=[r.value_ohm for r in read])
        return view

    def look_once(self):
        """One read and its plan, without moving the arm: what the run would pick first, and why the rest
        would stay. The arm must already be parked. Returns the Plan."""
        view = self.eyes.read_values()
        plan = self.planner.plan(view)
        self.show(0, view, plan, None)
        self.say(f"looked: {plan.waiting} to pick" + (f", #{plan.next.resistor.number} first" if plan.next else ""))
        return plan

    def show(self, n, view, plan, outcome):
        lines = [f"{'look ' + str(n) + ': ' if n else ''}{self.summary(view)}"
                 + (f"   last pick: {outcome}" if outcome else "")]
        image = self.window.show(view, plan, lines)
        seq = self.log.look(n, view, plan, image)
        self.log.event(n, "look", look=seq, resistors=len(view.resistors), read=sum(r.is_read for r in view.resistors),
                       to_pick=plan.waiting, faint=len(view.faint), missing=len(view.missing),
                       focus=getattr(view, "focus", ""))
        for number, fate in sorted(plan.fates.items()):
            print(f"  #{number}: {fate}", flush=True)

    def update_trays(self, target=None):
        """Tells the window what each bin holds, and which one the arm is heading for (if any)."""
        self.window.show_trays(self.bins.trays(self.memory.sorted), target)

    def summary(self, view):
        return (f"{len(view.resistors)} on the plate, {sum(r.is_read for r in view.resistors)} read, "
                f"{len(self.memory.sorted)} sorted so far")

    def say(self, text):
        print(text, flush=True)
        self.window.say(text)


class QuietWindow:
    """Shows nothing: for tests, and for runs without a screen."""

    def show(self, view, plan, lines):
        return None

    def show_trays(self, trays, target=None):
        pass

    def say(self, text):
        pass
