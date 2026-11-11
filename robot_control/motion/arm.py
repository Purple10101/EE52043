"""Arm: the robot's moves as a sorting run needs them. Start up after power-on, park, pick, drop.

Each move says whether it worked. The moves themselves are resistor_nav's Navigator and slide_wrist's
bench steps (scripts/), the same ones the navigation window and the README's procedures use on the rig:
every one goes through slide_wrist's checks (joint limits, the camera tower, the plate, the non-resistor bin),
and a move that isn't allowed is refused before anything moves. This class adds no moves of its own; it
gives the run a plain interface and turns refusals into answers.
"""

from dataclasses import dataclass

from robot_control.gripper import GripperError

REFUSALS = (SystemExit, ValueError, GripperError)   # how the Navigator and the bench stop a move
ELBOW_CHECK = (105.0, 115.0, 110.0)                  # the README's check after a start, from the park
ELBOW_CHECK_TOLERANCE_DEG = 1.0                      # each should land this close: more means start learned it wrong


class ArmError(Exception):
    """The arm is somewhere a person has to look at: it holds a resistor it couldn't drop, couldn't park,
    or couldn't start."""


@dataclass(frozen=True)
class ElbowCheck:
    """Where the elbow landed for each target of the check after a start."""
    landings: tuple                 # (target, landed) deg

    @property
    def worst_deg(self):
        return max(abs(landed - target) for target, landed in self.landings)

    @property
    def ok(self):
        return self.worst_deg <= ELBOW_CHECK_TOLERANCE_DEG

    def __str__(self):
        verdict = "good" if self.ok else "BAD, start again with nothing dragging on the arm"
        return f"elbow check {verdict} ({self.worst_deg:.2f} deg off)"


class Arm:
    def __init__(self, navigator, slots, bench):
        """navigator: a resistor_nav.Navigator, not started (its moves are called here, one at a time).
        slots: the trays' slots, {bin id: entry} as resistor_nav.load_bins gives. bench: the slide_wrist
        module, for the start-up steps."""
        self.navigator = navigator
        self.slots = slots
        self.bench = bench

    @property
    def holding(self):
        """Whether the jaws closed on something and haven't let go since. They can't feel a resistor,
        so after a miss this is still True."""
        return self.navigator.holding

    @property
    def message(self):
        """What the last move said."""
        return self.navigator.status

    @property
    def last_started(self):
        """When the arm was last started ("YYYY-MM-DD HH:MM"), or None if never. A power cut since can't be
        seen from here: the Nucleo forgets, and still accepts moves."""
        state = self.bench.load_state()
        return None if state is None else state.get("started", "before start times were kept")

    def start_up(self, say=print):
        """After the Nucleo is powered on: the README's start sequence, then its elbow check. The joints
        are calibrated by sweeping end to end, the elbow folded straight out of the tower's keep-out, the
        arm parked (slide fully up) and the wrist given its first move. Returns the ElbowCheck.

        Before it, with the servo supply off: the slide raised fully by hand, the elbow folded to 100 deg
        or more, the gripper turned away from the camera tower, the plate clear. Raises ArmError if a
        step is refused."""
        b, scara = self.bench, self.navigator.scara
        if self.holding:
            raise ArmError("won't start holding a resistor")
        try:
            say("starting: the base and elbow jump to zero, then sweep end to end")
            b.start(scara)
            say("folding the elbow out of the tower's keep-out")
            b.move(scara, "elbow", 110.0, b.MOVE_S)
            say("parking")
            b.park(scara)
            b.move(scara, "wrist", 0.0, b.WRIST_TURN_S)
            landings = []
            for target in ELBOW_CHECK:
                say(f"elbow check: to {target:g}")
                b.move(scara, "elbow", target, b.MOVE_S)
                landings.append((target, b.read_joints(scara)[1]))
        except REFUSALS as why:
            raise ArmError(f"start-up stopped: {why}") from None
        self.navigator.parked = True
        return ElbowCheck(tuple(landings))

    def park(self):
        """Out of the camera's view, the slide fully up. Raises ArmError if it can't."""
        if self.holding:
            raise ArmError("won't park holding a resistor: it must go to a bin first")
        try:
            self.navigator.park()
        except REFUSALS as why:
            raise ArmError(f"couldn't park: {why}") from None

    def pick(self, resistor):
        """Over the resistor, jaws across it, down, close, up. True if the jaws closed where it lay (the
        next look tells whether they took it); False if the arm didn't get there, so nothing was lowered."""
        try:
            return bool(self.navigator.pick(resistor.centre_mm, resistor.direction_deg))
        except REFUSALS as why:
            self.navigator.say(f"pick stopped: {why}")
            return False

    def drop(self, bin_id):
        """To the bin, down, open, up. True if it got there and let go."""
        try:
            return bool(self.navigator.to_bin(bin_id, self.slots[bin_id]))
        except REFUSALS as why:
            self.navigator.say(f"drop stopped: {why}")
            return False

    def touch(self, point_mm, lens_mm):
        """The closed tip down at a plate point, face-on to the camera, as camera_calibrate's check visits
        one: up to the travel height, over the point with the wrist taking up its play the same way each
        time, down to the pick height, the base and elbow landed on their angles. Stays there for the camera.
        Returns the potentiometers' (base, elbow) and the wrist, read there. Raises ArmError if refused."""
        from robot_control.vision.guide import plan_point
        b, n = self.bench, self.navigator
        try:
            base, elbow, wrist = plan_point(point_mm, lens_mm, n.kinematics, n.rig, n.correction)
            n.parked = False
            if n.gripper is not None:
                n.gripper.close()           # the camera finds the tip by the closed jaws' V
            n.lift()
            b.approach(n.scara, base, elbow, wrist, b.SWING_S)
            n.drive("slide", b.OVER_PLATE_SLIDE, 1.0)
            b.land(n.scara, {"base": base, "elbow": elbow}, wrist, b.OVER_PLATE_SLIDE)
            got = b.read_joints(n.scara)
        except REFUSALS as why:
            raise ArmError(f"couldn't reach the check point {tuple(point_mm)}: {why}") from None
        return float(got[0]), float(got[1]), float(b.load_state()["wrist"])

    def lift(self):
        """The slide up to the travel height, where it isn't already higher."""
        try:
            self.navigator.lift()
        except REFUSALS as why:
            raise ArmError(f"couldn't lift: {why}") from None
