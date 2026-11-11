"""ArmMonitor: what the robot is doing right now, for the UI to show.

    monitor = ArmMonitor(scara, gripper)        # the course API's Nucleo link, and the gripper board
    navigator = Navigator(monitor.nucleo, ..., gripper=monitor.gripper)     # hand these on instead
    monitor.pose()                              # an ArmPose: every joint now, mid-move included

Every command still goes to the hardware exactly as before: monitor.nucleo and monitor.gripper pass each
call straight through and only note it on the way. A move notes where each joint starts, where it is
going and how long it was given, so pose() can say where the arm is part way through. The firmware
moves every joint on one smooth time-scaled profile and answers once the move is done, so a minimum-jerk
curve over the move's time follows it closely, and the reply puts the joints at their targets. Every
potentiometer reading (SCARA_READ_ANGLE) corrects the base or elbow to where it really is.

Nothing here talks to the hardware on its own, so the UI can ask for the pose as often as it draws.
Angles are the firmware's, in degrees: base and elbow from their potentiometers, wrist and slide as
last commanded (the slide's negative angles are up).
"""

import threading
import time
from dataclasses import dataclass

JOINTS = ("base", "elbow", "wrist", "slide")    # the firmware's axes 0 to 3
JAWS_OPEN_MM = 12.0                             # gripper v4's jaws, fully open
JAWS_S = 0.4                                    # how long the jaws take to open or close


@dataclass(frozen=True)
class ArmPose:
    """The arm at one moment. A joint is None until the monitor has seen it moved or read."""
    base: float | None
    elbow: float | None
    wrist: float | None
    slide: float | None
    moving: bool                    # a move is under way
    goal: dict                      # the joints the move is heading for ({} when still)
    jaws_mm: float | None           # how far apart the jaws are; None before the gripper has moved
    holding: bool                   # the run believes a resistor is in the jaws
    calibrating: bool               # SCARA_AUTO_CALIBRATE is sweeping: base and elbow aren't known


def _smooth(u):
    """Minimum-jerk progress, 0 to 1, for a share u of the move's time."""
    u = min(max(u, 0.0), 1.0)
    return u * u * u * (10 - 15 * u + 6 * u * u)


class ArmMonitor:
    def __init__(self, scara, gripper=None):
        """scara: the course API's scara_motion_controller (or anything with its methods). gripper: a
        robot_control.gripper.Gripper, or None."""
        self._lock = threading.Lock()
        self._at = dict.fromkeys(JOINTS)        # where each joint was when the last move began or ended
        self._move = None                       # (start, goal, began, seconds)
        self._jaws = None                       # (from mm, to mm, began)
        self._calibrating = False
        self._holding = lambda: False
        self.nucleo = MonitoredNucleo(self, scara)
        self.gripper = None if gripper is None else MonitoredGripper(self, gripper)

    def know(self, **angles):
        """Joints known from elsewhere (the bench's saved wrist and slide), as name=deg; None is skipped."""
        with self._lock:
            self._at.update({k: float(v) for k, v in angles.items() if v is not None and k in JOINTS})

    def watch_holding(self, holding):
        """holding: a function saying whether the run believes the jaws hold a resistor."""
        self._holding = holding

    def pose(self, now=None):
        """The ArmPose at now (time.monotonic(); default now)."""
        now = time.monotonic() if now is None else now
        with self._lock:
            joints = self._pose_now(now)
            moving = self._move is not None
            goal = dict(self._move[1]) if moving else {}
            jaws = None
            if self._jaws is not None:
                was, to, began = self._jaws
                jaws = to if was is None else was + _smooth((now - began) / JAWS_S) * (to - was)
            calibrating = self._calibrating
        return ArmPose(joints["base"], joints["elbow"], joints["wrist"], joints["slide"], moving, dict(goal),
                       jaws, bool(self._holding()), calibrating)

    # Noted by the wrapped links.

    def _moving(self, goal, seconds):
        with self._lock:
            now = time.monotonic()
            current = self._pose_now(now)
            self._move = (current, {k: float(v) for k, v in goal.items()}, now, float(seconds))

    def _arrived(self):
        with self._lock:
            if self._move is not None:
                self._at.update(self._move[1])
                self._move = None

    def _read(self, joint, angle):
        with self._lock:
            if self._move is None:
                self._at[joint] = float(angle)

    def _sweeping(self, on):
        with self._lock:
            self._calibrating = on
            if on:
                self._move = None
                self._at["base"] = self._at["elbow"] = None     # unknown until read again

    def _jaws_to(self, mm):
        with self._lock:
            now = time.monotonic()
            if self._jaws is None:
                was = None
            else:
                a, b, began = self._jaws
                was = b if a is None else a + _smooth((now - began) / JAWS_S) * (b - a)
            self._jaws = (was, mm, now)

    def _pose_now(self, now):
        """Each joint where it is at now, mid-move included (the lock held)."""
        joints = dict(self._at)
        if self._move is not None:
            start, goal, began, seconds = self._move
            share = _smooth((now - began) / seconds) if seconds > 0 else 1.0
            for name, target in goal.items():
                joints[name] = target if start.get(name) is None else start[name] + share * (target - start[name])
        return joints


class MonitoredNucleo:
    """The Nucleo link with the monitor listening: the same methods, passed straight through."""

    def __init__(self, monitor, scara):
        self._monitor, self._scara = monitor, scara

    def SCARA_MOVE_JOINT(self, axis, angle, seconds):
        self._monitor._moving({JOINTS[axis]: angle}, seconds)
        try:
            return self._scara.SCARA_MOVE_JOINT(axis, angle, seconds)
        finally:
            self._monitor._arrived()

    def SCARA_MOVE_JOINTS(self, base, elbow, wrist, slide, seconds):
        self._monitor._moving(dict(base=base, elbow=elbow, wrist=wrist, slide=slide), seconds)
        try:
            return self._scara.SCARA_MOVE_JOINTS(base, elbow, wrist, slide, seconds)
        finally:
            self._monitor._arrived()

    def SCARA_READ_ANGLE(self, axis):
        angle = self._scara.SCARA_READ_ANGLE(axis)
        if axis in (0, 1):
            self._monitor._read(JOINTS[axis], angle)
        return angle

    def SCARA_AUTO_CALIBRATE(self):
        self._monitor._sweeping(True)
        try:
            return self._scara.SCARA_AUTO_CALIBRATE()
        finally:
            self._monitor._sweeping(False)

    def __getattr__(self, name):            # everything else, untouched
        return getattr(self._scara, name)


class MonitoredGripper:
    """The gripper board with the monitor listening: the same methods, passed straight through."""

    def __init__(self, monitor, gripper):
        self._monitor, self._gripper = monitor, gripper

    def open(self):
        self._monitor._jaws_to(JAWS_OPEN_MM)
        return self._gripper.open()

    def close(self):
        self._monitor._jaws_to(0.0)
        return self._gripper.close()

    def __getattr__(self, name):
        return getattr(self._gripper, name)
