"""The firmware's kinematics (calc_inverse_kinematics in scara.c), mirrored so a target is checked
before anything is sent. The firmware does no checks of its own: an unreachable target gives NaN
angles and an out-of-range one drives the servos past their limits.

Robot frame (metres, degrees): x = l1·sin θ1 + l2·sin(θ1 + θ2), y = l1·cos θ1 + l2·cos(θ1 + θ2).
The tool angle sent with SCARA_MOVE_COORD is the wrist's absolute angle, θ1 + θ2 + θ3; the firmware
sets θ3 = tool angle − (θ1 + θ2) without wrapping it, so the wrap is chosen here.
"""

import math
from dataclasses import dataclass

from robot_control.world import JointState


@dataclass(frozen=True)
class IKResult:
    joints: JointState | None
    tool_angle: float | None        # the tool angle to send: the one whose θ3 was checked
    reason: str = ""                # why it was refused

    @property
    def ok(self):
        return self.joints is not None


class Kinematics:
    def __init__(self, arm, keep_outs=()):
        self.arm = arm              # config.ArmSettings
        self.keep_outs = tuple(keep_outs)   # keep_out.KeepOut zones no pose may be in

    def fk(self, j0, j1, j2=0.0):
        """Joint angles (deg) to (x, y, tool angle)."""
        l1, l2 = self.arm.link1, self.arm.link2
        a, b = math.radians(j0), math.radians(j0 + j1)
        return l1 * math.sin(a) + l2 * math.sin(b), l1 * math.cos(a) + l2 * math.cos(b), j0 + j1 + j2

    def reach(self):
        """(r_min, r_max) of the wrist from the base axis, from the elbow limits."""
        l1, l2 = self.arm.link1, self.arm.link2
        r = lambda q: math.sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * math.cos(math.radians(q)))
        return r(self.arm.j1[1]), r(self.arm.j1[0])

    def ik(self, x, y, tool_angle, z, symmetric=False):
        """Joints for the wrist at (x, y) m with the tool at tool_angle deg, or a refusal.

        symmetric: the tool is the same turned 180 deg, so tool_angle + 180 may be sent instead.
        Of the valid tool angles (± 360, and + 180 if symmetric) the one needing the smallest wrist
        rotation is used.
        """
        arm = self.arm
        l1, l2 = arm.link1, arm.link2
        r = math.hypot(x, y)
        if r > l1 + l2:
            return IKResult(None, None, f"out of reach: {r * 1000:.1f} mm from the base, more than {(l1 + l2) * 1000:.0f}")
        if r < abs(l1 - l2) or r == 0:
            return IKResult(None, None, f"too close: {r * 1000:.1f} mm from the base")
        phi1 = math.acos(_clip((l2 * l2 - r * r - l1 * l1) / (-2 * r * l1)))
        phi2 = math.atan2(x, y)
        phi3 = math.acos(_clip((r * r - l1 * l1 - l2 * l2) / (-2 * l1 * l2)))
        j0 = math.degrees(phi2 - phi1)
        j1 = math.degrees(math.pi - phi3)
        if not _inside(j1, arm.j1):
            near = j1 > arm.j1[1]
            return IKResult(None, None, f"elbow {j1:.1f} deg outside {arm.j1[0]:.0f}..{arm.j1[1]:.0f}"
                                        + (f" (closer than {self.reach()[0] * 1000:.1f} mm)" if near else ""))
        if not _inside(j0, arm.j0):
            return IKResult(None, None, f"base {j0:.1f} deg outside {arm.j0[0]:.0f}..{arm.j0[1]:.0f}")
        if not _inside(z, arm.z):
            return IKResult(None, None, f"slide {z:.3f} m outside {arm.z[0]:.3f}..{arm.z[1]:.3f}")
        for k in self.keep_outs:
            if k.blocks(j0, j1):
                return IKResult(None, None, f"in the {k.name} keep-out (base {j0:.1f}, elbow {j1:.1f} deg)")
        options = [tool_angle + k * 360.0 for k in range(-2, 3)]
        if symmetric:
            options += [a + 180.0 for a in options]
        valid = [(abs(a - (j0 + j1)), a) for a in options if _inside(a - (j0 + j1), arm.j2)]
        if not valid:
            return IKResult(None, None, f"no tool angle near {tool_angle:.1f} deg keeps the wrist in "
                                        f"{arm.j2[0]:.0f}..{arm.j2[1]:.0f}")
        _, angle = min(valid)
        return IKResult(JointState(j0, j1, angle - (j0 + j1), z), angle)


def _clip(v):
    return max(-1.0, min(1.0, v))


def _inside(v, rng, tol=1e-6):
    return rng[0] - tol <= v <= rng[1] + tol
