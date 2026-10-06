"""
scara_workspace.py

Working limits for the SCARA arm, derived from the workspace-boundary script.

Conventions (taken from that script, after its 90-deg rotation/flip):
    q0 = base joint (deg), q1 = elbow joint (deg), measured from the +Y axis,
         positive clockwise (towards +X)
    x = L1*sin(q0) + L2*sin(q0 + q1)
    y = L1*cos(q0) + L2*cos(q0 + q1)
    q0 in [-90, 90], q1 in [0, 90]  (elbow only bends one way)
All lengths are in METRES (the firmware/sim protocol units); the original
script used millimetres.

Joint limits are the single source of truth: joint moves are checked against
them directly, Cartesian moves are checked via inverse kinematics, so the
reachable set is exactly the green region of the original plot.
"""

import math
from dataclasses import dataclass


class WorkspaceLimitError(ValueError):
    """Raised when a command would leave the allowed working envelope."""


def _linspace(a, b, n):
    return [a + (b - a) * i / (n - 1) for i in range(n)]


@dataclass(frozen=True)
class WorkspaceLimits:
    l1: float = 0.125                  # m
    l2: float = 0.100                  # m
    j0: tuple = (-90.0, 90.0)          # base, deg
    j1: tuple = (0.0, 90.0)            # elbow, deg
    j2: tuple = (-180.0, 180.0)        # wrist, deg  (not in the script: assumed)
    z: tuple = (0.095, 0.15)           # Z height, m (hardware default)
    angle_tol: float = 1e-3            # deg, slack for float round-off at edges
    z_tol: float = 1e-6                # m

    # ------------------------------------------------------------ kinematics
    def fk(self, q0, q1):
        """Joint angles (deg) -> tip (x, y) in metres."""
        a = math.radians(q0)
        b = math.radians(q0 + q1)
        return (self.l1 * math.sin(a) + self.l2 * math.sin(b),
                self.l1 * math.cos(a) + self.l2 * math.cos(b))

    def ik(self, x, y):
        """Tip (x, y) -> (q0, q1) in degrees on the elbow >= 0 branch, or None
        if the point is outside the arm's geometric reach. Joint limits are
        NOT applied here."""
        c2 = (x * x + y * y - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        if abs(c2) > 1 + 1e-9:
            return None
        q1 = math.acos(max(-1.0, min(1.0, c2)))
        q0 = math.atan2(x, y) - math.atan2(self.l2 * math.sin(q1),
                                           self.l1 + self.l2 * math.cos(q1))
        q0 = math.degrees(q0)
        q0 = (q0 + 180.0) % 360.0 - 180.0
        return q0, math.degrees(q1)

    def reach_range(self):
        """(r_min, r_max) of the tip distance from the base axis, in metres."""
        def r(q1):
            return math.sqrt(self.l1 ** 2 + self.l2 ** 2
                             + 2 * self.l1 * self.l2 * math.cos(math.radians(q1)))
        return r(self.j1[1]), r(self.j1[0])

    # ---------------------------------------------------------------- checks
    def _in(self, v, rng, tol):
        return rng[0] - tol <= v <= rng[1] + tol

    def check_axis(self, axis, value):
        """Check one joint command (axis 0/1/2 = deg, 3 = Z in metres)."""
        names = {0: ("base", self.j0, self.angle_tol, "deg"),
                 1: ("elbow", self.j1, self.angle_tol, "deg"),
                 2: ("wrist", self.j2, self.angle_tol, "deg"),
                 3: ("Z", self.z, self.z_tol, "m")}
        if axis not in names:
            raise WorkspaceLimitError(f"Unknown axis {axis}")
        name, rng, tol, unit = names[axis]
        if not self._in(value, rng, tol):
            raise WorkspaceLimitError(
                f"{name} {value:.4g} {unit} is outside its limit "
                f"[{rng[0]:.4g}, {rng[1]:.4g}] {unit}")

    def check_joints(self, q0, q1, q2, z):
        for axis, v in enumerate((q0, q1, q2, z)):
            self.check_axis(axis, v)

    def check_coord(self, x, y, z_angle, z):
        """Check a Cartesian target via IK + joint limits."""
        sol = self.ik(x, y)
        r_lo, r_hi = self.reach_range()
        if sol is None:
            raise WorkspaceLimitError(
                f"({x:.3f}, {y:.3f}) m is out of reach "
                f"(r = {math.hypot(x, y):.3f} m, allowed {r_lo:.3f}-{r_hi:.3f} m)")
        q0, q1 = sol
        if not self._in(q0, self.j0, self.angle_tol) or not self._in(q1, self.j1, self.angle_tol):
            raise WorkspaceLimitError(
                f"({x:.3f}, {y:.3f}) m needs base {q0:.1f} deg / elbow {q1:.1f} deg, "
                f"outside limits base {self.j0}, elbow {self.j1}")
        self.check_axis(2, z_angle)
        self.check_axis(3, z)

    def contains(self, x, y):
        try:
            self.check_coord(x, y, 0.0, sum(self.z) / 2)
            return True
        except WorkspaceLimitError:
            return False

    # ------------------------------------------------------------- utilities
    def project(self, x, y):
        """Return a reachable tip position near (x, y) by clamping in joint
        space (approximately the nearest point; always inside the workspace)."""
        c2 = (x * x + y * y - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        q1 = math.degrees(math.acos(max(-1.0, min(1.0, c2))))
        q1 = max(self.j1[0], min(self.j1[1], q1))
        q1r = math.radians(q1)
        q0 = math.degrees(math.atan2(x, y)
                          - math.atan2(self.l2 * math.sin(q1r), self.l1 + self.l2 * math.cos(q1r)))
        q0 = (q0 + 180.0) % 360.0 - 180.0
        q0 = max(self.j0[0], min(self.j0[1], q0))
        return self.fk(q0, q1)

    def polygon(self, n_arc=100, n_wing=50):
        """Closed boundary of the reachable region as [(x, y), ...] in metres,
        traced exactly like the original plotting script."""
        j0lo, j0hi = self.j0
        j1lo, j1hi = self.j1
        pts = []
        pts += [self.fk(q0, j1lo) for q0 in _linspace(j0lo, j0hi, n_arc)]   # outer arc
        pts += [self.fk(j0hi, q1) for q1 in _linspace(j1lo, j1hi, n_wing)]  # wing
        pts += [self.fk(q0, j1hi) for q0 in _linspace(j0hi, j0lo, n_arc)]   # inner arc
        pts += [self.fk(j0lo, q1) for q1 in _linspace(j1hi, j1lo, n_wing)]  # wing
        return pts