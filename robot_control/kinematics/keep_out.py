"""Keep-out zones in joint space: poses and moves that would hit an obstacle (the camera tower).

The firmware moves every joint on the same time-scaled profile (move_js_scara: one move time, and
speed, acceleration and jerk all in proportion to each joint's distance), so every move, joint or
coordinate, is a straight line in joint space. A move is clear if every point along that line is
outside every map. The maps cover every wrist angle and slide height, so only base and elbow matter.
"""

import math

import numpy as np

from robot_control.world import JointState

PATH_STEP_DEG = 0.25        # a move is checked at least this finely, in its biggest joint's degrees


class KeepOut:
    def __init__(self, keep_out_map):
        self.map = keep_out_map     # config.KeepOutMap
        self.name = keep_out_map.name

    def blocks(self, j0, j1):
        """Whether (base, elbow) deg is in the zone: any of the grid points around it blocked, or off
        the grid (unknown counts as blocked)."""
        m = self.map
        a = (j0 - m.j0[0]) / m.j0[1]
        b = (j1 - m.j1[0]) / m.j1[1]
        rows, cols = m.blocked.shape
        if a < -1e-9 or b < -1e-9 or a > rows - 1 + 1e-9 or b > cols - 1 + 1e-9:
            return True
        a0, b0 = min(int(math.floor(a + 1e-9)), rows - 1), min(int(math.floor(b + 1e-9)), cols - 1)
        a1, b1 = min(a0 + (a - a0 > 1e-9), rows - 1), min(b0 + (b - b0 > 1e-9), cols - 1)
        return bool(m.blocked[a0:a1 + 1, b0:b1 + 1].any())

    def first_hit(self, start, end):
        """Where a straight joint-space move from start to end ((j0, j1) or JointState) first enters
        the zone, as (j0, j1), or None if it is clear."""
        (a0, a1), (b0, b1) = _angles(start), _angles(end)
        n = max(1, math.ceil(max(abs(b0 - a0), abs(b1 - a1)) / PATH_STEP_DEG))
        for t in np.linspace(0.0, 1.0, n + 1):
            p = (float(a0 + t * (b0 - a0)), float(a1 + t * (b1 - a1)))
            if self.blocks(*p):
                return p
        return None


def first_hit(keep_outs, start, end):
    """(zone name, (j0, j1)) where a move first enters any zone, or None."""
    for k in keep_outs:
        p = k.first_hit(start, end)
        if p is not None:
            return k.name, p
    return None


def route(keep_outs, start, end, elbow_max, step=1.0):
    """Waypoints (JointStates, ending with end) for a move from start to end that stays clear, or None.

    Straight there if that is clear. Otherwise the elbow folds first, at the start's base angle; the
    base swings across with the elbow folded; then the elbow unfolds at the end's base angle (the
    camera tower README's safe path). The smallest fold that clears every leg is used. The wrist and
    slide move on the middle leg."""
    if first_hit(keep_outs, start, end) is None:
        return [end]
    lowest = max(start.j1, end.j1)
    folds = [lowest] + [k * step for k in range(math.floor(lowest / step) + 1, math.floor(elbow_max / step + 1e-9) + 1)]
    for fold in folds:
        path = [(start.j0, start.j1), (start.j0, fold), (end.j0, fold), (end.j0, end.j1)]
        if all(first_hit(keep_outs, a, b) is None for a, b in zip(path, path[1:])):
            return [JointState(start.j0, fold, start.j2, start.z), JointState(end.j0, fold, end.j2, end.z), end]
    return None


def _angles(p):
    return (p.j0, p.j1) if isinstance(p, JointState) else (float(p[0]), float(p[1]))
