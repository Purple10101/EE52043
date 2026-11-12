"""BodyMemory: each resistor on the plate followed from look to look, and the value it was read as.

Two things, kept apart (robot_offline's handoff 2026-11-11_ee52043_read_once_plan.md):

**Where the bodies are: every look.** Segmentation runs each time the arm is parked, and the bodies are
followed by where they lie, as robot_offline's Steady does (inf/steady.py). A new body needs the normal
confidence; one already seen counts while it is found at all (segmentation runs down to FLOOR, as Steady
keeps a body whose score swings). One found nowhere is "missing", not gone, until it's missed GONE_LOOKS
looks in a row; the run looks again rather than end while one is missing. A body the arm has picked is
marked: if the next look finds nothing there it's gone, and if anything is found there the pick failed and
it is still that body (on 2026-11-11 a failed pick left a 5k6 that segmentation then scored too low for 11
looks, and the run took the empty spot for a success).

**What each body's value is: read once, at the start of a run.** The camera's focus drifts while it is in use
(about 4 settings a minute), and the bands need it sharp, so the band reader runs once, straight after a
focus calibration (vision.Eyes.read_values), and its readings are kept here for the run (set_reading). After
that a body keeps its value only while it is where it was read: within MATCH_MM, its long axis within
MATCH_DEG. A body that has moved further, or one not there at the read (missed, or uncovered later), has no
value: it is to be measured (with the ohmmeter, once there is one).
"""

from dataclasses import dataclass

import numpy as np

MATCH_MM = 3.0          # the same body if its centre is this close (robot_offline closeup.MATCH_PX, 3 mm)
MATCH_DEG = 15.0        # and its long axis within this (the handoff's tolerance)
FLOOR = 0.05            # segmentation is run this low (robot_offline inf/steady.py FLOOR): a known body still counts
GONE_LOOKS = 2          # a body missed this many looks in a row is forgotten


@dataclass
class Body:
    id: int
    centre_mm: tuple
    direction_deg: float
    missed: int = 0             # looks in a row it wasn't found at all
    looks: int = 0              # looks it was found in
    picked: bool = False        # the arm has just picked it: gone if the next look doesn't find it
    reading: object = None      # its learned.Reading from the start-of-run read, or None if it wasn't read
    read_at: tuple = None       # (centre mm, direction deg) where it lay when it was read
    moved: bool = False         # it has moved since it was read, so its reading no longer holds


def _angle_apart(a, b):
    """Degrees between two long axes (which have no head or tail): 0 to 90."""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


class BodyMemory:
    def __init__(self, conf=0.5, match_mm=MATCH_MM, match_deg=MATCH_DEG, gone=GONE_LOOKS):
        """conf: the segmentation score a new body needs."""
        self.conf, self.match_mm, self.match_deg, self.gone = conf, match_mm, match_deg, gone
        self.bodies = []
        self._next_id = 1

    def match(self, detections):
        """detections: [(centre_mm, score, direction_deg)] from one look, at FLOOR or above. Returns, in order,
        each one's body id, or None for a detection that isn't a body (a new one under conf). Bodies not
        found count a miss; those missed GONE_LOOKS in a row, or picked and not found, are forgotten."""
        order = sorted(range(len(detections)), key=lambda i: -detections[i][1])
        ids, taken = [None] * len(detections), set()
        for i in order:
            centre, score, direction = detections[i]
            near = [(self._distance(b.centre_mm, centre), b) for b in self.bodies if b.id not in taken]
            near = [(d, b) for d, b in near if d <= self.match_mm]
            if near:
                body = min(near, key=lambda n: n[0])[1]
            elif score >= self.conf:
                body = Body(self._next_id, tuple(centre), float(direction))
                self._next_id += 1
                self.bodies.append(body)
            else:
                continue                    # faint and new: not a body (yet)
            body.centre_mm, body.direction_deg = tuple(centre), float(direction)
            body.missed, body.picked = 0, False
            body.looks += 1
            if body.read_at is not None and not self._still_where_read(body):
                body.moved = True
            ids[i] = body.id
            taken.add(body.id)
        for body in self.bodies:
            if body.id not in taken:
                body.missed += 1
        self.bodies = [b for b in self.bodies if b.missed < self.gone and not (b.picked and b.missed)]
        return ids

    def set_reading(self, body_id, reading):
        """The start-of-run read of this body: its value holds while it stays where it was read."""
        body = self.body(body_id)
        body.reading, body.read_at, body.moved = reading, (body.centre_mm, body.direction_deg), False

    def reading(self, body_id):
        """The body's reading if it was read this run and hasn't moved since; None otherwise."""
        body = self.body(body_id)
        return None if body.moved else body.reading

    def why_no_value(self, body_id):
        """Why a body has no trusted value, in a few words ("" if it has one)."""
        body = self.body(body_id)
        if body.reading is None:
            return "not read this run (missed at the start, or uncovered since)"
        if body.moved:
            return "moved since it was read"
        if body.reading.value is None:
            return "unsure"
        return ""

    def missing(self):
        """Bodies seen before but not in the last look (and not yet given up)."""
        return [b for b in self.bodies if b.missed > 0]

    def picked_near(self, centre_mm):
        """The arm has just picked the body at centre_mm: gone if the next look doesn't find it there."""
        for b in self.bodies:
            if self._distance(b.centre_mm, centre_mm) <= self.match_mm:
                b.picked = True

    def body(self, body_id):
        return next(b for b in self.bodies if b.id == body_id)

    def _still_where_read(self, body):
        centre, direction = body.read_at
        return (self._distance(body.centre_mm, centre) <= self.match_mm
                and _angle_apart(body.direction_deg, direction) <= self.match_deg)

    @staticmethod
    def _distance(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))
