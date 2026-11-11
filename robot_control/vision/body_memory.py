"""BodyMemory: each resistor on the plate followed from look to look, so no single look decides alone.

The sorting run looks once per pick, and a single look can slip in two ways (2026-11-11 run of 8,
experiments/2026-11-11_full_run_8):
- segmentation can miss a plainly visible body: its score swings from frame to frame, and one look
  scored it under the threshold, so the run thought the plate was empty and stopped;
- the band reader can flip: one 1k read "unsure" on one look and a confident, wrong "10 ohm" a few looks later.

robot_offline's live tools answer both, and this does the same for the robot, in scene mm:
- Like its Steady (inf/steady.py): a body is followed by where it lies (MATCH_MM). A new body needs the
  normal confidence; one already seen stays a body while it is found at all (down to FLOOR), as Steady
  keeps a body whose score swings. One found nowhere is "missing", not gone, until it has been missed
  GONE_LOOKS looks in a row; the run looks again rather than end while a body is missing.
- Like its learned.Views: each body's band-reader log-probabilities are averaged over its last WINDOW looks,
  its close-up pinned in with the same weight, and decoded once. Averaged, not summed: looks a moment apart
  are nearly the same picture, so they must not add up to more confidence than one good view gives.

A body the arm has picked is marked so (picked_near). If the next look finds nothing there, even at FLOOR,
it's gone: forgotten at once, not waited for. If anything is found there, the pick failed and it is still
that body, counted even when faint: on 2026-11-11 a failed pick left a 5k6 on the plate that segmentation
then scored too low for 11 looks, and the run, taking the empty spot for a success, stopped twice with it there.
"""

from dataclasses import dataclass

import numpy as np
from robot_offline.post.bands import learned

MATCH_MM = 3.0          # one body if its centre is this close to where it was (robot_offline closeup.MATCH_PX, 3 mm)
FLOOR = 0.05            # segmentation is run this low (robot_offline inf/steady.py FLOOR): a known body still counts
WINDOW = 3              # looks a body's reading is averaged over (its close-up as well)
GONE_LOOKS = 2          # a body missed this many looks in a row is forgotten


@dataclass
class Body:
    id: int
    centre_mm: tuple
    missed: int = 0             # looks in a row it wasn't found at all
    looks: int = 0              # looks it was found in
    picked: bool = False        # the arm has just picked it: gone if the next look doesn't find it


class BodyMemory:
    def __init__(self, min_confidence, conf=0.5, match_mm=MATCH_MM, window=WINDOW, gone=GONE_LOOKS):
        """min_confidence: the band reader's limit. conf: the segmentation score a new body needs."""
        self.conf, self.match_mm, self.gone = conf, match_mm, gone
        self.views = learned.Views(window=window, min_confidence=min_confidence)
        self.bodies = []
        self._next_id = 1

    def match(self, detections):
        """detections: [(centre_mm, score)] from one look, at FLOOR or above. Returns, in order, each one's
        body id, or None for a detection that isn't a body (a new one under conf). Bodies not found count a
        miss; those missed GONE_LOOKS in a row are forgotten."""
        order = sorted(range(len(detections)), key=lambda i: -detections[i][1])
        ids, taken = [None] * len(detections), set()
        for i in order:
            centre, score = detections[i]
            near = [(self._distance(b.centre_mm, centre), b) for b in self.bodies if b.id not in taken]
            near = [(d, b) for d, b in near if d <= self.match_mm]
            if near:
                body = min(near, key=lambda n: n[0])[1]
            elif score >= self.conf:
                body = Body(self._next_id, tuple(centre))
                self._next_id += 1
                self.bodies.append(body)
            else:
                continue                    # faint and new: not a body (yet)
            body.centre_mm, body.missed, body.picked = tuple(centre), 0, False
            body.looks += 1
            ids[i] = body.id
            taken.add(body.id)
        for body in self.bodies:
            if body.id not in taken:
                body.missed += 1
        self.bodies = [b for b in self.bodies if b.missed < self.gone and not (b.picked and b.missed)]
        self.views.keep_only([b.id for b in self.bodies])
        return ids

    def missing(self):
        """Bodies seen before but not in the last look (and not yet given up)."""
        return [b for b in self.bodies if b.missed > 0]

    def read(self, body_id, log_probs):
        """This look's view of a body's bands; returns its Reading over its recent views and close-up."""
        return self.views.add(body_id, log_probs)

    def has_closeup(self, body_id):
        return body_id in self.views.pinned

    def pin_closeup(self, body_id, log_probs):
        self.views.pin(body_id, log_probs)

    def picked_near(self, centre_mm):
        """The arm has just picked the body at centre_mm: gone if the next look doesn't find it there."""
        for b in self.bodies:
            if self._distance(b.centre_mm, centre_mm) <= self.match_mm:
                b.picked = True

    @staticmethod
    def _distance(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))
