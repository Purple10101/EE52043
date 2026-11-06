"""Finds the plate in a live frame from nothing: no saved fit, and resistors on it are fine.

robot_offline's own first search starts from the frame's largest bright blob, which on this rig misses a
corner of the plate by hundreds of pixels (its light falls off across the top), and its edge search then
looks in the wrong place. Long straight lines are a surer start: they pair up into four-sided outlines of
about the plate's size, and each outline is tried by robot_offline's real fit, which only passes the plate.
"""

import itertools
import threading
import time

import cv2
import numpy as np
from robot_offline.pre import rectify

SCALE = 0.25                # lines are looked for at this share of the frame's size
CANNY = (20, 60)
HOUGH_VOTES = 60
MIN_LINE_PX = 120           # at SCALE: about a third of the plate's side
MAX_GAP_PX = 25             # at SCALE: a lead lying over an edge breaks it by less than this
SAME_ANGLE_RAD = 0.05       # lines this alike, and SAME_OFFSET_PX apart, are one line
SAME_OFFSET_PX = 25
LEAN_RAD = np.radians(35)   # a side lies within this of the frame's axes
MAX_LINES = 14              # of each kind, longest first
SIDE_PX = (800, 2200)       # the plate's sides in a raw frame are about 1200 to 1600
MAX_SIDE_RATIO = 1.6


def _lines(frame):
    """Long straight lines in a frame as (angle, offset, length), raw px, longest first and without repeats."""
    small = cv2.resize(frame, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
    gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (0, 0), 1.5)
    found = cv2.HoughLinesP(cv2.Canny(gray, *CANNY), 1, np.pi / 360, threshold=HOUGH_VOTES,
                            minLineLength=MIN_LINE_PX, maxLineGap=MAX_GAP_PX)
    if found is None:
        return []
    keep = []
    segments = found.reshape(-1, 4) / SCALE
    for x0, y0, x1, y1 in sorted(segments.tolist(), key=lambda s: -np.hypot(s[2] - s[0], s[3] - s[1])):
        angle = np.arctan2(y1 - y0, x1 - x0) % np.pi
        offset = np.array([-np.sin(angle), np.cos(angle)]) @ (x0, y0)
        if not any(min(abs(angle - a), np.pi - abs(angle - a)) < SAME_ANGLE_RAD and abs(offset - d) < SAME_OFFSET_PX
                   for a, d in keep):
            keep.append((angle, offset))
    return keep


def _crossing(p, q):
    normals = [[-np.sin(p[0]), np.cos(p[0])], [-np.sin(q[0]), np.cos(q[0])]]
    return np.linalg.solve(np.array(normals), np.array([p[1], q[1]]))


def outlines(frame):
    """Four-sided outlines (4 x 2 raw px, image order) the plate could be, largest first."""
    lines = _lines(frame)
    across = [n for n in lines if min(n[0], np.pi - n[0]) < LEAN_RAD][:MAX_LINES]
    down = [n for n in lines if abs(n[0] - np.pi / 2) < LEAN_RAD][:MAX_LINES]
    quads = []
    for (top, bottom), (left, right) in itertools.product(itertools.combinations(across, 2),
                                                          itertools.combinations(down, 2)):
        try:
            corners = [_crossing(top, left), _crossing(top, right), _crossing(bottom, right), _crossing(bottom, left)]
        except np.linalg.LinAlgError:
            continue
        quad = rectify._image_order(np.array(corners))
        sides = np.linalg.norm(quad - np.roll(quad, -1, 0), axis=1)
        if SIDE_PX[0] <= sides.min() and sides.max() <= SIDE_PX[1] and sides.max() / sides.min() <= MAX_SIDE_RATIO:
            quads.append(quad)
    return sorted(quads, key=lambda q: -cv2.contourArea(q.astype(np.float32)))


def find(frame, settings):
    """The plate's Calibration from one frame, from nothing. Raises rectify.PlateNotFound."""
    quads = outlines(frame)
    for quad in quads:
        try:
            fit = rectify.find_plate(frame, settings, start=quad)
        except rectify.PlateNotFound:
            continue
        return rectify._calibration(fit, (frame.shape[1], frame.shape[0]), settings)
    raise rectify.PlateNotFound(f"none of {len(quads)} outlines from the frame's straight lines fitted the plate")


class Finder(threading.Thread):
    """Finds the plate once, in the background, from the newest frame. .again looks for it afresh, keeping
    the last fit until the new one is found."""

    def __init__(self, settings):
        super().__init__(daemon=True)
        self.settings = settings
        self.frame = None           # the newest frame, set by the display loop
        self.cal = None
        self.error = None           # why the latest try failed, or None
        self.again = True
        self.stop = False

    def run(self):
        done = None
        while not self.stop:
            frame = self.frame
            if not self.again or frame is None or frame is done:
                time.sleep(0.02)
                continue
            done = frame
            try:
                cal = find(frame, self.settings)
                rectify.rectify(frame, cal)             # builds its remap tables here, not in the display loop
                self.cal, self.error, self.again = cal, None, False
            except rectify.PlateNotFound as e:
                self.error = str(e)
