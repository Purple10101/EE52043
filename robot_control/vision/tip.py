"""Finds the closed gripper's tip on the plate, in a raw frame, near where the arm says it should be.

The gripper is black and the plate white, so it's what is solid and dark on the plate, in one piece or
several (a light cable lying across it splits it). The camera looks at an angle: anything above the
plate lands further from the camera's foot than it is, so the end nearest the foot is the lowest
point, the tip. Face-on to the camera the closed tips make a sharp V; side-on the end is blunt, and no
point can be read from it, so a blunt end is refused.

Shape alone can't always tell the V from the servo's cable or a corner of the gripper, so only the
SEARCH_MM around the expected tip is looked at. The cable is dark too: the shape is thinned to what's
solid, which drops most of it and the V's point, then grown back just far enough to recover the point.
"""

import cv2
import numpy as np

from robot_offline.pre import rectify

from robot_control.vision.detector import camera_mm
from robot_control.vision.rig import correct_height

DARK_SHARE = 0.4            # gripper: darker than this share of the plate's level (shadows stay above it)
SEAM_MM = 1.0               # face-on, light shows through the closed jaws: a slit this wide is bridged
IGNORE_MARGIN_MM = 0.5      # around an outline to ignore: it may cut into a V right beside it, so kept small
SOLID_MM = 3.0            # thinner than this is cable, not gripper (the cable's wires are about 2 mm)
REGROW_MM = 2.0             # how far the solid shape is grown back: past the V's lost point, not along a wire
MIN_AREA_MM2 = 20.0         # less than this in all is nothing to do with the gripper
SEARCH_MM = 15.0            # around the expected tip: the nominal transform is out by 7.3 mm at worst
EDGE_CLEAR_MM = 2.0         # a tip this near the plate's edge may run off it, so its end can't be seen
APEX_MM = 0.5               # the tip's sideways position: the middle of the shape this far back from its end
WIDTH_BACK_MM = 2.5         # how sharp the end is: the shape's width this far back from its end, past any
                            # stub of cable left by REGROW_MM
MAX_WIDTH_MM = 7.0          # wider than this there, the end is blunt
MAX_LEAN = 0.15             # the V points at the camera's foot (it leans 0.04 in real frames): a corner of
                            # the jaws seen side-on leans 0.35


class TipNotFound(Exception):
    """No tip that can be read in this frame."""


def _disc(radius_px):
    d = 2 * int(round(radius_px)) + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (d, d))


def _end(points, foot, start, s):
    """The end of a shape (px, N × 2) toward foot, looking along the line from start: (apex px, width
    behind it in mm, how far the apex is off the middle of that width, as a share of the width)."""
    toward = foot - start
    toward /= np.linalg.norm(toward)
    across = np.array([-toward[1], toward[0]])
    along, sideways = points @ toward, points @ across
    end = along.max()
    behind = np.sort(sideways[np.abs(along - (end - WIDTH_BACK_MM * s)) < 1.0])
    if len(behind) == 0:
        raise TipNotFound("the dark shape ends too abruptly to be the tip")
    middle = np.median(sideways[along > end - APEX_MM * s])
    # Only the stretch the end belongs to: a stub of cable or the body may lie level with it further along.
    run = min(np.split(behind, np.nonzero(np.diff(behind) > 2.0)[0] + 1), key=lambda r: np.abs(r - middle).min())
    width = max(run[-1] - run[0], 1.0) / s
    return end * toward + middle * across, width, abs((run[0] + run[-1]) / 2 - middle) / (width * s)


def tip_in_rectified(rectified, settings, lens_mm, near_mm, ignore=()):
    """The tip in a rectified image, near scene mm near_mm on the plate surface: (scene mm on that
    surface, width behind the end in mm). ignore: outlines (rectified px) of dark things that aren't
    the gripper, the resistors. Raises TipNotFound."""
    s = settings.px_per_mm
    grey = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    plate = rectify.plate_mask(settings)
    values = grey[plate]
    level = np.median(values[values >= np.median(values)])
    dark = ((grey < DARK_SHARE * level) & plate).astype(np.uint8)
    if len(ignore):
        hidden = np.zeros_like(dark)
        cv2.fillPoly(hidden, [np.round(q).astype(np.int32) for q in ignore], 1)
        dark[cv2.dilate(hidden, _disc(IGNORE_MARGIN_MM * s)) > 0] = 0
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, _disc(SEAM_MM / 2 * s)) & plate.astype(np.uint8)
    solid = cv2.morphologyEx(dark, cv2.MORPH_OPEN, _disc(SOLID_MM / 2 * s))
    if np.count_nonzero(solid) < MIN_AREA_MM2 * s * s:
        raise TipNotFound("nothing on the plate big enough to be the gripper")

    v, u = np.nonzero(cv2.dilate(solid, _disc(REGROW_MM * s)) & dark)
    points = np.column_stack([u, v]).astype(float)
    near = rectify.mm_to_rect([near_mm], settings)[0]
    points = points[np.linalg.norm(points - near, axis=1) < SEARCH_MM * s]
    if len(points) == 0:
        raise TipNotFound("nothing dark near where the tip should be")
    foot = rectify.mm_to_rect([lens_mm[:2]], settings)[0]
    apex, _, _ = _end(points, foot, points.mean(0), s)
    apex, width, lean = _end(points, foot, apex, s)     # again, along the line the tip itself lies on
    if np.linalg.norm(apex - near) > (SEARCH_MM - 1.0) * s:
        raise TipNotFound("the gripper runs out of the search area: the tip isn't near where it should be")
    if width > MAX_WIDTH_MM:
        raise TipNotFound(f"blunt end, {width:.1f} mm wide: are the jaws face-on to the camera?")
    if lean > MAX_LEAN:
        raise TipNotFound(f"a corner, not the V (leaning {lean:.2f}): are the jaws face-on to the camera?")

    inside = cv2.erode(plate.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=int(EDGE_CLEAR_MM * s))
    if not inside[int(round(apex[1])), int(round(apex[0]))]:
        raise TipNotFound("the tip is at the plate's edge")
    return rectify.rect_to_mm([apex], settings)[0], width


def find_tip(frame, cal, near_mm, height_mm=0.0, ignore=()):
    """Scene mm (x, y) of the tip in a raw frame, height_mm above the plate, where near_mm is where it
    should be. ignore: as for tip_in_rectified. Raises TipNotFound."""
    lens = camera_mm(cal)
    z = cal.settings.top_z_mm
    seen_near = correct_height([near_mm], lens, z, -height_mm)[0]       # near enough, for a search area
    seen, _ = tip_in_rectified(rectify.rectify(frame, cal), cal.settings, lens, seen_near, ignore)
    return correct_height([seen], lens, z, height_mm)[0]
