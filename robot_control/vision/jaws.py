"""Finds gripper v5's closed jaws in a raw frame: each jaw's body corner and point, and from them the tip.

Gripper v5 (Robot Resources/gripper/v5, config/grippers/rack_pinion_v5.toml) is blue-grey, not black, so the
v4 finder (tip.py: the darkest V) can't see it. Each jaw is one piece: a body (x 0..16 mm from the
centreline, z 29.4..46 in the gripper frame) and a point below it ending in an apex 9 mm lower. The bodies
meet along the centreline, a thin dark seam on the face, which opens into a slit between the points: the
apexes don't quite touch. The jaws must face the camera (the face with the spring slots), as for v4.

 1. Gripper: its colour (not dark, a little blue, not the arm's strong blue), against the plate, the bench
    beyond the plate's edge and the arm alike.
 2. Bottom profile: for each column, the gripper's lowest pixel.
 3. Bottom edge: one straight line through the two bodies' flats (z 46), the points left out by the fit.
 4. Corners: where that line meets each body's side edge, fitted up to the bodies' top (from the CAD,
    through the camera model) where the wider cover starts.
 5. Seam: the darkest column of each row of the jaw block, near the middle, as a robust line. It must run
    along the image's vertical at the gripper and cross the bottom edge midway between the corners.
 6. Apexes: the points hang in the bodies' shadow, so for them anything not plate-coloured counts, in a strip
    along the seam below the bodies, split by the seam: in each half the pixels furthest down.
 7. To scene mm, each at its own height (apexes at the tip's, corners 9 mm higher). The tip is the apexes'
    middle: on the face toward the camera, so TIP_OFFSET_MM (6.0) from the axis, as v4's V was.
 8. Checks: the corners CORNER_SPACING_MM apart and the tip near their midpoint, or it is refused.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from robot_offline.pre import rectify

from robot_control.vision.detector import camera_mm
from robot_control.vision.rig import correct_height
from robot_control.vision.tip import TipNotFound

# Gripper v5 rev 3 CAD (all closed).
CORNER_ABOVE_TIP_MM = 9.0   # the bodies' bottom corners (z 46) above the apexes (z 55)
CORNER_SPACING_MM = 32.0    # the corners at x +/-16
FACE_MM = 6.0               # the visible face's distance from the axis (bodies and points y -6..6)
BODY_MM = 16.6              # the bodies' height (z 29.4..46): the cover above is wider
# The gripper's colour, Lab (OpenCV 0..255; b* 128 neutral, lower bluer). Measured 2026-11-18 on tour frames:
# gripper L 69-128, b* 113-126; plate b* 143-159 even in shade; the bench beyond the plate's edge L <= 53,
# b* 128-135; the blue shaft and arm b* 72-90.
L_MIN, B_MIN, B_MAX = 50, 105, 130
POINT_B_MAX = 134           # the points: anything bluer than the plate, dark or not
# Search and fitting.
ROI_MM = (-45.0, 45.0, -80.0, 25.0)   # around the expected tip: left, right, up, down (mm at the tip's scale)
MIN_AREA_PX = 2000
LINE_TOL_PX = 2.5           # a profile point this close to the bottom edge's line is on it
SIDE_NEAR_MM = 3.0          # a side edge is looked for this far either side of its corner's column
SEAM_HALF_MM = 6.0          # the seam is looked for this far either side of the bottom edge's middle
SEAM_MIN_CONTRAST = 25      # Lab L: the seam is at least this much darker than the face beside it
SEAM_MAX_TILT_DEG = 15.0
SEAM_MAX_OFF_MIDDLE = 0.12  # share of the corners' spacing
POINT_MM = 14.0             # the points reach 9 mm below the bodies: looked for this far below the edge
POINT_HALF_MM = 5.0         # ... and this far either side of the seam (each point is 3.25 mm wide)
APEX_BAND_PX = 1.5          # an apex: the mean of its point's pixels within this of the furthest down
IGNORE_MARGIN_MM = 0.5
# Checks: measured on 7 frames over the plate (2026-11-18), corners 31.4-32.0 mm apart and the tip 0.6-0.75
# mm from their midpoint (away from the camera, every time).
SPACING_TOL_MM = 1.5
TIP_OFF_MID_MAX_MM = 2.0


class JawsNotFound(TipNotFound):
    """No closed v5 jaws that can be read in this frame."""


@dataclass
class Jaws:
    corners_px: np.ndarray      # 2 x 2 raw px: left, right in the image
    apexes_px: np.ndarray       # 2 x 2 raw px
    corners_mm: np.ndarray      # 2 x 2 scene mm, at their height
    apexes_mm: np.ndarray
    tip_mm: np.ndarray          # the apexes' middle, scene mm at the tip's height
    centre_mm: np.ndarray       # the axis, from the corners: FACE_MM beyond their midpoint, away from the camera
    jaw_deg: float              # the line the jaws close along, scene deg from +x, 0..180
    spacing_mm: float           # the corners' distance apart
    detail: dict                # for drawing: edge and seam lines, the jaws' masks and the search region


def scene_to_px(points_mm, height_mm, cal):
    """Scene (x, y) of points height_mm above the plate to raw frame px."""
    lens = camera_mm(cal)
    p = np.asarray(points_mm, float).reshape(-1, 2)
    foot = np.asarray(lens[:2], float)
    above = lens[2] - cal.settings.top_z_mm
    surface = foot + (p - foot) * above / (above - height_mm)
    return rectify.to_full(rectify.mm_to_rect(surface, cal.settings), cal)


def px_to_scene(points_px, height_mm, cal):
    """Raw frame px of points height_mm above the plate to their scene (x, y)."""
    surface = rectify.to_scene_mm(np.asarray(points_px, float).reshape(-1, 2), cal)
    return correct_height(surface, camera_mm(cal), cal.settings.top_z_mm, height_mm)


def _image_down(px, height_mm, cal):
    """Unit image direction of straight down at raw px, for something height_mm up."""
    xy = px_to_scene([px], height_mm, cal)
    a, b = scene_to_px(xy, height_mm, cal)[0], scene_to_px(xy, height_mm - 5.0, cal)[0]
    return (b - a) / np.linalg.norm(b - a)


def _fit_line(pts, tol, iters=8):
    """Robust total-least-squares line through pts (N x 2): (point, unit direction)."""
    pts = np.asarray(pts, float)
    keep = np.ones(len(pts), bool)
    for _ in range(iters):
        c = pts[keep].mean(0)
        d = np.linalg.svd(pts[keep] - c)[2][0]
        r = np.abs((pts - c) @ np.array([-d[1], d[0]]))
        new = r < max(tol, 2.5 * np.median(r[keep]))
        if new.sum() < 3 or (new == keep).all():
            break
        keep = new
    return c, d


def _intersect(c1, d1, c2, d2):
    t = np.linalg.solve(np.array([d1, -d2]).T, c2 - c1)
    return c1 + t[0] * d1


def _gripper(lab):
    """The gripper's pixels in a Lab region: its largest piece of the right colour, the seam bridged."""
    L, b = lab[:, :, 0], lab[:, :, 2]
    mask = ((L > L_MIN) & (b > B_MIN) & (b < B_MAX)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        raise JawsNotFound("nothing the gripper's colour near where the tip should be")
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[best, cv2.CC_STAT_AREA] < MIN_AREA_PX:
        raise JawsNotFound("nothing big enough to be the gripper near where the tip should be")
    return labels == best


def _bottom_edge(blob):
    """The profile (column, lowest row), the bottom edge's line (point, direction left to right, normal down)
    and the profile points at its two ends."""
    cols = np.nonzero(blob.any(0))[0]
    profile = np.array([(c, np.nonzero(blob[:, c])[0].max()) for c in cols], float)
    c, d = _fit_line(profile[profile[:, 1] > np.percentile(profile[:, 1], 40)], LINE_TOL_PX)
    d = d if d[0] > 0 else -d
    n = np.array([-d[1], d[0]])
    n = n if n[1] > 0 else -n
    on = np.nonzero(np.abs((profile - c) @ n) < LINE_TOL_PX)[0]
    if len(on) < 10:
        raise JawsNotFound("no straight bottom edge to the jaws")
    return profile, c, d, n, profile[on.min()], profile[on.max()]


def _side_edge(blob, end, side, top_row, px_mm):
    """A body's side edge above an end of the bottom edge: the outermost pixel near the end's column in each row
    up to just below top_row, the bodies' top."""
    x_end, row = end
    pts = []
    for r in range(int(row - 1.0 * px_mm), int(top_row + 2.0 * px_mm), -1):
        xs = np.nonzero(blob[r])[0]
        xs = xs[np.abs(xs - x_end) < SIDE_NEAR_MM * px_mm]
        if len(xs):
            pts.append((xs.min() if side == "left" else xs.max(), r))
    if len(pts) < 5:
        raise JawsNotFound(f"no side edge to the {side} jaw's body: are the jaws face-on to the camera?")
    return np.array(pts, float)


def _seam(L, blob, n, corners, top_row, bottom_row, px_mm):
    """The dark line between the jaws: (point, direction down the image)."""
    mid = corners.mean(0)
    pts = []
    for r in range(int(top_row) + 3, int(bottom_row) - 1):
        cx = mid[0] + (r - mid[1]) / n[1] * n[0]
        x0, x1 = int(cx - SEAM_HALF_MM * px_mm), int(cx + SEAM_HALF_MM * px_mm)
        if x0 < 0 or x1 > L.shape[1] or blob[r, x0:x1].sum() < 0.8 * (x1 - x0):
            continue
        row = L[r, x0:x1]
        j = int(np.argmin(row))
        if np.median(row) - row[j] >= SEAM_MIN_CONTRAST:
            pts.append((x0 + j, r))
    if len(pts) < 10:
        raise JawsNotFound("no seam between the jaws: are they closed and face-on to the camera?")
    sc, sd = _fit_line(pts, 1.5)
    return sc, (sd if sd @ n > 0 else -sd), np.array(pts, float)


def find_jaws(frame, cal, near_mm, height_mm=0.0, ignore=()):
    """The closed jaws in a raw frame, the tip height_mm above the plate and near scene mm near_mm. ignore:
    outlines (rectified px) of things that aren't the gripper, the resistors. Raises JawsNotFound."""
    (tx, ty), = scene_to_px([near_mm], height_mm, cal)
    (ax, ay), = scene_to_px([np.asarray(near_mm, float) + [1.0, 0.0]], height_mm, cal)
    px_mm = float(np.hypot(ax - tx, ay - ty))
    x0, y0 = max(int(tx + ROI_MM[0] * px_mm), 0), max(int(ty + ROI_MM[2] * px_mm), 0)
    x1, y1 = min(int(tx + ROI_MM[1] * px_mm), frame.shape[1]), min(int(ty + ROI_MM[3] * px_mm), frame.shape[0])
    if x1 - x0 < 20 * px_mm or y1 - y0 < 20 * px_mm:
        raise JawsNotFound("where the tip should be is out of the camera's view")
    off = np.array([x0, y0], float)
    lab = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2LAB)
    if len(ignore):
        hidden = np.zeros(lab.shape[:2], np.uint8)
        for q in ignore:
            poly = rectify.to_full(np.asarray(q, float).reshape(-1, 2), cal) - off
            cv2.fillPoly(hidden, [np.round(poly).astype(np.int32)], 1)
        k = 2 * int(IGNORE_MARGIN_MM * px_mm) + 1
        hidden = cv2.dilate(hidden, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))) > 0
        lab[hidden] = (255, 128, 160)       # plate-coloured: neither gripper nor point
    blob = _gripper(lab)
    profile, c, d, n, left_end, right_end = _bottom_edge(blob)

    corner_h = height_mm + CORNER_ABOVE_TIP_MM
    corners, tops = [], []
    for side, end in (("left", left_end), ("right", right_end)):
        top = scene_to_px(px_to_scene([end + off], corner_h, cal), corner_h + BODY_MM, cal)[0] - off
        sc_, sd_ = _fit_line(_side_edge(blob, end, side, top[1], px_mm), 1.5)
        corners.append(_intersect(c, d, sc_, sd_))
        tops.append(top[1])
    corners = np.array(corners)
    top_row = max(tops) + 1.0 * px_mm

    L = lab[:, :, 0].astype(float)
    sc, sd, seam_pts = _seam(L, blob, n, corners, top_row, corners[:, 1].min(), px_mm)
    down = _image_down(np.array([tx, ty]), height_mm, cal)
    if abs(sd @ down) < np.cos(np.radians(SEAM_MAX_TILT_DEG)):
        raise JawsNotFound(f"the seam found leans {np.degrees(np.arccos(min(abs(sd @ down), 1.0))):.0f} deg "
                           "off the vertical")
    share = (_intersect(c, d, sc, sd) - corners[0]) @ (corners[1] - corners[0]) / np.sum((corners[1] - corners[0]) ** 2)
    if abs(share - 0.5) > SEAM_MAX_OFF_MIDDLE:
        raise JawsNotFound(f"the seam meets the bottom edge {share:.0%} of the way along, not in the middle")

    rows, cols = np.mgrid[0:blob.shape[0], 0:blob.shape[1]]
    pix = np.stack([cols, rows], -1).astype(float)
    across = (pix - sc) @ np.array([sd[1], -sd[0]])
    below = (pix - c) @ n
    point = (lab[:, :, 2] < POINT_B_MAX) & (below > LINE_TOL_PX) & (below < POINT_MM * px_mm) \
        & (np.abs(across) < POINT_HALF_MM * px_mm)
    point = cv2.morphologyEx(point.astype(np.uint8), cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))) > 0
    apexes = []
    for half in (point & (across < -0.5), point & (across > 0.5)):
        p = pix[half]
        if len(p) < 20:
            raise JawsNotFound("a jaw's point isn't below its body: are the jaws closed?")
        depth = p @ down
        apexes.append(p[depth > depth.max() - APEX_BAND_PX].mean(0))
    apexes = np.array(apexes) + off
    corners = corners + off

    corners_mm = px_to_scene(corners, corner_h, cal)
    apexes_mm = px_to_scene(apexes, height_mm, cal)
    v = corners_mm[1] - corners_mm[0]
    spacing = float(np.hypot(*v))
    mid = corners_mm.mean(0)
    away = np.array([-v[1], v[0]]) / spacing
    if (np.asarray(camera_mm(cal)[:2], float) - mid) @ away > 0:
        away = -away
    tip = apexes_mm.mean(0)
    if abs(spacing - CORNER_SPACING_MM) > SPACING_TOL_MM:
        raise JawsNotFound(f"the corners are {spacing:.1f} mm apart, not {CORNER_SPACING_MM:g}: not both corners "
                           "of the closed jaws")
    if np.hypot(*(tip - mid)) > TIP_OFF_MID_MAX_MM:
        raise JawsNotFound(f"the tip is {np.hypot(*(tip - mid)):.1f} mm from the corners' midpoint: a point or a "
                           "corner was misread")
    block = blob & (rows > top_row)
    detail = {"roi": (x0, y0, x1, y1), "edge": (c + off, d), "seam": (sc + off, sd), "seam_pts": seam_pts + off,
              "left_jaw": block & (across < -0.5), "right_jaw": block & (across > 0.5), "px_mm": px_mm}
    return Jaws(corners, apexes, corners_mm, apexes_mm, tip, mid + FACE_MM * away,
                float(np.degrees(np.arctan2(v[1], v[0]))) % 180.0, spacing, detail)


def draw(frame, jaws, cal, height_mm=0.0):
    """The search region of frame with the jaws tinted (left red, right green), the bottom edge and seam, the
    corners (red), apexes (magenta) and the axis (green), for looking at."""
    x0, y0, x1, y1 = jaws.detail["roi"]
    out = frame[y0:y1, x0:x1].copy()
    tint = out.copy()
    tint[jaws.detail["left_jaw"]] = (0, 0, 255)
    tint[jaws.detail["right_jaw"]] = (0, 200, 0)
    out = cv2.addWeighted(tint, 0.35, out, 0.65, 0)
    off = np.array([x0, y0], float)

    def at(q):
        return tuple(np.round(np.asarray(q, float) - off).astype(int))
    for (c, d), colour, reach in ((jaws.detail["edge"], (0, 0, 255), 700), (jaws.detail["seam"], (0, 255, 255), 400)):
        cv2.line(out, at(c - reach * d), at(c + reach * d), colour, 1, cv2.LINE_AA)
    for q in jaws.corners_px:
        cv2.drawMarker(out, at(q), (0, 0, 255), cv2.MARKER_CROSS, 30, 2, cv2.LINE_AA)
    for q in jaws.apexes_px:
        cv2.drawMarker(out, at(q), (255, 0, 255), cv2.MARKER_CROSS, 20, 2, cv2.LINE_AA)
    cv2.drawMarker(out, at(scene_to_px([jaws.centre_mm], height_mm, cal)[0]), (0, 255, 0),
                   cv2.MARKER_TILTED_CROSS, 30, 2, cv2.LINE_AA)
    return out
