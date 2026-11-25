"""Finds gripper v5's closed jaws in a raw frame: each jaw's body corner and point, and from them the tip.

Gripper v5 (Robot Resources/gripper/v5, config/grippers/rack_pinion_v5.toml) is dark blue, not black, so the
v4 finder (tip.py: the darkest V) can't see it. Each jaw is one piece: a body (x 0..16 mm from the
centreline, z 29.4..46 in the gripper frame) and a point below it ending in an apex 9 mm lower. The bodies
meet along the centreline, a thin dark seam on the face, which opens into a slit between the points: the
apexes don't quite touch. The jaws must face the camera (the face with the spring slots), as for v4.

 1. Gripper: given a reference (the plate with the arm away, shortly before), what changed from it in colour, or
    far more in brightness than a shadow does: whatever the light. Without one, or if that fails, what isn't plate,
    by colour (the plate's measured in the same frame): bluer than the plate or much darker. Either way over the
    plate's surface (the plate's outline from the calibration). Off
    the plate the bench can look just like the gripper, so only the plate's surface is used: the jaws' bottom
    few mm must be seen against it, which the camera's angle allows over most of the plate but not at its far
    (-y) edge or its +x edge (camera_calibrate.POINTS keeps clear of them).
 2. Bottom profile: for each column, the gripper's lowest pixel.
 3. Bottom edge: one straight line through the two bodies' flats (z 46): the profile's two longest flat runs near
    its lowest (near the camera the housing's underside shows as a step higher up).
 4. Corners: where that line meets each body's side edge. The sides are upright, so each runs along the image's
    down at it; only where it lies is measured, from the rows up to the bodies' top (from the CAD, through the
    camera model) that have the plate beyond them, which may be only a few.
 5. Seam: the darkest column of each row of the jaw block, near the middle, as a robust line, if it runs along
    the image's vertical and crosses the bottom edge midway between the corners. Otherwise (near the camera it is
    short and can show lighter than the face) the upright line midway between the corners, from the camera model.
 6. Apexes: the gripper's pixels in a strip along the seam below the bodies, split by the seam: in each half
    the pixels furthest down.
 7. To scene mm, each at its own height (apexes at the tip's, corners 9 mm higher). The tip is the apexes'
    middle: on the face toward the camera, so TIP_OFFSET_MM (6.0) from the axis, as v4's V was.
 8. Checks: the corners CORNER_SPACING_MM apart, both apexes at the seam and the tip near the corners' midpoint,
    or it is refused.
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
# The gripper is found as what isn't plate, over the plate, so it doesn't matter how it is lit: Lab (OpenCV
# 0..255; b* 128 neutral, lower bluer), against the plate's own colour in the same frame (white balance and
# exposure drift). Measured on tour frames: under a warm lamp (2026-11-18) the gripper L 58-125 against the
# plate's 150-185, b* 113-126 against 143-153; in daylight (2026-11-23) the gripper's face is in shade, L
# 20-45 against 180-205, b* 109-127 against 133-151. Beyond the plate's far edge the black bench is the
# gripper's colour in daylight (L 20, b* 117), so only the plate's surface is used. The arm's blue link
# (a* about 150, b* 83-92) is left out whatever it lies over.
# Not plate: b* under a cut PLATE_B_SHARE of the way from neutral (128) to the plate's b*, and at least
# PLATE_B_MIN_DROP under the plate's, or L under PLATE_L_SHARE of the plate's. The gripper is blue, under
# neutral; the plate is yellow, its shadow too though less so (under the lamp the gripper's shadow on the plate
# was b* 139-145 to the lit plate's 150-153, L 110-150 to 155-190), so a cut near neutral keeps the shadow out.
PLATE_B_SHARE = 0.3
PLATE_B_MIN_DROP = 4
PLATE_L_SHARE = 0.3
LINK_A_MIN = 140            # a*: the arm's blue link, neither plate nor gripper
PLATE_INSET_MM = 1.0        # the plate's outline shrunk by this, its rounded corners followed: clear of its edges
PLATE_REF_SHARE = 0.3       # the plate's colour: the median of this brightest share of its surface in view
MIN_PLATE_PX = 2000         # with less of the plate in view than this, it can't be measured
# With a reference (the same view without the arm, taken shortly before), the gripper is what changed instead:
# independent of the light's colour and direction, as long as it holds between the two pictures. Shadows only
# darken the plate; the gripper changes its colour (dark blue on the plate, whatever lights both).
CHANGE_CHROMA = 0.09        # chromaticity distance (r, g, b each a share of their sum): a colour change
                            # (shadows shift it too, lit by the other lights: 0.03-0.06; the gripper 0.1+)
CHANGE_DARK = 0.25          # or this share of the reference's brightness or less: darker than the jaws' shadow
                            # (the navy gripper on the plate in daylight changes brightness more than colour)
CHANGE_BRIGHT = 1.8         # or this many times brighter: the screws' glints
GAIN_BELOW_TIP_MM = 5.0     # the light's drift measured on the plate this far nearer the camera than the tip
# Search and fitting.
ROI_MM = (-45.0, 45.0, -80.0, 25.0)   # around the expected tip: left, right, up, down (mm at the tip's scale)
MIN_AREA_PX = 2000
LINE_TOL_PX = 2.5           # a profile point this close to the bottom edge's line is on it
FLAT_SMOOTH_MM = 1.0        # the bottom profile is smoothed over this before its slope is taken
FLAT_MAX_SLOPE = 0.25       # a flat: the profile's slope under this (the points' sides are steeper)
FLAT_MIN_MM = 3.0           # and this long (each body's bottom is up to 12 mm, beside its point)
FLAT_LEVEL_MM = 4.0         # and no further than this above the lowest flat (the bodies' bottoms, tilted in the
                            # image by up to about 2.5 mm across; the housing's step above them is 8 mm or more)
PAST_PLATE_MM = 1.0         # a side edge counts only where the plate is this far beyond it
SIDE_MIN_ROWS = 3           # and is found in at least this many rows
SIDE_NEAR_MM = 3.0          # a side edge is looked for this far either side of its corner's column
SEAM_HALF_MM = 6.0          # the seam is looked for this far either side of the bottom edge's middle
SEAM_MIN_CONTRAST = 6       # Lab L: the seam is at least this much darker than the face beside it ...
SEAM_MIN_SHARE = 0.25       # ... and this share of the face's L (the face is L 20-45 in shade, 60-125 lit)
SEAM_MAX_TILT_DEG = 15.0
SEAM_MAX_OFF_MIDDLE = 0.12  # share of the corners' spacing
POINT_MM = 14.0             # the points reach 9 mm below the bodies: looked for this far below the edge
POINT_HALF_MM = 5.0         # ... and this far either side of the seam (each point is 3.25 mm wide)
APEX_BAND_PX = 1.5          # an apex: the mean of its point's pixels within this of the furthest down
APEX_OFF_SEAM_MM = 2.2      # each apex this close to the seam at most (the closed points meet on it; 0.7-1.7 seen)
IGNORE_MARGIN_MM = 0.5
# Checks: on 17 tour frames over the plate in both lights (2026-11-18 and 23), corners 31.5-33.3 mm apart and
# the tip 0.33-0.74 mm from their midpoint (away from the camera, every time; 1.6 once at h 1 mm).
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


def _plate_surface(cal, off, shape):
    """Where the plate's top is in a region of the raw frame (off: its top-left), PLATE_INSET_MM inside its
    edges, the rounded corners followed: a boolean mask."""
    s = cal.settings
    (x0, y0), (x1, y1) = mm_to_rect_box(s)
    inset, r = PLATE_INSET_MM * s.px_per_mm, (s.corner_radius_mm + PLATE_INSET_MM) * s.px_per_mm
    x0, y0, x1, y1 = x0 + inset, y0 + inset, x1 - inset, y1 - inset
    r = min(r, (x1 - x0) / 2, (y1 - y0) / 2)
    outline = []
    for cx, cy, start in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        t = np.radians(np.linspace(start, start + 90, 12))
        outline.append(np.stack([cx + r * np.cos(t), cy + r * np.sin(t)], 1))
    poly = rectify.to_full(np.concatenate(outline), cal) - off
    mask = np.zeros(shape, np.uint8)
    cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 1)
    return mask > 0


def mm_to_rect_box(settings):
    """The plate's top as an upright box in rectified px: ((left, top), (right, bottom))."""
    c = rectify.mm_to_rect(settings.plate_corners_mm, settings)
    return c.min(0), c.max(0)


def _not_plate(lab, plate, b_share, l_share):
    """Pixels that aren't the plate's colour, the plate's colour measured on its surface (plate: mask) in view,
    and the arm's blue link left out: a boolean mask."""
    L, a, b = (lab[:, :, i].astype(float) for i in range(3))
    if plate.sum() < MIN_PLATE_PX:
        raise JawsNotFound("too little of the plate in view round where the tip should be")
    lit = L[plate] >= np.quantile(L[plate], 1.0 - PLATE_REF_SHARE)
    plate_L, plate_b = np.median(L[plate][lit]), np.median(b[plate][lit])
    b_cut = min(128.0 + b_share * (plate_b - 128.0), plate_b - PLATE_B_MIN_DROP)
    return ((b < b_cut) | (L < l_share * plate_L)) & (a < LINK_A_MIN)


def _gain(frame, reference, cal, tip_row, px_mm):
    """Per channel, how much brighter the frame is than the reference on the plate: the exposure and white
    balance's drift, and the light's. Measured on the plate nearer the camera than the tip, where no part of the
    arm can be (further up, near the camera, the arm can cover most of the plate)."""
    whole = _plate_surface(cal, np.zeros(2), frame.shape[:2])
    whole[: max(int(tip_row + GAIN_BELOW_TIP_MM * px_mm), 0)] = False
    whole = whole[::4, ::4]
    if whole.sum() < MIN_PLATE_PX / 16:
        whole = _plate_surface(cal, np.zeros(2), frame.shape[:2])[::4, ::4]
    ratio = (frame[::4, ::4].astype(np.float32) + 1.0) / (reference[::4, ::4].astype(np.float32) + 1.0)
    return np.median(ratio[whole], axis=0)


def _changed(frame, reference, gain):
    """Pixels changed from the reference (both BGR, the same region) as an object changes them, not as a shadow
    does: their colour (chromaticity) moved by CHANGE_CHROMA or more, or much darker or brighter than a shadow
    or the light's drift would make them. A boolean mask."""
    f = frame.astype(np.float32) / gain + 1.0
    r = reference.astype(np.float32) + 1.0
    fs, rs = f.sum(2), r.sum(2)
    chroma = np.linalg.norm(f / fs[..., None] - r / rs[..., None], axis=2)
    k = fs / rs
    return (chroma > CHANGE_CHROMA) | (k < CHANGE_DARK) | (k > CHANGE_BRIGHT)


def _gripper(lab, plate, near_px, changed=None):
    """The gripper's pixels over the plate's surface (plate: mask): the piece of what isn't plate (or, given a
    reference, of what changed from it: changed) nearest near_px (the tip expected, region px), the seam bridged."""
    found = _not_plate(lab, plate, PLATE_B_SHARE, PLATE_L_SHARE) if changed is None else changed
    mask = (found & plate).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    big = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= MIN_AREA_PX]
    if not big:
        raise JawsNotFound("nothing big enough to be the gripper over the plate near where the tip should be")
    rows, cols = np.nonzero(np.isin(labels, big))
    d = np.hypot(cols - near_px[0], rows - near_px[1])
    nearest = np.full(n, np.inf)
    np.minimum.at(nearest, labels[rows, cols], d)
    return labels == int(np.argmin(nearest))


def _flats(profile, px_mm):
    """Index runs of the profile that are flat (the bodies' bottoms, not the points' sides), longest first."""
    y = profile[:, 1]
    w = max(int(round(FLAT_SMOOTH_MM * px_mm)) // 2 * 2 + 1, 3)
    smooth = cv2.blur(y.reshape(1, -1), (w, 1), borderType=cv2.BORDER_REPLICATE).ravel()
    flat = np.abs(np.gradient(smooth, profile[:, 0])) < FLAT_MAX_SLOPE
    runs, run = [], []
    for i in np.nonzero(flat)[0]:
        if run and (profile[i, 0] - profile[run[-1], 0] > 2 or abs(y[i] - y[run[-1]]) > 2):
            runs.append(run)
            run = []
        run.append(i)
    if run:
        runs.append(run)
    return sorted((r for r in runs if len(r) >= FLAT_MIN_MM * px_mm), key=len, reverse=True)


def _bottom_edge(blob, px_mm):
    """The profile (column, lowest row), the bottom edge's line (point, direction left to right, normal down)
    and the profile points at its two ends. The line is fitted to the two longest flats: with only the bodies'
    bottom few mm over the plate, the points' sides are as much of the profile as the flats are."""
    cols = np.nonzero(blob.any(0))[0]
    profile = np.array([(c, np.nonzero(blob[:, c])[0].max()) for c in cols], float)
    flats = _flats(profile, px_mm)
    if not flats:
        raise JawsNotFound("no flat bottom to the jaws' bodies")
    # Near the camera the housing's underside shows as a step higher up: only the flats near the lowest are the bodies'.
    lowest = max(profile[r, 1].mean() for r in flats)
    flats = [r for r in flats if profile[r, 1].mean() > lowest - FLAT_LEVEL_MM * px_mm][:2]
    c, d = _fit_line(profile[np.concatenate(flats)], LINE_TOL_PX)
    d = d if d[0] > 0 else -d
    n = np.array([-d[1], d[0]])
    n = n if n[1] > 0 else -n
    on = np.nonzero(np.abs((profile - c) @ n) < LINE_TOL_PX)[0]
    if len(on) < 10:
        raise JawsNotFound("no straight bottom edge to the jaws")
    return profile, c, d, n, profile[on.min()], profile[on.max()]


def _side_edge(blob, plate, end, side, top_row, px_mm):
    """A body's side edge above an end of the bottom edge: the outermost pixel near the end's column in each row
    up to just below top_row, the bodies' top, where the plate is beyond it (else it may be the plate's edge)."""
    x_end, row = end
    beyond = int(round(PAST_PLATE_MM * px_mm)) * (-1 if side == "left" else 1)
    pts, cut = [], 0
    for r in range(int(row - 1.0 * px_mm), max(int(top_row + 2.0 * px_mm), -1), -1):
        xs = np.nonzero(blob[r])[0]
        xs = xs[np.abs(xs - x_end) < SIDE_NEAR_MM * px_mm]
        if len(xs):
            x = xs.min() if side == "left" else xs.max()
            if 0 <= x + beyond < plate.shape[1] and plate[r, x + beyond]:
                pts.append((x, r))
            else:
                cut += 1
    if len(pts) < SIDE_MIN_ROWS:
        if cut >= SIDE_MIN_ROWS:
            raise JawsNotFound(f"the {side} jaw's body reaches past the plate's edge in the camera's view")
        raise JawsNotFound(f"no side edge to the {side} jaw's body: are the jaws face-on to the camera?")
    return np.array(pts, float)


def _seam(L, face, n, corners, top_row, bottom_row, px_mm):
    """The dark line between the jaws, looked for where face (mask) covers the row: (point, direction down the
    image)."""
    mid = corners.mean(0)
    pts = []
    for r in range(max(int(top_row) + 3, 0), min(int(bottom_row) - 1, L.shape[0])):
        cx = mid[0] + (r - mid[1]) / n[1] * n[0]
        x0, x1 = int(cx - SEAM_HALF_MM * px_mm), int(cx + SEAM_HALF_MM * px_mm)
        if x0 < 0 or x1 > L.shape[1] or face[r, x0:x1].sum() < 0.8 * (x1 - x0):
            continue
        row = L[r, x0:x1]
        j = int(np.argmin(row))
        level = np.median(row)
        if level - row[j] >= max(SEAM_MIN_CONTRAST, SEAM_MIN_SHARE * level):
            pts.append((x0 + j, r))
    if len(pts) < 10:
        raise JawsNotFound("no seam between the jaws: are they closed and face-on to the camera?")
    sc, sd = _fit_line(pts, 1.5)
    return sc, (sd if sd @ n > 0 else -sd), np.array(pts, float)


def find_jaws(frame, cal, near_mm, height_mm=0.0, ignore=(), reference=None):
    """The closed jaws in a raw frame, the tip height_mm above the plate and near scene mm near_mm. ignore:
    outlines (rectified px) of things that aren't the gripper, the resistors. reference: the same view with the
    arm away (raw frame, same camera and plate fit), taken shortly before: the gripper is then found as what
    changed from it, whatever the light, and by its colour only if that fails. On 41 tour frames in three lights
    (lamp, daylight, dusk) either way alone read 30, the two together 36 (the rest at the plate's edges), and
    where both read they agreed within 0.46 mm. Raises JawsNotFound."""
    if reference is None:
        return _find_jaws(frame, cal, near_mm, height_mm, ignore, None)
    try:
        return _find_jaws(frame, cal, near_mm, height_mm, ignore, reference)
    except JawsNotFound as by_change:
        try:
            return _find_jaws(frame, cal, near_mm, height_mm, ignore, None)
        except JawsNotFound as by_colour:
            raise JawsNotFound(f"{by_change} (by its colour: {by_colour})") from None


def _find_jaws(frame, cal, near_mm, height_mm, ignore, reference):
    """find_jaws one way: against the reference if one is given, else by colour."""
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
    plate = _plate_surface(cal, off, lab.shape[:2])
    changed = None
    if reference is not None:
        changed = _changed(frame[y0:y1, x0:x1], reference[y0:y1, x0:x1], _gain(frame, reference, cal, ty, px_mm))
        if len(ignore):
            changed &= ~hidden
    blob = _gripper(lab, plate, (tx - x0, ty - y0), changed)
    profile, c, d, n, left_end, right_end = _bottom_edge(blob, px_mm)

    corner_h = height_mm + CORNER_ABOVE_TIP_MM
    corners, tops = [], []
    for side, end in (("left", left_end), ("right", right_end)):
        top = scene_to_px(px_to_scene([end + off], corner_h, cal), corner_h + BODY_MM, cal)[0] - off
        # The bodies' sides are upright, so each side edge runs along the image's down at it: only where it lies
        # is measured, which a few rows will do when the plate's far edge leaves little of it over the plate.
        side_down = _image_down(end + off, corner_h, cal)
        across_side = np.array([-side_down[1], side_down[0]])
        pts = _side_edge(blob, plate, end, side, top[1], px_mm)
        corners.append(_intersect(c, d, end + np.median((pts - end) @ across_side) * across_side, side_down))
        tops.append(top[1])
    corners = np.array(corners)
    top_row = max(tops) + 1.0 * px_mm

    L = lab[:, :, 0].astype(float)
    # The seam's rows may run up past the plate's far edge, where the bench is behind them: not plate is enough.
    face = _not_plate(lab, plate, PLATE_B_SHARE, PLATE_L_SHARE)
    if changed is not None:
        face |= changed
    down = _image_down(np.array([tx, ty]), height_mm, cal)
    try:
        sc, sd, seam_pts = _seam(L, face, n, corners, top_row, corners[:, 1].min(), px_mm)
        share = (_intersect(c, d, sc, sd) - corners[0]) @ (corners[1] - corners[0]) / np.sum((corners[1] - corners[0]) ** 2)
        measured = abs(sd @ down) >= np.cos(np.radians(SEAM_MAX_TILT_DEG)) and abs(share - 0.5) <= SEAM_MAX_OFF_MIDDLE
    except JawsNotFound:
        measured = False
    if not measured:
        # Near the camera the seam below the screw is short and can show lighter than the face (the plate through
        # the gap), so it isn't always found. It is upright, midway between the corners: the camera model gives it,
        # and the points must then meet on it (APEX_OFF_SEAM_MM, below), which wrong corners wouldn't.
        sc = corners.mean(0)
        sd = _image_down(sc + off, corner_h, cal)
        sd, seam_pts = (sd if sd @ n > 0 else -sd), np.zeros((0, 2))

    rows, cols = np.mgrid[0:blob.shape[0], 0:blob.shape[1]]
    pix = np.stack([cols, rows], -1).astype(float)
    across = (pix - sc) @ np.array([sd[1], -sd[0]])
    below = (pix - c) @ n
    point = blob & (below > LINE_TOL_PX) & (below < POINT_MM * px_mm) \
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
    off_seam = np.abs((np.array(apexes) - sc) @ np.array([sd[1], -sd[0]])) / px_mm
    if off_seam.max() > APEX_OFF_SEAM_MM:
        raise JawsNotFound(f"a jaw's point is {off_seam.max():.1f} mm from the seam, not at it: the corners or a "
                           "point were misread")
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
