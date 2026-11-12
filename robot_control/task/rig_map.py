"""RigMap: where each bin is on the rig, and where the arm is now, drawn on the twin's top view.

The picture is robot_control/config/rig_map.png, the digital twin's rig from straight above with tray A
turned as it is on the rig (scripts/twin/render_top_view.py; kept out of git with the twin). Each slot is
drawn on it where it physically is (rig_map.json's slots), labelled with its id and the value it holds,
and coloured as in the tray list: green holds a value, grey not used yet, an orange border the bin the
arm is heading for. Without the picture the slots are drawn on a plain background where they were
taught (bins_real.toml), which is about where the arm drops.

Given an ArmPose (hardware.ArmMonitor), the arm is drawn over it as it is at that moment: its two links
from the base axis, the jaws at the wrist turned the way they close and opened as far as they are, a
resistor between them while it holds one, an orange ring where a move is heading, and the slide's
height on a gauge in the title strip.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
from robot_offline.post.bands.learned import ohms_text

from robot_control.config import ArmSettings, RigSettings
from robot_control.hardware.arm_monitor import JAWS_OPEN_MM
from robot_control.vision.guide import jaw_angle
from robot_control.vision.rig import robot_to_scene

MAP = Path(__file__).resolve().parents[1] / "config" / "rig_map"
SLOT_MM = (66.0, 16.0)                      # each slot's length and width (bins.toml, the CAD)
EXTENT_MM = {"x": (-200.0, 140.0), "y": (-200.0, 95.0)}     # without the picture (the parked elbow reaches x 122)
PLACES = {"PLATE": (0.0, -130.0), "ROBOT BASE": (0.0, 0.0), "CAMERA": (-150.0, -160.0),
          "NON-RES BIN": (68.0, -130.0)}    # landmarks, scene mm
HOLDS, UNUSED, TARGET = (40, 140, 40), (110, 110, 110), (0, 160, 255)    # BGR
ARM, ARM_EDGE, JAWS, BODY = (70, 205, 240), (20, 60, 90), (40, 40, 40), (200, 120, 30)
LINK_WIDTH_MM = (26.0, 18.0)                # how wide each link is drawn
PAD_MM = (3.0, 9.0)                         # a jaw's pad: thick (along the closing line), long
RESISTOR_MM = (6.3, 2.4)                    # a 1/4 W body
SLIDE_GAUGE = (-50.0, 63.0)                 # the slide's range, fully up to the plate (firmware deg)
TITLE_H = 30
FONT = cv2.FONT_HERSHEY_SIMPLEX


class RigMap:
    def __init__(self, slots, picture=None, info=None, arm=None, rig=None):
        """slots: {bin id: entry} with centre_mm and long_axis_deg (resistor_nav.load_bins). picture, info:
        the twin's top view and its rig_map.json, or None for a plain background. arm, rig: config's
        ArmSettings and RigSettings, to draw the arm; None to leave it out."""
        if info and "slots" in info:        # where they are on the rig
            slots = info["slots"]
        self.slots = slots
        if picture is None:
            k = 2.0
            info = {"px_per_mm": k, "x_left_mm": EXTENT_MM["x"][0], "y_top_mm": EXTENT_MM["y"][1]}
            size = (int((EXTENT_MM["y"][1] - EXTENT_MM["y"][0]) * k), int((EXTENT_MM["x"][1] - EXTENT_MM["x"][0]) * k))
            picture = np.full((*size, 3), 45, np.uint8)
        self.picture, self.info = picture, info
        self.links_m = None if arm is None else (arm.link1, arm.link2)
        self.rig = rig
        self._resized = {}

    @classmethod
    def load(cls, slots, path=MAP):
        """With the twin's picture if it has been made, else on a plain background; the arm from the config."""
        png, meta = path.with_suffix(".png"), path.with_suffix(".json")
        arm, rig = ArmSettings.from_config(), RigSettings.from_config()
        if png.exists() and meta.exists():
            return cls(slots, cv2.imread(str(png)), json.loads(meta.read_text()), arm, rig)
        return cls(slots, arm=arm, rig=rig)

    def draw(self, trays, target, width, pose=None):
        """The map width px wide, each slot labelled from trays (planning.Tray); target: the bin the arm is
        heading for, or None; pose: the arm now (hardware.ArmPose), or None to leave it out."""
        image, k = self._background(width)
        px = lambda p: ((p[0] - self.info["x_left_mm"]) * k, (self.info["y_top_mm"] - p[1]) * k)
        self._landmarks(image, px)
        self._slots(image, px, trays, target)
        if pose is not None and self.links_m is not None:
            self._arm(image, px, k, pose)
        return np.vstack([self._title(width, pose), image])

    # The parts of the map, bottom to top.

    def _background(self, width):
        if width not in self._resized:
            scale = width / self.picture.shape[1]
            self._resized[width] = (cv2.resize(self.picture, (width, int(self.picture.shape[0] * scale)),
                                               interpolation=cv2.INTER_AREA), self.info["px_per_mm"] * scale)
        image, k = self._resized[width]
        return image.copy(), k

    def _landmarks(self, image, px):
        for name, where in PLACES.items():
            x, y = px(where)
            (w, _), _ = cv2.getTextSize(name, FONT, 0.4, 1)
            cv2.putText(image, name, (int(x - w / 2), int(y)), FONT, 0.4, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(image, name, (int(x - w / 2), int(y)), FONT, 0.4, (30, 30, 30), 1, cv2.LINE_AA)

    def _slots(self, image, px, trays, target):
        by_id = {t.bin_id: t for t in trays}
        overlay = image.copy()
        outlines = []
        for bin_id, slot in self.slots.items():
            tray = by_id.get(bin_id)
            colour = HOLDS if tray and (tray.count or tray.value_ohm) else UNUSED
            poly = _poly(slot_corners(slot["centre_mm"], slot["long_axis_deg"]), px)
            cv2.fillPoly(overlay, [poly], colour, cv2.LINE_AA, shift=2)
            outlines.append((poly, bin_id == target))
        image[:] = cv2.addWeighted(overlay, 0.75, image, 0.25, 0)
        for poly, is_target in outlines:
            cv2.polylines(image, [poly], True, TARGET if is_target else (255, 255, 255), 3 if is_target else 1,
                          cv2.LINE_AA, shift=2)
        for bin_id, slot in self.slots.items():
            tray = by_id.get(bin_id)
            text = bin_id + (f" {ohms_text(tray.value_ohm)}" if tray and tray.value_ohm else "")
            upright = abs(math.sin(math.radians(slot["long_axis_deg"]))) > 0.7     # written along the slot
            _label(image, text, px(slot["centre_mm"]), upright)

    def _arm(self, image, px, k, pose):
        if pose.base is None or pose.elbow is None:
            if pose.calibrating:
                cv2.putText(image, "calibrating: base and elbow sweeping", (10, 24), FONT, 0.5, TARGET, 2, cv2.LINE_AA)
            return
        if pose.moving and ("base" in pose.goal or "elbow" in pose.goal):
            goal = self.joints_mm(pose.goal.get("base", pose.base), pose.goal.get("elbow", pose.elbow))
            gx, gy = px(goal[2])
            cv2.circle(image, (int(gx), int(gy)), max(4, int(6 * k)), TARGET, 2, cv2.LINE_AA)
        base, elbow, wrist = self.joints_mm(pose.base, pose.elbow)
        overlay = image.copy()
        for (a, b), width_mm in zip(((base, elbow), (elbow, wrist)), LINK_WIDTH_MM):
            pa, pb = px(a), px(b)
            thick = max(3, int(width_mm * k))
            cv2.line(overlay, _pt(pa), _pt(pb), ARM_EDGE, thick + 4, cv2.LINE_AA)
            cv2.line(overlay, _pt(pa), _pt(pb), ARM, thick, cv2.LINE_AA)
            cv2.circle(overlay, _pt(pb), thick // 2 + 2, ARM_EDGE, -1, cv2.LINE_AA)
            cv2.circle(overlay, _pt(pb), thick // 2, ARM, -1, cv2.LINE_AA)
        image[:] = cv2.addWeighted(overlay, 0.85, image, 0.15, 0)
        if pose.wrist is not None:
            self._jaws(image, px, wrist, jaw_angle(pose.base, pose.elbow, pose.wrist), pose)

    def _jaws(self, image, px, centre, closing_deg, pose):
        """Two pads either side of the jaws' centre along the line they close on, as far apart as they are
        open; a resistor lying across that line between them while holding one."""
        a = math.radians(closing_deg)
        along = np.array([math.cos(a), math.sin(a)])
        gap = JAWS_OPEN_MM if pose.jaws_mm is None else pose.jaws_mm
        if pose.holding:
            gap = max(gap, RESISTOR_MM[1])
            cv2.fillPoly(image, [_poly(slot_corners(centre, closing_deg + 90.0, RESISTOR_MM), px)], BODY, cv2.LINE_AA, shift=2)
        for side in (-1, 1):
            pad = np.asarray(centre) + side * (gap / 2 + PAD_MM[0] / 2) * along
            cv2.fillPoly(image, [_poly(slot_corners(pad, closing_deg, PAD_MM), px)], JAWS, cv2.LINE_AA, shift=2)

    def _title(self, width, pose):
        title = np.full((TITLE_H, width, 3), 30, np.uint8)
        cv2.putText(title, "WHERE THE BINS ARE", (10, 21), FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        if pose is not None and pose.slide is not None:
            x0, x1, y = width - 190, width - 50, TITLE_H // 2
            cv2.putText(title, "slide", (x0 - 46, y + 5), FONT, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
            cv2.line(title, (x0, y), (x1, y), (110, 110, 110), 4)
            share = (pose.slide - SLIDE_GAUGE[0]) / (SLIDE_GAUGE[1] - SLIDE_GAUGE[0])
            mark = int(x0 + min(max(share, 0.0), 1.0) * (x1 - x0))
            cv2.circle(title, (mark, y), 7, ARM, -1, cv2.LINE_AA)
            cv2.putText(title, "up" if share < 0.25 else "hover" if share < 0.7 else "low", (x1 + 8, y + 5), FONT, 0.45,
                        (200, 200, 200), 1, cv2.LINE_AA)
        return title

    def joints_mm(self, base_deg, elbow_deg):
        """(base axis, elbow, wrist) in scene mm, by the firmware's forward kinematics."""
        l1, l2 = self.links_m
        b, e = math.radians(base_deg), math.radians(base_deg + elbow_deg)
        elbow = (l1 * math.sin(b), l1 * math.cos(b))
        wrist = (elbow[0] + l2 * math.sin(e), elbow[1] + l2 * math.cos(e))
        return tuple(robot_to_scene([(0.0, 0.0), elbow, wrist], self.rig))



def _label(image, text, centre, upright, scale=0.42):
    """White text centred on centre (px), turned to read upwards when upright."""
    (w, h), base = cv2.getTextSize(text, FONT, scale, 1)
    patch = np.zeros((h + base + 4, w + 4), np.uint8)
    cv2.putText(patch, text, (2, h + 2), FONT, scale, 255, 1, cv2.LINE_AA)
    if upright:
        patch = cv2.rotate(patch, cv2.ROTATE_90_COUNTERCLOCKWISE)
    ph, pw = patch.shape
    x0, y0 = int(centre[0] - pw / 2), int(centre[1] - ph / 2)
    xa, ya, xb, yb = max(x0, 0), max(y0, 0), min(x0 + pw, image.shape[1]), min(y0 + ph, image.shape[0])
    if xa >= xb or ya >= yb:
        return
    alpha = patch[ya - y0:yb - y0, xa - x0:xb - x0, None] / 255.0
    region = image[ya:yb, xa:xb]
    region[:] = (region * (1 - alpha) + 255 * alpha).astype(np.uint8)


def _pt(p):
    return int(round(p[0])), int(round(p[1]))


def _poly(corners, px):
    """Scene mm corners to a polygon for cv2's fixed-point drawing (shift=2)."""
    return np.round(np.array([px(c) for c in corners]) * 4).astype(np.int32)


def slot_corners(centre, long_axis_deg, size=SLOT_MM):
    """A slot's four corners, scene mm: size (length, width) about centre, its length along long_axis_deg."""
    a = np.radians(long_axis_deg)
    along, across = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    c = np.asarray(centre, float)
    l, w = size[0] / 2, size[1] / 2
    return [c + l * along + w * across, c - l * along + w * across, c - l * along - w * across, c + l * along - w * across]
