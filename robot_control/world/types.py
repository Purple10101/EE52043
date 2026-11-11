"""Plain records. Robot-frame lengths are metres, angles degrees (robot frame: angle 0 along +y,
positive toward +x, as the firmware's forward kinematics)."""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass(frozen=True)
class JointState:
    j0: float                       # base, deg
    j1: float                       # elbow, deg
    j2: float                       # wrist, deg
    z: float                        # slide, m


@dataclass(frozen=True, eq=False)
class Detection:
    """One resistor body: its outline in both frames and the geometry derived from it, once.

    Build with `from_outlines`; vision passes both outlines in, Detection does not know how pixels
    map to metres.
    """

    snapshot_id: str
    outline_px: np.ndarray          # N × 2, raw camera frame px
    outline_robot: np.ndarray       # N × 2, robot frame m
    score: float                    # the detector's confidence
    centre: tuple[float, float]     # m
    yaw: float                      # long axis, deg in (-90, 90]
    length: float                   # m
    width: float                    # m
    plausible: bool                 # the size matches a configured package

    @classmethod
    def from_outlines(cls, snapshot_id, outline_px, outline_robot, packages, tolerance, score=1.0):
        """packages: config.Package sizes; a body is plausible within tolerance (a share) of one."""
        outline_px = np.asarray(outline_px, float).reshape(-1, 2)
        outline_robot = np.asarray(outline_robot, float).reshape(-1, 2)
        centre, length, width, yaw = rectangle(outline_robot)
        plausible = any(abs(length - p.length) <= tolerance * p.length and abs(width - p.width) <= tolerance * p.width
                        for p in packages)
        return cls(snapshot_id, outline_px, outline_robot, float(score), centre, yaw, length, width, plausible)


def rectangle(outline):
    """(centre, length, width, yaw) of the minimum-area rectangle around an outline in metres.

    Yaw comes from the rectangle's corners, not minAreaRect's angle, whose convention changed between
    OpenCV versions.
    """
    mm = (np.asarray(outline, float) * 1000).astype(np.float32)   # minAreaRect is happier in mm
    rect = cv2.minAreaRect(mm)
    corners = cv2.boxPoints(rect).astype(float) / 1000
    a, b = corners[1] - corners[0], corners[2] - corners[1]
    long_side, short_side = (a, b) if np.hypot(*a) >= np.hypot(*b) else (b, a)
    yaw = math.degrees(math.atan2(long_side[0], long_side[1]))     # from +y toward +x
    yaw = (yaw + 90.0) % 180.0 - 90.0                              # [-90, 90)
    if yaw == -90.0:
        yaw = 90.0
    centre = corners.mean(0)
    return (float(centre[0]), float(centre[1])), float(np.hypot(*long_side)), float(np.hypot(*short_side)), yaw


@dataclass(frozen=True, eq=False)
class Resistor:
    """One resistor as the camera saw it, ready to plan with: where it lies, which way, and what its bands
    say. Scene frame: mm on the plate, degrees from scene +x anticlockwise (workspace.toml)."""

    number: int                     # its place in the look, from 1, as the window labels it
    centre_mm: tuple[float, float]  # where the jaws' centre goes: the body's centroid, moved for its height
    direction_deg: float            # its long axis
    score: float                    # how sure segmentation is that this is a body
    value_ohm: float | None         # its value when the band reader is sure enough, else None
    best_ohm: float                 # the reader's best guess, sure or not
    confidence: float               # the reader's confidence in best_ohm, 0 to 1
    outline_px: np.ndarray          # N × 2, rectified px, for drawing
    closeup: bool = False           # its value read from a close-up too, not just the plate-wide picture
    colours: tuple = ()             # the bands the reader made of it, in reading order
    body_id: int = 0                # which body this is from look to look (vision.BodyMemory), 0 if not followed
    crop: object = None             # its band crop from this look's picture (BGR), for the log
    closeup_crop: object = None     # its close-up's band crop, if it has had one

    @property
    def is_read(self):
        """Whether its value is known well enough to sort it."""
        return self.value_ohm is not None

    def distance_to(self, point_mm):
        return float(math.hypot(self.centre_mm[0] - point_mm[0], self.centre_mm[1] - point_mm[1]))


@dataclass(eq=False)
class PlateView:
    """One look at the plate, taken with the arm parked: the frame, where the plate is in it, and the
    resistors on it."""

    id: str
    frame: np.ndarray               # the raw camera frame
    cal: object                     # robot_offline's rectify.Calibration for that frame
    resistors: list[Resistor]
    faint: list = field(default_factory=list)       # (outline px, score, centre mm) found but too faint to be a body
    missing: list = field(default_factory=list)     # centres (mm) of bodies seen before but not in this look


@dataclass(eq=False)
class Snapshot:
    id: str
    image: np.ndarray               # the raw camera frame
    captured: float                 # time.time()
    detections: list[Detection] = field(default_factory=list)
