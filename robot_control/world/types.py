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


@dataclass(eq=False)
class Snapshot:
    id: str
    image: np.ndarray               # the raw camera frame
    captured: float                 # time.time()
    detections: list[Detection] = field(default_factory=list)
