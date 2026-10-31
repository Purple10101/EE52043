import math

import numpy as np
import pytest

from robot_control.config import RigSettings, WorkspaceSettings
from robot_control.vision.rig import correct_height, robot_to_scene, scene_to_robot

RIG = RigSettings(180.0, (0.0, 0.0))


def test_plate_centre_in_the_robot_frame():
    assert scene_to_robot([[0.0, -130.0]], RIG)[0] == pytest.approx((0.0, 0.130))
    assert scene_to_robot([[50.0, -80.0]], RIG)[0] == pytest.approx((-0.050, 0.080))


def test_rig_round_trip_with_a_correction():
    rig = RigSettings(178.5, (0.002, -0.001))
    p = np.array([[12.0, -150.0], [-40.0, -90.0]])
    assert robot_to_scene(scene_to_robot(p, rig), rig) == pytest.approx(p)


def test_height_correction():
    camera = (-145.0, -145.0, 178.0)            # 200 mm above the plate top at z -22
    # Straight below the camera nothing moves; elsewhere the point comes 1/200 per mm of height
    # toward the camera's foot.
    assert correct_height([[-145.0, -145.0]], camera, -22.0, 1.2)[0] == pytest.approx((-145.0, -145.0))
    moved = correct_height([[55.0, -145.0]], camera, -22.0, 1.2)[0]
    assert moved == pytest.approx((55.0 - 200.0 * 1.2 / 200.0, -145.0))


def test_to_detections_on_the_rectified_plate():
    rectify = pytest.importorskip("robot_offline.pre.rectify")
    from robot_control.vision.detector import to_detections

    st = rectify.Settings.from_config()
    cal = rectify.Calibration(np.eye(3), np.zeros((4, 2)), (4608, 2592), st, fit={"camera_mm": [0.0, -130.0, 178.0]})
    # A 1/4 W body at the plate centre, its long axis along scene x (robot yaw +-90).
    a = math.radians(30.0)
    corners_mm = np.array([[0.0, -130.0] + np.array([math.cos(a), math.sin(a)]) * sx * 3.15
                           + np.array([-math.sin(a), math.cos(a)]) * sy * 1.2
                           for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))])
    polygon = rectify.mm_to_rect(corners_mm, st)
    [d] = to_detections([polygon], [0.9], cal, RIG, WorkspaceSettings.from_config(), "t")
    assert d.centre == pytest.approx((0.0, 0.130), abs=1e-6)     # straight below the camera: no shift
    # The outline is taken to be at the axis height (half the width) and so pulled toward the camera.
    assert d.length == pytest.approx(0.0063 * (200 - 1.2) / 200, abs=1e-6)
    assert d.plausible
    # Scene direction 30 deg from +x is, turned 180, robot direction 210 deg from +x = yaw (from +y) 60 - 180.
    assert d.yaw == pytest.approx(60.0, abs=0.01)
    assert d.outline_px == pytest.approx(polygon)                # identity homography, no lens model
