import math

import numpy as np
import pytest

from robot_control.config import Package
from robot_control.world import Detection

PACKAGES = (Package("1/4W", 0.0063, 0.0024), Package("1/2W", 0.009, 0.0032))


def body(cx, cy, length, width, yaw_deg):
    """Rectangle corners (m) with its long side at yaw_deg (robot convention: from +y toward +x)."""
    a = math.radians(yaw_deg)
    along, across = np.array([math.sin(a), math.cos(a)]), np.array([math.cos(a), -math.sin(a)])
    return np.array([[cx, cy] + sa * along * length / 2 + sb * across * width / 2
                     for sa, sb in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


@pytest.mark.parametrize("yaw", [0.0, 45.0, 90.0, -30.0, 89.0, -89.0])
def test_rectangle_geometry(yaw):
    d = Detection.from_outlines("s", np.zeros((4, 2)), body(0.01, 0.13, 0.0063, 0.0024, yaw), PACKAGES, 0.3)
    assert d.centre == pytest.approx((0.01, 0.13), abs=1e-6)
    assert d.length == pytest.approx(0.0063, abs=1e-6)
    assert d.width == pytest.approx(0.0024, abs=1e-6)
    assert d.yaw == pytest.approx(yaw, abs=0.01)
    assert -90.0 < d.yaw <= 90.0


def test_yaw_range_ends_at_plus_90():
    d = Detection.from_outlines("s", np.zeros((4, 2)), body(0, 0.13, 0.0063, 0.0024, -90.0), PACKAGES, 0.3)
    assert d.yaw == pytest.approx(90.0, abs=0.01)


@pytest.mark.parametrize("length, width, plausible", [
    (0.0063, 0.0024, True),     # 1/4 W
    (0.0088, 0.0033, True),     # 1/2 W
    (0.015, 0.015, False),      # a blob
    (0.002, 0.001, False),      # a speck
])
def test_plausible(length, width, plausible):
    d = Detection.from_outlines("s", np.zeros((4, 2)), body(0, 0.13, length, width, 20.0), PACKAGES, 0.3)
    assert d.plausible is plausible
