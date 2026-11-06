import cv2
import numpy as np
import pytest

rectify = pytest.importorskip("robot_offline.pre.rectify")
from robot_control.vision.tip import TipNotFound, find_tip, tip_in_rectified  # noqa: E402

ST = rectify.Settings.from_config()
LENS = (-143.0, -144.0, 185.0)      # the foot is toward scene -x from the plate, so the tip points that way
TIP = (-5.0, -130.0)


def plate():
    w, h = ST.size
    image = np.zeros((h, w, 3), np.uint8)
    image[rectify.plate_mask(ST)] = 190
    return image


def fill(image, points_mm):
    cv2.fillPoly(image, [rectify.mm_to_rect(points_mm, ST).round().astype(np.int32)], (20, 20, 20))


def radial(tip, points):
    """Points given as (mm away from the camera's foot, mm across) from tip, to scene mm."""
    away = np.array(tip) - LENS[:2]
    away /= np.linalg.norm(away)
    across = np.array([-away[1], away[0]])
    return [tuple(np.array(tip) + a * away + b * across) for a, b in points]


def gripper(image, tip=TIP):
    """The closed tips' V, 4.5 mm wide 2 mm back, on a body: upright, so it lies along the camera's line."""
    fill(image, radial(tip, [(0, 0), (4, 4.5), (4, 15), (40, 15), (40, -15), (4, -15), (4, -4.5)]))


def blunt(image, end_x=-5.0):
    fill(image, [(end_x, -140), (end_x, -120), (end_x + 40, -120), (end_x + 40, -140)])


def cable(image, start, end):
    a, b = rectify.mm_to_rect([start, end], ST).round().astype(int)
    cv2.line(image, tuple(int(i) for i in a), tuple(int(i) for i in b), (20, 20, 20), 20)


def test_finds_the_point_of_the_v():
    image = plate()
    gripper(image)
    mm, width = tip_in_rectified(image, ST, LENS, TIP)
    assert mm == pytest.approx(TIP, abs=0.3)
    assert width < 7.0


def test_light_through_the_closed_jaws_does_not_split_the_v():
    image = plate()
    gripper(image)
    seam = rectify.mm_to_rect(radial(TIP, [(-0.5, 0), (14, 0)]), ST).round().astype(int)
    cv2.line(image, tuple(int(i) for i in seam[0]), tuple(int(i) for i in seam[1]), (190, 190, 190), 6)
    assert tip_in_rectified(image, ST, LENS, TIP)[0] == pytest.approx(TIP, abs=0.5)     # the slit takes the very point


def test_a_dark_resistor_beside_the_tip_is_ignored_when_outlined():
    image = plate()
    gripper(image)
    body = [(-9.0, -131.0), (-9.0, -124.0), (-5.5, -124.0), (-5.5, -131.0)]     # just past the V, toward the camera
    fill(image, body)
    outline = rectify.mm_to_rect(body, ST)
    assert tip_in_rectified(image, ST, LENS, TIP, ignore=[outline])[0] == pytest.approx(TIP, abs=0.8)


def test_a_cable_off_the_gripper_is_not_the_tip():
    image = plate()
    gripper(image)
    cable(image, (10.0, -120.0), (-40.0, -100.0))
    assert tip_in_rectified(image, ST, LENS, TIP)[0] == pytest.approx(TIP, abs=0.3)


def test_a_blunt_end_with_a_cable_off_it_is_refused():
    image = plate()
    blunt(image)
    cable(image, (0.0, -125.0), (-40.0, -100.0))
    with pytest.raises(TipNotFound, match="blunt"):
        tip_in_rectified(image, ST, LENS, TIP)


def test_dark_things_away_from_the_expected_tip_are_ignored():
    image = plate()
    gripper(image)
    fill(image, radial((-35.0, -100.0), [(0, 0), (4, 4.5), (20, 4.5), (20, -4.5), (4, -4.5)]))  # a V nearer the camera
    assert tip_in_rectified(image, ST, LENS, (-1.0, -128.0))[0] == pytest.approx(TIP, abs=0.3)


def test_a_gripper_only_reaching_the_search_area_is_refused():
    image = plate()
    gripper(image)
    with pytest.raises(TipNotFound, match="search area"):
        tip_in_rectified(image, ST, LENS, (25.0, -130.0))


def test_nothing_near_the_expected_tip():
    image = plate()
    gripper(image)
    with pytest.raises(TipNotFound, match="nothing dark"):
        tip_in_rectified(image, ST, LENS, (-30.0, -100.0))


def test_the_lowest_piece_counts_when_a_cable_splits_the_gripper():
    image = plate()
    fill(image, [(-20, -140), (0, -140), (0, -120), (-20, -120)])       # the blunt jaws, the smaller piece
    gripper(image, (5.0, -130.0))                                       # a sharp end on the bigger one
    with pytest.raises(TipNotFound, match="blunt"):
        tip_in_rectified(image, ST, LENS, (-15.0, -130.0))


def test_a_corner_of_something_turned_is_refused():
    image = plate()
    fill(image, radial(TIP, [(0, 0), (40, 3), (40, -40)]))
    with pytest.raises(TipNotFound, match="corner"):
        tip_in_rectified(image, ST, LENS, TIP)


def test_marks_and_an_empty_plate_are_not_a_tip():
    image = plate()
    fill(image, [(0, -130), (1, -130), (1, -129), (0, -129)])
    with pytest.raises(TipNotFound):
        tip_in_rectified(image, ST, LENS, TIP)


def test_a_tip_at_the_plates_edge_is_refused():
    image = plate()
    gripper(image, (-49.0, -144.0))
    with pytest.raises(TipNotFound, match="edge"):
        tip_in_rectified(image, ST, LENS, (-49.0, -144.0))


def test_find_tip_corrects_for_height():
    cal = rectify.Calibration(np.eye(3), np.zeros((4, 2)), ST.size, ST, fit={"camera_mm": list(LENS)})
    image = plate()
    gripper(image)
    low, high = find_tip(image, cal, TIP), find_tip(image, cal, TIP, height_mm=20.0)
    above = LENS[2] - ST.top_z_mm
    assert high - low == pytest.approx((np.array(LENS[:2]) - low) * 20.0 / above, abs=1e-6)
