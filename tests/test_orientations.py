import cv2
import numpy as np
import pytest

from robot_control.vision.orientations import merge_mean, predict_two_ways, rotate_area, unrotate_points

SIZE = 1200
AREA = (100, 100, 1000, 1000)


def test_unrotate_points_undoes_rot90_pixel_for_pixel():
    image = np.zeros((SIZE, SIZE), np.uint8)
    image[300, 700] = 255                                   # (x 700, y 300)
    turned = np.rot90(image)
    y, x = np.argwhere(turned == 255)[0]
    assert unrotate_points([[x, y]], SIZE)[0].tolist() == [700, 300]


def test_rotate_area_covers_the_same_pixels():
    mask = np.zeros((SIZE, SIZE), np.uint8)
    x0, y0, w, h = 50, 200, 900, 700
    mask[y0:y0 + h, x0:x0 + w] = 1
    turned = np.rot90(mask)
    rx0, ry0, rw, rh = rotate_area((x0, y0, w, h), SIZE)
    assert turned[ry0:ry0 + rh, rx0:rx0 + rw].all() and turned.sum() == rw * rh


def _square(cx, cy, r=10):
    return np.array([[cx - r, cy - r], [cx + r, cy - r], [cx + r, cy + r], [cx - r, cy + r]], float)


def test_merge_mean_averages_matched_and_counts_unmatched_as_zero():
    first = ([_square(100, 100), _square(500, 500)], [0.9, 0.6])
    second = ([_square(101, 100), _square(800, 800)], [0.5, 0.8])
    polygons, scores, pairs = merge_mean(first, second)
    got = {pair: score for pair, score in zip(pairs, scores)}
    assert got == {(0.9, 0.5): pytest.approx(0.7), (0.6, 0.0): pytest.approx(0.3), (0.0, 0.8): pytest.approx(0.4)}
    # the matched body keeps the better-scoring way's outline
    k = [p for p in pairs].index((0.9, 0.5))
    assert np.allclose(polygons[k], _square(100, 100))


def _bar(image, centre, angle_deg, length=60, width=22):
    box = cv2.boxPoints((centre, (length, width), angle_deg)).astype(np.int32)
    cv2.fillPoly(image, [box], (40, 40, 40))


def fake_model(model, image, frame_tiles, conf, grey):
    """Finds dark bars; unsure (0.4) of one lying at 120-140 deg in the image it is given, sure (0.9) otherwise."""
    dark = (cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) < 100).astype(np.uint8)
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    polygons, scores = [], []
    for c in contours:
        (_, _), (w, h), a = cv2.minAreaRect(c)
        angle = (a if w >= h else a + 90) % 180
        s = 0.4 if 120 <= angle <= 140 else 0.9
        if s >= conf:
            polygons.append(c[:, 0, :].astype(float))
            scores.append(s)
    return polygons, scores


def test_two_ways_lifts_a_body_at_the_weak_angle_and_keeps_outlines_in_place():
    image = np.full((SIZE, SIZE, 3), 220, np.uint8)
    _bar(image, (400, 500), 130)          # the weak angle this way round, 40 deg turned
    _bar(image, (800, 700), 20)           # fine both ways
    polygons, scores, pairs = predict_two_ways(None, image, AREA, 1024, 128, conf=0.5, grey=True,
                                               predict=fake_model, with_pairs=True)
    assert len(polygons) == 2
    by_x = sorted(zip(polygons, scores, pairs), key=lambda t: t[0][:, 0].mean())
    (p1, s1, pr1), (p2, s2, pr2) = by_x
    assert pr1 == (0.4, 0.9) and s1 == pytest.approx(0.65)
    assert s2 == pytest.approx(0.9)
    assert np.allclose(p1.mean(axis=0), (400, 500), atol=2)
    assert np.allclose(p2.mean(axis=0), (800, 700), atol=2)


def test_two_ways_drops_what_averages_under_conf():
    image = np.full((SIZE, SIZE, 3), 220, np.uint8)
    _bar(image, (600, 600), 45)
    only_straight = lambda m, img, t, conf, grey: fake_model(m, img, t, conf, grey) if img[0, 0, 0] == 221 else ([], [])
    image[0, 0] = 221                     # marks the straight image (the turned one has it elsewhere)
    polygons, scores = predict_two_ways(None, image, AREA, 1024, 128, conf=0.5, grey=True, predict=only_straight)
    assert polygons == [] and scores == []       # 0.9 one way, nothing the other: 0.45


def test_non_square_refused():
    with pytest.raises(ValueError):
        predict_two_ways(None, np.zeros((10, 12, 3), np.uint8), AREA, 1024, 128, 0.5, True, predict=fake_model)
