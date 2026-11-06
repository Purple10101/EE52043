import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("serial")
pytest.importorskip("robot_offline.pre.rectify")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import resistor_nav  # noqa: E402
from resistor_nav import BIN_SLIDE, Navigator, bin_at, bin_path, body_at, load_bins, waiting_view  # noqa: E402

from robot_control.config import KeepOutMap  # noqa: E402
from robot_control.kinematics import KeepOut, first_hit  # noqa: E402
from robot_control.vision.guide import jaw_angle, wrist_across  # noqa: E402

A = np.array([[100, 100], [160, 100], [160, 124], [100, 124]], float)
B = A + (300, 200)


def test_clicking_a_body_chooses_it():
    assert body_at([A, B], (130, 110)) == 0
    assert body_at([A, B], (430, 310)) == 1


def test_a_click_just_off_a_body_still_chooses_it():
    assert body_at([A, B], (170, 110)) == 0


def test_a_click_on_bare_plate_chooses_nothing():
    assert body_at([A, B], (300, 500)) is None
    assert body_at([], (10, 10)) is None


def test_the_eight_bins_load_in_order():
    assert list(load_bins()) == ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4"]


def test_a_bin_button_is_found_along_its_row():
    names = list("abcd")
    assert [bin_at(x, 400, names) for x in (0, 99, 100, 399, 400)] == ["a", "a", "b", "d", "d"]


def test_the_way_to_a_bin_stays_clear_of_the_tower():
    zones = [KeepOut(m) for m in KeepOutMap.from_config()]
    start = (-22.0, 78.0)                  # elbow low, so a straight swing would cross the tower
    path = bin_path(zones, start, (60.6, 84.7))
    assert len(path) > 1
    for a, b in zip([start] + path, path):
        assert first_hit(zones, a, b) is None


def test_there_is_no_way_to_a_bin_inside_the_tower_zone():
    zones = [KeepOut(m) for m in KeepOutMap.from_config()]
    assert bin_path(zones, (-77.0, 110.0), (7.0, 85.3)) is None


def test_the_waiting_view_fits_a_wide_camera_frame_into_the_plate_view():
    view = waiting_view(np.full((2592, 4608, 3), 200, np.uint8), 750, 750)
    assert view.shape == (750, 750, 3)
    assert view[375, 375, 0] == 200 and view[0, 375, 0] == 0


sent = []


def run_to_bin(monkeypatch, landed):
    """to_bin with the arm faked, landing at landed (base, elbow): the moves it made, in order."""
    made = []
    sent.clear()
    bench = resistor_nav.bench
    monkeypatch.setattr(resistor_nav, "pots_for", lambda *a: (20.0, 100.0))
    monkeypatch.setattr(resistor_nav, "bin_path", lambda *a: [(20.0, 100.0)])
    monkeypatch.setattr(bench, "read_joints", lambda *a: landed)
    monkeypatch.setattr(bench, "load_state", lambda: {"wrist": 50.0, "slide": 20.0})
    monkeypatch.setattr(bench, "swing", lambda *a: (made.append("swing"), sent.append(a)))
    nav = Navigator(None, None, None, None)
    monkeypatch.setattr(nav, "drive", lambda joint, angle, seconds: made.append((joint, angle)))
    nav.to_bin("B1", {"centre_mm": [-129.3, -46.4], "long_axis_deg": 10.9})
    return made, nav.status


def test_the_gripper_lowers_once_it_is_over_the_bin(monkeypatch):
    made, status = run_to_bin(monkeypatch, (20.5, 100.5))
    assert made == [("slide", resistor_nav.bench.HOVER_SLIDE), "swing", ("slide", BIN_SLIDE)]
    assert "lowered" in status


def test_the_gripper_stays_up_if_the_arm_missed_the_bin(monkeypatch):
    made, status = run_to_bin(monkeypatch, (26.0, 100.0))
    assert ("slide", BIN_SLIDE) not in made
    assert "NOT LOWERING" in status


def test_the_jaws_turn_so_a_held_resistor_lies_along_the_bin(monkeypatch):
    run_to_bin(monkeypatch, (20.0, 100.0))
    _, base, elbow, wrist, _ = sent[0]
    assert (jaw_angle(base, elbow, wrist) - 10.9 - 90.0) % 180.0 == pytest.approx(0.0, abs=1e-6)


def test_every_bin_has_a_wrist_angle_in_range():
    for slot in load_bins().values():
        assert -90.0 <= wrist_across(50.0, 110.0, slot["long_axis_deg"], (-90.0, 120.0), 50.0) <= 120.0


def run_go(monkeypatch, slide, path):
    """go with the arm faked and the slide at slide: the moves it made, in order."""
    made = []
    bench = resistor_nav.bench
    monkeypatch.setattr(resistor_nav, "plan_jaws", lambda *a, **k: (-50.0, 100.0, 40.0))
    monkeypatch.setattr(resistor_nav, "bin_path", lambda *a: path)
    monkeypatch.setattr(bench, "swing_refusal", lambda *a, **k: None)
    monkeypatch.setattr(bench, "read_joints", lambda *a: (55.0, 110.0))
    monkeypatch.setattr(bench, "load_state", lambda: {"wrist": 8.0, "slide": slide})
    monkeypatch.setattr(bench, "swing", lambda *a: made.append("swing"))
    monkeypatch.setattr(bench, "glide", lambda *a: made.append("glide"))

    class Arm:
        arm = None

    nav = Navigator(None, Arm(), None, None)
    monkeypatch.setattr(nav, "drive", lambda joint, angle, seconds: made.append((joint, angle)))
    nav.go((0.0, -130.0), 45.0)
    return made, nav.status


def test_leaving_a_bin_for_a_resistor_lifts_first(monkeypatch):
    made, _ = run_go(monkeypatch, BIN_SLIDE, [(-50.0, 100.0)])
    assert made == [("slide", resistor_nav.bench.HOVER_SLIDE), "glide"]


def test_going_round_the_tower_comes_after_the_lift(monkeypatch):
    made, _ = run_go(monkeypatch, BIN_SLIDE, [(55.0, 115.0), (-50.0, 115.0), (-50.0, 100.0)])
    assert made == [("slide", resistor_nav.bench.HOVER_SLIDE), "swing", "swing", "glide"]


def test_a_move_between_resistors_doesnt_lift(monkeypatch):
    made, _ = run_go(monkeypatch, resistor_nav.NAV_SLIDE, [(-50.0, 100.0)])
    assert made == ["glide"]


def test_nothing_moves_with_no_way_round_the_tower(monkeypatch):
    made, status = run_go(monkeypatch, BIN_SLIDE, None)
    assert made == []
    assert "nothing moved" in status
