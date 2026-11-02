import json

import numpy as np
import pytest

from robot_control.config import ArmSettings, ConfigError, KeepOutMap
from robot_control.kinematics import KeepOut, Kinematics, first_hit, route
from robot_control.world import JointState


def tower_like():
    """Base 10..60 deg with the elbow under 85 deg blocked, like the camera tower README's map."""
    j0s, j1s = np.arange(-90, 91), np.arange(0, 142)
    blocked = (j0s[:, None] >= 10) & (j0s[:, None] <= 60) & (j1s[None, :] < 85)
    return KeepOut(KeepOutMap("tower", (-90.0, 1.0), (0.0, 1.0), blocked, 10.0))


def test_blocks():
    k = tower_like()
    assert k.blocks(30, 50)
    assert not k.blocks(30, 100)
    assert not k.blocks(-46.12, 110.42)     # over the plate centre
    assert k.blocks(9.5, 50)                # between grid points: blocked if either neighbour is
    assert not k.blocks(8.5, 50)
    assert k.blocks(30, 84.5) and not k.blocks(30, 85.0)


def test_off_the_grid_counts_as_blocked():
    assert tower_like().blocks(0, 150)


def test_moves_are_straight_lines_in_joint_space():
    k = tower_like()
    hit = k.first_hit((0, 50), (80, 50))
    assert hit is not None and hit[0] == pytest.approx(9.25, abs=0.3)
    assert k.first_hit((0, 100), (80, 100)) is None
    # Both ends safe, but the line between them dips through the zone.
    assert k.first_hit((-46, 110), (75, 40)) is not None
    assert first_hit([k], JointState(-46, 110, 0, 0.12), JointState(75, 40, 0, 0.12))[0] == "tower"


def test_route_folds_the_elbow_to_get_past():
    k = tower_like()
    start, end = JointState(-46.12, 110.42, 0.0, 0.15), JointState(75.0, 40.0, 20.0, 0.15)
    path = route([k], start, end, elbow_max=120.0)
    assert path[-1] is end
    assert [(p.j0, p.j1) for p in path[:2]] == [(-46.12, 110.42), (75.0, 110.42)]
    legs = [start, *path]
    assert all(k.first_hit(a, b) is None for a, b in zip(legs, legs[1:]))


def test_route_takes_the_smallest_fold():
    k = tower_like()
    start, end = JointState(0, 40, 0, 0.15), JointState(75, 40, 0, 0.15)
    folded = route([k], start, end, elbow_max=120.0)[0]
    assert folded.j1 == 85.0


def test_route_straight_when_clear_and_none_when_impossible():
    k = tower_like()
    a, b = JointState(-60, 100, 0, 0.15), JointState(-20, 90, 0, 0.15)
    assert route([k], a, b, 120.0) == [b]
    assert route([k], a, JointState(30, 50, 0, 0.15), 120.0) is None      # the end is in the zone


def test_ik_refuses_a_pose_in_the_zone():
    kin = Kinematics(ArmSettings.from_config(), [tower_like()])
    x, y, _ = kin.fk(30, 50)
    r = kin.ik(x, y, 0.0, 0.12)
    assert not r.ok and "tower keep-out" in r.reason
    assert kin.ik(0.0, 0.130, 0.0, 0.12).ok


def test_map_file_round_trip(tmp_path):
    path = tmp_path / "m.json"
    path.write_text(json.dumps({"name": "box", "margin_mm": 5, "j0_deg": [-1, 1, 1], "j1_deg": [0, 2, 1],
                                "blocked": ["000", "010", "000"]}))
    m = KeepOutMap.load(path)
    assert m.blocked.shape == (3, 3) and m.blocked[1, 1] and m.blocked.sum() == 1
    path.write_text(json.dumps({"name": "box", "margin_mm": 5, "j0_deg": [-1, 1, 1], "j1_deg": [0, 2, 1],
                                "blocked": ["000", "010"]}))
    with pytest.raises(ConfigError, match="doesn't match"):
        KeepOutMap.load(path)


def test_the_configured_maps_load():
    maps = KeepOutMap.from_config()
    assert [m.name for m in maps] == ["camera tower"]
    assert maps[0].blocked.any()
