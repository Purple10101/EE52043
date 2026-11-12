"""The taught pick calibration (vision/pick_fit.py): fits recover a known correction from pairs made with it."""

import numpy as np
import pytest

from robot_control.config import ArmSettings, RigSettings
from robot_control.kinematics.arm import Kinematics
from robot_control.vision import pick_fit
from robot_control.vision.guide import plan_jaws

K = Kinematics(ArmSettings.from_config())
WRISTS = (-90.0, 120.0)
SPOTS = [((-13.9, -149.9), -34), ((18.0, -111.3), -34), ((-26.9, -104.4), -81), ((8.2, -139.1), -9),
         ((30.6, -155.6), -174), ((-31.7, -170.0), -180), ((40.7, -129.7), 40), ((0.0, -125.0), 60)]


def pairs_for(truth):
    """The potentiometers and wrist that centre the jaws on each spot, if truth is the real geometry."""
    joints, wrists = [], []
    for centre, body in SPOTS:
        b, e, w = plan_jaws(centre, body, WRISTS, K, truth.rig, 50.0, jaw_offset=truth.jaw_offset)
        joints.append((b, e))
        wrists.append(w)
    return np.array([c for c, _ in SPOTS]), np.array(joints), np.array(wrists)


def test_the_cad_geometry_itself_fits_with_nothing_to_correct():
    cad = RigSettings.from_config()
    truth = pick_fit.PickCalibration("rigid+jaws", cad.rotation_deg, cad.translation_m, (0.0, 0.0))
    centres, joints, wrists = pairs_for(truth)
    assert np.abs(pick_fit.cad_errors(centres, joints, wrists, K)).max() < 0.05


@pytest.mark.parametrize("model", ["shift", "rigid", "rigid+jaws"])
def test_each_model_recovers_what_it_can_describe(model):
    cad = RigSettings.from_config()
    rotation = cad.rotation_deg + (0.0 if model == "shift" else 0.6)
    jaw = (2.0, -3.0) if model == "rigid+jaws" else (0.0, 0.0)
    truth = pick_fit.PickCalibration(model, rotation, (0.0012, -0.0008), jaw)
    centres, joints, wrists = pairs_for(truth)
    cal, errors = pick_fit.fit(model, centres, joints, wrists, K)
    assert cal.rms_mm < 0.05
    assert cal.rotation_deg == pytest.approx(rotation, abs=0.02)
    assert np.allclose(cal.translation_m, truth.translation_m, atol=2e-5)
    assert np.allclose(cal.jaw_mm, jaw, atol=0.05)


def test_choose_keeps_the_jaws_model_only_when_the_jaws_are_off_the_shaft():
    cad = RigSettings.from_config()
    off = pick_fit.PickCalibration("rigid+jaws", cad.rotation_deg, (0.001, 0.0), (3.0, 1.0))
    on = pick_fit.PickCalibration("shift", cad.rotation_deg, (0.001, 0.0), (0.0, 0.0))
    quiet = lambda text: None
    assert pick_fit.choose(*pairs_for(off), K, say=quiet).model == "rigid+jaws"
    assert pick_fit.choose(*pairs_for(on), K, say=quiet).model == "shift"


def test_a_plan_by_the_fit_puts_the_jaws_centre_on_the_body(tmp_path):
    cad = RigSettings.from_config()
    truth = pick_fit.PickCalibration("rigid+jaws", cad.rotation_deg + 0.4, (0.0005, 0.001), (1.5, -2.5))
    cal, _ = pick_fit.fit("rigid+jaws", *pairs_for(truth), K)
    cal.save(tmp_path / "pick.json")
    loaded = pick_fit.load(tmp_path / "pick.json")
    target, body = (5.0, -140.0), 20
    b, e, w = plan_jaws(target, body, WRISTS, K, loaded.rig, 50.0, jaw_offset=loaded.jaw_offset)
    landed = pick_fit.predict(truth.rotation_deg, [1000 * v for v in truth.translation_m], truth.jaw_mm,
                              np.array([(b, e)]), np.array([w]), K)[0]
    assert np.hypot(*(landed - target)) < 0.1
