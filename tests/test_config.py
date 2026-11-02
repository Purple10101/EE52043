import pytest

from robot_control import config


def test_every_file_loads():
    arm = config.ArmSettings.from_config()
    assert arm.j1 == (0.0, 139.0)
    config.RigSettings.from_config()
    ws = config.WorkspaceSettings.from_config()
    assert {p.name for p in ws.packages} >= {"1/4W"}
    assert config.GripperSettings.from_config().tool_offset == (0.0, 0.0)
    config.VisionSettings.from_config()


def test_missing_key_names_file_and_key(tmp_path):
    (tmp_path / "robot_real.toml").write_text("[arm]\nlink1_m = 0.125\nlink2_m = 0.1\n")
    with pytest.raises(config.ConfigError, match=r"robot_real.toml: missing key 'limits'"):
        t = config.load("robot_real", tmp_path)
        t["limits"]


def test_min_above_max_fails(tmp_path):
    (tmp_path / "x.toml").write_text("[limits]\nj1_deg = [120, 0]\n")
    with pytest.raises(config.ConfigError, match="min < max"):
        config.load("x", tmp_path)["limits"].range("j1_deg")
