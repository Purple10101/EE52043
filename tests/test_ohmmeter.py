"""The ohmmeter: reading its replies, placing a reading on E24, the serial link (faked), its station."""

import math

import pytest

pytest.importorskip("robot_offline.post.bands.value")

from robot_control.config import ConfigError, OhmmeterSettings  # noqa: E402
from robot_control.hardware.ohmmeter import (FakeOhmmeter, OhmmeterError, SerialOhmmeter, judge,  # noqa: E402
                                             nearest_e24, parse_reply)

SETTINGS = OhmmeterSettings(enabled=True, samples=3, settle_s=0.0)


@pytest.mark.parametrize("reply, ohms", [
    ("Raw: 4701.23 Ohms -> E12: 4700 Ohms  [ch=3 raw=519]", 4701.23),
    ("Raw: 9.87 Ohms -> ~9.9 Ohms (no E12 match)  [ch=0 raw=978]\r\n", 9.87),
    ("4700", 4700.0), ("4.7", 4.7), ("Short circuit", 0.0),
])
def test_replies_are_read_in_ohms(reply, ohms):
    assert parse_reply(reply) == pytest.approx(ohms)


def test_open_circuit_is_infinite():
    assert math.isinf(parse_reply("Open circuit\r\n"))


@pytest.mark.parametrize("reply", ["", "Debug on", "Unknown command (try: measure, debug on, debug off)", "-12.0"])
def test_a_reply_that_isnt_a_resistance_is_refused(reply):
    with pytest.raises(ValueError):
        parse_reply(reply)


def test_a_reading_lands_on_the_nearest_e24_value_within_tolerance():
    assert nearest_e24(4683.0, 6) == 4700
    assert nearest_e24(9.8, 6) == 10
    assert nearest_e24(9950.0, 6) == 10_000          # the next decade's first value
    assert nearest_e24(1.02e3, 6) == 1000
    assert nearest_e24(1400.0, 3) is None            # between 1k3 and 1k5
    assert nearest_e24(math.inf, 6) is None and nearest_e24(None, 6) is None


def test_judging_samples():
    m = judge([4690.0, 4701.0, 4695.0], SETTINGS)
    assert (m.ohms, m.nominal_ohm, m.why) == (4695.0, 4700, "")
    assert judge([4700.0, math.inf, 4700.0], SETTINGS).is_open
    assert judge([2e7], SETTINGS).is_open                         # above open_ohm
    assert judge([0.0, 0.0, 0.0], SETTINGS).why.startswith("short circuit")
    assert judge([4700.0, 5200.0, 4710.0], SETTINGS).why.startswith("unsteady")
    assert judge([1400.0], SETTINGS).why == "not an E24 value"
    assert judge([], SETTINGS).nominal_ohm is None


class FakeLink:
    """A serial link whose meter answers each line sent with the next of replies (the first ones: its
    banner and the answer to "debug on")."""

    def __init__(self, *replies):
        self.replies, self.sent, self.closed = list(replies), [], False
        self.port = "COM99"

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.sent.append(data)

    def flush(self):
        pass

    def readline(self):
        return (self.replies.pop(0) + "\n").encode() if self.replies else b""

    def close(self):
        self.closed = True


def up(*replies):
    return FakeLink("<Ready!>", "Debug on", *replies)


def test_the_serial_meter_turns_debug_on_measures_once_per_sample_and_takes_the_median():
    link = up("Raw: 4690.0 Ohms -> E12: 4700 Ohms", "Raw: 4710.0 Ohms -> E12: 4700 Ohms",
              "Raw: 4700.0 Ohms -> E12: 4700 Ohms")
    m = SerialOhmmeter(SETTINGS, link).measure()
    assert link.sent == [b"debug on\n"] + [b"measure\n"] * 3
    assert (m.ohms, m.nominal_ohm) == (4700.0, 4700)


def test_open_and_short_are_told_apart():
    assert SerialOhmmeter(SETTINGS, up(*["Open circuit"] * 3)).measure().is_open
    assert SerialOhmmeter(SETTINGS, up(*["Short circuit"] * 3)).measure().why.startswith("short circuit")


def test_something_that_isnt_the_ohmmeter_sketch_is_refused():
    with pytest.raises(OhmmeterError, match="debug on"):
        SerialOhmmeter(SETTINGS, FakeLink("<Ready!>", "hello"))


def test_a_meter_that_stops_answering_or_talks_nonsense_raises():
    with pytest.raises(OhmmeterError, match="no reply"):
        SerialOhmmeter(SETTINGS, up()).measure()
    with pytest.raises(OhmmeterError, match="This just in"):
        SerialOhmmeter(SETTINGS, up("This just in ... measure")).measure()


def test_the_fake_gives_its_readings_in_turn_then_keeps_the_last():
    meter = FakeOhmmeter([1500.0, math.inf, None, OhmmeterError("unplugged")])
    assert meter.measure().nominal_ohm == 1500
    assert meter.measure().is_open
    assert meter.measure().why == "no reading"
    for _ in range(2):
        with pytest.raises(OhmmeterError):
            meter.measure()


def test_the_config_file_loads_with_its_station():
    s = OhmmeterSettings.from_config()
    assert s.station is not None or s.enabled is False
    assert s.measure in ("unsure", "all") and s.reject_bin and s.usb_vid == 0x2341


def write_config(tmp_path, enabled, station=None):
    from robot_control.config import CONFIG_DIR
    text = (CONFIG_DIR / "ohmmeter.toml").read_text(encoding="utf-8")
    (tmp_path / "ohmmeter.toml").write_text(text.replace("enabled = false", f"enabled = {str(enabled).lower()}"),
                                            encoding="utf-8")
    if station is not None:
        (tmp_path / "ohmmeter_station.toml").write_text(station, encoding="utf-8")


def test_the_meter_cant_be_turned_on_before_its_station_is_taught(tmp_path):
    write_config(tmp_path, True)
    with pytest.raises(ConfigError, match="isn't taught"):
        OhmmeterSettings.from_config(tmp_path)


def test_a_taught_station_loads(tmp_path):
    write_config(tmp_path, True, STATION_TOML)
    st = OhmmeterSettings.from_config(tmp_path).station
    assert (st.centre_mm, st.lead_axis_deg, st.contact_slide, st.measure_slide) == ((80.5, -40.0), 12.0, 44.0, 45.5)


def test_a_station_measuring_above_its_contact_is_refused(tmp_path):
    write_config(tmp_path, False, STATION_TOML.replace("measure_slide = 45.5", "measure_slide = 40"))
    with pytest.raises(ConfigError, match="measure_slide"):
        OhmmeterSettings.from_config(tmp_path)


STATION_TOML = """
centre_mm = [80.5, -40.0]
lead_axis_deg = 12.0
contact_slide = 44.0
measure_slide = 45.5
"""
