"""The ohmmeter: the backup for resistors the band reader isn't sure of.

The meter is an Arduino Uno running the team's ohm-meter_serial_en sketch (9600 baud). Opening the port resets
it; it prints "<Ready!>" when it is up. Each "measure" line is answered with one line: the resistance between
its two pads, or "Open circuit" / "Short circuit". With "debug on" (sent once on opening) the answer is the
raw value, "Raw: 4701.23 Ohms -> E12: 4700 Ohms  [ch=3 raw=519]"; without it, E12-snapped to 2 s.f., which
would lose E24 values, so it is always on here. The pads are on a station the arm presses the held resistor's
leads onto (ohmmeter_station.toml, scripts/teach_ohmmeter.py).

    python scripts/ohmmeter_check.py --list            # its port and USB ids
    python scripts/ohmmeter_check.py --raw             # what it sends back
    python scripts/ohmmeter_check.py                   # readings of whatever is on the pads, as the run takes them

The sorting run only uses measure(), which returns a Measurement, and close(). Anything with those two
(FakeOhmmeter, for the tests and dry runs) can stand in for the meter.
"""

import math
import re
import statistics
import time
from dataclasses import dataclass

from robot_offline.post.bands.value import E24


class OhmmeterError(Exception):
    """The meter isn't there, or stopped answering."""


@dataclass(frozen=True)
class Measurement:
    """One reading of whatever is between the contacts.

    ohms: the median of the samples (inf for an open circuit), None if there was no usable reading.
    nominal_ohm: the E24 value it is, None if it isn't close enough to one (or open, or no reading).
    why: why it has no nominal value, "" when it has one."""
    ohms: float | None
    nominal_ohm: float | None
    samples: tuple = ()
    why: str = ""

    @property
    def is_open(self):
        return self.ohms is not None and math.isinf(self.ohms)


READY = "<Ready!>"
RAW = re.compile(r"RAW:\s*([+-]?\d*\.?\d+(?:E[+-]?\d+)?)\s*OHMS", re.IGNORECASE)


def parse_reply(text):
    """The resistance in ohms from one reply line: math.inf for "Open circuit", 0.0 for "Short circuit".
    Reads the debug line ("Raw: 4701.23 Ohms -> ...") or a bare number (debug off: "4700", "4.7"). Raises
    ValueError if it can't be read."""
    t = text.strip()
    if not t:
        raise ValueError("empty reply")
    if t.lower().startswith("open circuit"):
        return math.inf
    if t.lower().startswith("short circuit"):
        return 0.0
    m = RAW.match(t)
    if m is None and re.fullmatch(r"[+-]?\d*\.?\d+(?:[eE][+-]?\d+)?", t):
        value = float(t)
    elif m is not None:
        value = float(m.group(1))
    else:
        raise ValueError(f"can't read {text!r}")
    if value < 0:
        raise ValueError(f"negative resistance {text!r}")
    return value


def nearest_e24(ohms, tolerance_pct):
    """The E24 value nearest ohms, or None if none is within tolerance_pct of it."""
    if ohms is None or not math.isfinite(ohms) or ohms <= 0:
        return None
    decade = 10.0 ** math.floor(math.log10(ohms))
    options = [d * decade / 10 for d in E24] + [10 * decade]      # this decade, and the next one's first
    best = min(options, key=lambda v: abs(math.log(ohms / v)))
    best = float(f"{best:.6g}")                                   # 4.7 * 1000 is 4700.000000000001
    return best if abs(ohms - best) <= best * tolerance_pct / 100 else None


def judge(samples, settings):
    """A Measurement from raw sample values (ohms, inf for open), as settings (OhmmeterSettings) says."""
    samples = tuple(samples)
    if not samples:
        return Measurement(None, None, samples, "no reading")
    if any(math.isinf(s) or s >= settings.open_ohm for s in samples):
        return Measurement(math.inf, None, samples, "open circuit: the leads aren't on both pads")
    if any(s == 0.0 for s in samples):
        return Measurement(0.0, None, samples, "short circuit: the pads are joined (a lead across both?)")
    median = statistics.median(samples)
    if any(abs(s - median) > median * settings.agree_pct / 100 for s in samples):
        return Measurement(median, None, samples, "unsteady: the samples disagree")
    nominal = nearest_e24(median, settings.e24_tolerance_pct)
    return Measurement(median, nominal, samples, "" if nominal is not None else "not an E24 value")


class SerialOhmmeter:
    """The meter on its serial port. settings: config.OhmmeterSettings. link: an open serial link (for tests),
    or None to open settings.port (or find it by its USB ids). Waits for it to come up and turns debug on.
    Raises OhmmeterError if it can't."""

    def __init__(self, settings, link=None):
        self.settings = settings
        self.link = link if link is not None else self._open()
        self.wake()

    def _open(self):
        import serial
        s = self.settings
        port = s.port or find_port(s.usb_vid, s.usb_pid)
        try:
            return serial.Serial(port, s.baud, timeout=s.timeout_s)
        except serial.SerialException as why:
            raise OhmmeterError(f"can't open the ohmmeter on {port}: {why}") from None

    def readline(self):
        return self.link.readline().decode(errors="replace").strip()

    def wake(self):
        """Waits up to ready_s for "<Ready!>" (opening the port resets the Uno), then "debug on". Raises
        OhmmeterError if debug isn't answered: it isn't the ohmmeter sketch."""
        deadline = time.monotonic() + self.settings.ready_s
        while time.monotonic() < deadline:
            line = self.readline()
            if line == READY:
                break
            if not line:            # a timeout with no banner: it was already up (no reset), carry on
                break
        self.link.reset_input_buffer()
        self.link.write(b"debug on\n")
        self.link.flush()
        reply = self.readline()
        if reply != "Debug on":
            raise OhmmeterError(f"the ohmmeter answered 'debug on' with {reply!r}: is it running ohm-meter_serial_en?")

    def ask(self):
        """One "measure" and its reply line, as the meter sent it. Raises OhmmeterError on no reply."""
        self.link.reset_input_buffer()
        self.link.write(b"measure\n")
        self.link.flush()
        reply = self.readline()
        if not reply:
            raise OhmmeterError(f"no reply to 'measure' within {self.settings.timeout_s:g} s")
        return reply

    def read_ohms(self):
        """One sample, ohms (inf for open, 0 for short). Raises OhmmeterError if there is no reply or it can't
        be read."""
        reply = self.ask()
        try:
            return parse_reply(reply)
        except ValueError as why:
            raise OhmmeterError(f"the meter's reply {reply!r}: {why}") from None

    def measure(self, settle=True):
        """A Measurement of what is on the pads now: settings.samples readings after settle_s (none if not
        settle). Raises OhmmeterError if the meter doesn't answer (the run stops: it can't measure any more)."""
        if settle:
            time.sleep(self.settings.settle_s)
        return judge([self.read_ohms() for _ in range(self.settings.samples)], self.settings)

    def close(self):
        self.link.close()


class FakeOhmmeter:
    """Stands in for the meter: answers each measure() with the next of readings (ohms, math.inf for open,
    None for no reading, or an Exception to raise), then keeps giving the last one."""

    def __init__(self, readings, settings=None):
        from robot_control.config import OhmmeterSettings
        self.readings = list(readings)
        self.settings = settings or OhmmeterSettings(enabled=True)
        self.measured = 0

    def measure(self, settle=True):
        value = self.readings[min(self.measured, len(self.readings) - 1)]
        self.measured += 1
        if isinstance(value, Exception):
            raise value
        return judge([] if value is None else [value], self.settings)

    def close(self):
        pass


def find_port(vid, pid):
    """The meter's port, by its USB ids. Raises OhmmeterError unless exactly one device matches."""
    from serial.tools import list_ports
    if not vid:
        raise OhmmeterError("no ohmmeter port: set port (or usb_vid and usb_pid) in robot_control/config/ohmmeter.toml "
                            "(python scripts/ohmmeter_check.py --list shows what is plugged in)")
    found = [p.device for p in list_ports.comports() if p.vid == vid and (not pid or p.pid == pid)]
    if len(found) != 1:
        raise OhmmeterError(f"expected one ohmmeter (USB {vid:04x}:{pid:04x}), found {found or 'none'}")
    return found[0]


def from_config(settings=None):
    """The meter ohmmeter.toml describes, or None when it is off. Raises OhmmeterError if it is on but
    can't be opened."""
    from robot_control.config import OhmmeterSettings
    settings = settings or OhmmeterSettings.from_config()
    return SerialOhmmeter(settings) if settings.enabled else None
