"""Talks to the ohmmeter on its own, to bring it up before the sorting run uses it. Nothing else moves.

    python scripts/ohmmeter_check.py --list             # every serial port, with its USB ids
    python scripts/ohmmeter_check.py --raw              # sends "measure", prints the meter's reply as it comes
    python scripts/ohmmeter_check.py                    # 5 measurements, as the run would take them
    python scripts/ohmmeter_check.py --port COM12 --count 20

Settings come from robot_control/config/ohmmeter.toml (port, baud...); --port and --baud override them.
It works with enabled = false there, so the meter can be checked before the run is allowed to use it.
Hold a known resistor's leads on the pads: the reading should land on its E24 value.
"""

import argparse
import dataclasses
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_control.config import OhmmeterSettings  # noqa: E402
from robot_control.hardware.ohmmeter import OhmmeterError, SerialOhmmeter  # noqa: E402


def list_ports():
    from serial.tools import list_ports as lp
    ports = lp.comports()
    if not ports:
        print("no serial ports")
    for p in ports:
        ids = f"usb_vid = 0x{p.vid:04x}, usb_pid = 0x{p.pid:04x}" if p.vid is not None else "not USB"
        print(f"{p.device:8} {ids:40} {p.description}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--list", action="store_true", help="list the serial ports and stop")
    p.add_argument("--raw", action="store_true", help="print the meter's raw replies, unparsed")
    p.add_argument("--port", help="the meter's port (default: ohmmeter.toml)")
    p.add_argument("--baud", type=int, help="the link's baud rate (default: ohmmeter.toml)")
    p.add_argument("--count", type=int, default=5, help="how many queries or measurements (default 5)")
    args = p.parse_args()
    if args.list:
        list_ports()
        return
    settings = OhmmeterSettings.from_config()
    changes = {k: v for k, v in (("port", args.port), ("baud", args.baud)) if v}
    settings = dataclasses.replace(settings, **changes)
    try:
        meter = SerialOhmmeter(settings)
    except OhmmeterError as why:
        sys.exit(str(why))
    print(f"ohmmeter on {meter.link.port} at {settings.baud} baud")
    try:
        for i in range(1, args.count + 1):
            if args.raw:
                print(f"{i}: {meter.ask()!r}")
                continue
            m = meter.measure()
            ohms = "none" if m.ohms is None else "open" if m.is_open else f"{m.ohms:.2f} ohm"
            nominal = f"-> {m.nominal_ohm:g} ohm" if m.nominal_ohm is not None else f"-> not placed: {m.why}"
            print(f"{i}: {ohms} {nominal}   samples {[round(s, 2) for s in m.samples]}")
    except OhmmeterError as why:
        sys.exit(f"stopped: {why}")
    finally:
        meter.close()


if __name__ == "__main__":
    main()
