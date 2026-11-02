"""Live potentiometer readout, for checking the arm's angles by hand with the servos unpowered
(step 1 of firmware/README.md "On the arm").

    python scripts/pot_check.py              # finds the Nucleo's port; Ctrl+C to stop
    python scripts/pot_check.py --port COM8

Shows the base and elbow angles a few times a second, as the firmware reads them, with the elbow's
smallest and largest so far. Move the arm gently by hand: elbow straight out, elbow square (90 deg),
then fold the elbow slowly until the links (or anything else) touch, and hold it there a moment.
Every reading goes to logs/pot_check_<time>.csv.

Only reads (command 6, READ_ANGLE): nothing is sent that could move the arm. Works with the course
firmware and the lab builds.
"""

import argparse
import csv
import struct
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

ROOT = Path(__file__).resolve().parents[1]
ST_LINK_VID = 0x0483


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == ST_LINK_VID]
    if not ports:
        sys.exit("No Nucleo found: plug its USB cable in, or give --port.")
    return ports[0]


def read_once(ser, axis):
    ser.reset_input_buffer()
    ser.write(struct.pack("<ii", 6, axis) + bytes(40))
    ser.flush()
    r = ser.read(8)
    if len(r) != 8 or struct.unpack("<i", r[:4])[0] != 1:
        return None
    return struct.unpack("<f", r[4:])[0]


def read_angle(ser, axis, n=5):
    """The median of n readings: a single one jitters by a couple of degrees."""
    values = sorted(v for v in (read_once(ser, axis) for _ in range(n)) if v is not None)
    return values[len(values) // 2] if values else None


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    p.add_argument("--seconds", type=float, default=0, help="stop after this long (default: Ctrl+C)")
    args = p.parse_args()

    port = args.port or find_port()
    ser = serial.Serial(port, 115200, timeout=0.5)
    time.sleep(0.5)
    if read_angle(ser, 1) is None:
        sys.exit(f"No answer on {port}: press the Nucleo's black RESET button and try again.")

    log_dir = ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"pot_check_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    lo = hi = None
    t0 = time.time()
    print(f"Reading {port}; logging to {path.relative_to(ROOT)}. Ctrl+C to stop.\n")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "base_deg", "elbow_deg"])
        try:
            while not args.seconds or time.time() - t0 < args.seconds:
                base, elbow = read_angle(ser, 0), read_angle(ser, 1)
                t = time.time() - t0
                w.writerow([f"{t:.2f}", "" if base is None else f"{base:.2f}", "" if elbow is None else f"{elbow:.2f}"])
                if elbow is not None:
                    lo = elbow if lo is None else min(lo, elbow)
                    hi = elbow if hi is None else max(hi, elbow)
                    b = "  ---  " if base is None else f"{base:7.1f}"
                    print(f"\r  base {b} deg   elbow {elbow:6.1f} deg   (elbow smallest {lo:6.1f}, largest {hi:6.1f})   ",
                          end="", flush=True)
                time.sleep(0.15)
        except KeyboardInterrupt:
            pass
    ser.close()
    print(f"\n\nElbow smallest {lo:.1f} deg, largest {hi:.1f} deg. Log: {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
