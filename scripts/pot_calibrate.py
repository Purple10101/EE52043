"""The setup guide's potentiometer calibration (course_files/lab_scripts/RobotSetup.md, steps 6-9),
without STM32CubeIDE: reads 'adc_raw' over USB instead of the debugger's live expressions.

    python scripts/pot_calibrate.py              # finds the Nucleo's port
    python scripts/pot_calibrate.py --port COM8

Needs firmware/ built from the current firmware/Core/Src/potentiometers.c and flashed, with
'#define API' on and the mode switch on PROGRAM. Servo power OFF (step 1). It sends READ_ANGLE
(command 6) and works the raw value back out of the angle with the calibration in
potentiometers.c: the angle is computed from the same pot_raw_value the debugger shows as adc_raw.

Asks for the guide's four marks in its order, the median of 50 readings each:
  6. Axis 0 at -90 deg (anti-clockwise)      -> pots[0].min_raw_value
  7. Axis 0 at +90 deg                       -> pots[0].max_raw_value
  8. Axis 1 at 0 deg (link 2 in line)        -> pots[1].min_raw_value
  9. Axis 1 at +90 deg (clockwise)           -> pots[1].max_raw_value
then prints the values for potentiometers_init() and saves them to logs/pot_calibrate_<time>.csv.

Only reads: nothing is sent that could move the arm.
"""

import argparse
import csv
import re
import statistics
import struct
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

ROOT = Path(__file__).resolve().parents[1]
POTS_C = ROOT / "firmware" / "Core" / "Src" / "potentiometers.c"
ST_LINK_VID = 0x0483

STEPS = [(6, 0, "min_raw_value", "Axis 0 (base) at -90 deg (anti-clockwise), by the markings"),
         (7, 0, "max_raw_value", "Axis 0 (base) at +90 deg"),
         (8, 1, "min_raw_value", "Axis 1 (elbow) at 0 deg (link 2 in line with link 1)"),
         (9, 1, "max_raw_value", "Axis 1 (elbow) at +90 deg (clockwise)")]


def flashed_calibration():
    """{(pot, field): value} from potentiometers_init(), skipping commented-out lines."""
    cal = {}
    for line in POTS_C.read_text().splitlines():
        m = re.match(r"\s*pots\[(\d)\]\.(\w+)\s*=\s*(-?[\d.]+)\s*;", line)
        if m:
            cal[int(m[1]), m[2]] = float(m[3])
    return cal


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == ST_LINK_VID]
    if not ports:
        sys.exit("No Nucleo found: plug its USB cable in, or give --port.")
    return ports[0]


def read_raw(ser, pot, cal):
    ser.reset_input_buffer()
    ser.write(struct.pack("<ii", 6, pot) + bytes(40))
    ser.flush()
    r = ser.read(8)
    if len(r) != 8 or struct.unpack("<i", r[:4])[0] != 1:
        return None
    angle = struct.unpack("<f", r[4:])[0]
    lo, hi = cal[pot, "min_raw_value"], cal[pot, "max_raw_value"]
    a0, a1 = cal[pot, "min_angle"], cal[pot, "max_angle"]
    return lo + (angle - a0) * (hi - lo) / (a1 - a0)


def sample(ser, pot, cal, n=50):
    values = [v for v in (read_raw(ser, pot, cal) for _ in range(n)) if v is not None]
    if not values:
        return None, None
    return statistics.median(values), max(values) - min(values)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    args = p.parse_args()

    cal = flashed_calibration()
    ser = serial.Serial(args.port or find_port(), 115200, timeout=0.5)
    time.sleep(0.2)
    if read_raw(ser, 0, cal) is None:
        sys.exit("No reply to READ_ANGLE: is firmware/ flashed with '#define API' on, and the switch on PROGRAM?")

    rows = []
    for step, pot, field, where in STEPS:
        while True:
            if input(f"Step {step}: hold {where}, then press Enter (q to stop): ").strip().lower() == "q":
                return
            raw, spread = sample(ser, pot, cal)
            if raw is not None:
                break
            print("  no reply, try again")
        print(f"  adc_raw[{pot}] = {raw:.0f}  (spread {spread:.0f})")
        if raw < 20 or raw > 4075:
            print("  that is the end of the ADC's range (0-4095): check the potentiometer's wiring")
        rows.append((step, pot, field, round(raw), round(spread)))

    print("\nFor potentiometers_init() in firmware/Core/Src/potentiometers.c:")
    for _, pot, field, raw, _ in rows:
        print(f"    pots[{pot}].{field} = {raw}.0;")
    out = ROOT / "logs" / f"pot_calibrate_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    out.parent.mkdir(exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "pot", "field", "adc_raw", "spread"])
        w.writerows(rows)
    print(f"Saved {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
