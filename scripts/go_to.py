"""Sends the arm to one position with the course API (hardware_api/) and reads back where it landed.

    python scripts/go_to.py                    # x 0, y 0.2 m: base about -24 deg, elbow about 55 deg
    python scripts/go_to.py --x 0.05 --y 0.18
    python scripts/go_to.py --port COM8 --time 3

Needs firmware/ flashed, the mode switch on PROGRAM, RESET pressed after the demo, and servo power
on (5 V). Follows the course's scara_robot_test_script.py, except that it calibrates before the
first move: SCARA_INITIALISE, SCARA_AUTO_CALIBRATE (the base and elbow sweep, as at the start of the
demo), SCARA_MOVE_COORD, then SCARA_READ_ANGLE and SCARA_READ_COORD.

It calibrates first because, until it has, the firmware maps the elbow's joint 0 deg to 2500 us,
which on this arm measured about -83 deg (servo centre near elbow 7 deg, not the firmware's 90): the
course script's first SCARA_MOVE_JOINT(1, 90, 1) would swing the elbow there.
"""

import argparse
import math
import sys
from pathlib import Path

import serial.tools.list_ports

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hardware_api"))

import scara_motion_controller_api as smc_api  # noqa: E402

ST_LINK_VID = 0x0483
L1, L2 = 0.125, 0.1                  # the course values, as in scara_robot_test_script.py
Z_MIN, Z_MAX = 0.095, 0.15


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == ST_LINK_VID]
    if not ports:
        sys.exit("No Nucleo found: plug its USB cable in, or give --port.")
    return ports[0]


def ik(x, y):
    """The firmware's calc_inverse_kinematics (scara.c): (base, elbow) in degrees."""
    r = math.hypot(x, y)
    phi1 = math.acos((L2 * L2 - r * r - L1 * L1) / (-2 * r * L1))
    phi3 = math.acos((r * r - L1 * L1 - L2 * L2) / (-2 * L1 * L2))
    return math.degrees(math.atan2(x, y) - phi1), math.degrees(math.pi - phi3)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    p.add_argument("--x", type=float, default=0.0, help="metres (0)")
    p.add_argument("--y", type=float, default=0.2, help="metres (0.2)")
    p.add_argument("--time", type=float, default=2.0, help="seconds for the move (2)")
    p.add_argument("--settle", type=float, default=1.0,
                   help="settling time (1, the course value): it steps the base and elbow pulses 5 us "
                        "every 50 ms toward the target, with no limit on the pulse")
    args = p.parse_args()

    if not L2 - L1 < math.hypot(args.x, args.y) < L1 + L2:
        sys.exit("Out of reach.")
    base, elbow = ik(args.x, args.y)
    print(f"Target x {args.x:.3f} y {args.y:.3f} m: base {base:.1f} deg, elbow {elbow:.1f} deg")
    if not -85 < base < 75 or not 5 < elbow < 107:
        sys.exit("Keep to base -85..75 and elbow 5..107 deg (the elbow's stop is about 128).")
    if elbow > 90 and args.settle > 0.2:
        sys.exit("Past elbow 90 use --settle 0.2: if the elbow can't get there, a longer settle "
                 "pushes its pulse past the servo's end and leaves it there.")

    input("Servo power on at 5 V, switch on PROGRAM, RESET pressed, arm clear, hand on the plug? Enter: ")
    scara = smc_api.scara_motion_controller(port=args.port or find_port(), timeout=60)
    scara.SCARA_INITIALISE(L1, L2, Z_MIN, Z_MAX, args.settle, 1.3, 0.01, 0.001, 1.3, 0.01, 0.001)
    print("Calibrating: the base and elbow sweep end to end (as at the start of the demo)...")
    scara.SCARA_AUTO_CALIBRATE()
    print("Moving...")
    scara.SCARA_MOVE_COORD(args.x, args.y, 0.0, Z_MAX, args.time)

    got_base, got_elbow = scara.SCARA_READ_ANGLE(0), scara.SCARA_READ_ANGLE(1)
    x, y, *_ = scara.SCARA_READ_COORD()
    print(f"\nBase   wanted {base:6.1f}  got {got_base:6.1f} deg  (off {got_base - base:+.1f})")
    print(f"Elbow  wanted {elbow:6.1f}  got {got_elbow:6.1f} deg  (off {got_elbow - elbow:+.1f})")
    print(f"Tip    wanted ({args.x:.3f}, {args.y:.3f})  got ({x:.3f}, {y:.3f}) m  "
          f"(off {1000 * math.hypot(x - args.x, y - args.y):.1f} mm)")


if __name__ == "__main__":
    main()
