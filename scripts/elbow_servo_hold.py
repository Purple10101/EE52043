"""Holds the elbow servo at its clockwise end (500 us) or its centre (1500 us), for refitting the elbow
link on the servo's horn.

    python scripts/elbow_servo_hold.py                  # clockwise end, 500 us
    python scripts/elbow_servo_hold.py --to centre      # 1500 us
    python scripts/elbow_servo_hold.py --port COM8

On this arm the elbow stopped at about 100 deg (2026-11-02): the servo ran out of travel. Refitted
with the servo at its clockwise end (the elbow's + direction is clockwise, setup guide step 9) and the
link folded clockwise as far as it goes, the servo's travel (about 185 deg) covers the elbow from its
stop back past 0.

Uses only the course API. SCARA_INITIALISE with every PID gain and the settling time 0 turns the
potentiometer loop off (with the link off the horn the potentiometer doesn't follow the servo), so
SCARA_MOVE_JOINT(1, a, 2) leaves the pulse where the firmware's defaults put joint a: 120 -> 500 us,
90 -> 1500 us. That holds only straight after a RESET: SCARA_AUTO_CALIBRATE (the demo, go_to.py)
changes the mapping.

The move starts from the firmware's power-on elbow position, joint 0 (2500 us), so the servo first
swings to its other end: run it with the elbow link OFF the servo's horn. RESET (or unplugging servo
power) afterwards leaves the servo limp: the firmware sends no pulses until a move.
"""

import argparse
import contextlib
import io
import sys
from pathlib import Path

import serial.tools.list_ports

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hardware_api"))

import scara_motion_controller_api as smc_api  # noqa: E402

ST_LINK_VID = 0x0483
JOINT = {"clockwise": 120.0, "centre": 90.0}      # joint angle -> 500 / 1500 us with the defaults


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == ST_LINK_VID]
    if not ports:
        sys.exit("No Nucleo found: plug its USB cable in, or give --port.")
    return ports[0]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--to", choices=JOINT, default="clockwise", help="where to hold the servo (clockwise)")
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    args = p.parse_args()

    input("Elbow link OFF the servo horn, RESET pressed (switch on PROGRAM), servo power on at 5 V? Enter: ")
    scara = smc_api.scara_motion_controller(port=args.port or find_port(), timeout=10)
    with contextlib.redirect_stdout(io.StringIO()):
        # READ_COORD's z and wrist come from the slide's and wrist's last commanded angles, not sensors:
        # after a RESET they are 0, z 0.128 m. Anything else means a move (go_to.py, the demo) since.
        _, _, angle_z, z = scara.SCARA_READ_COORD()
        wrist = angle_z - scara.SCARA_READ_ANGLE(0) - scara.SCARA_READ_ANGLE(1)
    if abs(z - 0.128) > 0.001 or abs(wrist) > 0.5:
        sys.exit("The board hasn't been reset since a move, so the firmware's mapping isn't the default "
                 "and the servo won't go to its end: press RESET (the black button on the Nucleo) and run again.")
    with contextlib.redirect_stdout(io.StringIO()):
        scara.SCARA_INITIALISE(0.125, 0.1, 0.095, 0.15, 0, 0, 0, 0, 0, 0, 0)
        scara.SCARA_MOVE_JOINT(1, JOINT[args.to], 2)
    if args.to == "clockwise":
        print("The elbow servo is holding 500 us, its clockwise end. Fold the elbow link clockwise as far")
        print("as it goes and push it onto the horn at the nearest tooth that doesn't make it press on the stop.")
    else:
        print("The elbow servo is holding 1500 us, its centre. Set the elbow at 90 deg and push the link")
        print("onto the horn at the nearest tooth (if between two, the one with the elbow a little straighter).")
    print("Then press RESET (or unplug servo power): the servo goes limp.")


if __name__ == "__main__":
    main()
