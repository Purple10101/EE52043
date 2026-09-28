"""Smoke test for the Unity SCARA simulator: connect, then sweep each joint through its range.

Start the simulator first (open Assets/Scenes/main_scene, press Play, click Start Server), then
run from the repo root:

    python -m robot_control.sim_joint_test
"""

import argparse
import socket
import sys

from hardware_api.sim_scara_motion_controller_api import sim_scara_motion_controller

# Joint limits the simulator checks against (sim_scara_controller_client.cs).
# Axes 0-2 are rotations in degrees; axis 3 is the Z slide in metres.
JOINT_LIMITS = {
    0: (-90.0, 90.0),
    1: (0.0, 120.0),
    2: (-180.0, 180.0),
    3: (0.025, 0.075),
}
JOINT_NAMES = {0: "shoulder", 1: "elbow", 2: "wrist", 3: "z slide"}

# Neutral pose, matching the course's real-robot test script (axis 1 at 90 degrees)
HOME = (0.0, 90.0, 0.0, 0.05)

# The simulator only replies once a move and its settling time are done, so allow plenty
REPLY_TIMEOUT_S = 30.0


def check_limits(axis, value):
    low, high = JOINT_LIMITS[axis]
    if not low <= value <= high:
        raise ValueError(f"axis {axis} target {value} is outside the simulator limits [{low}, {high}]")


def move_joint(scara, axis, value, move_time):
    check_limits(axis, value)
    print(f"axis {axis} ({JOINT_NAMES[axis]}) -> {value}")
    scara.SCARA_MOVE_JOINT(axis, value, move_time)


def move_home(scara, move_time):
    for axis, value in enumerate(HOME):
        check_limits(axis, value)
    print(f"home -> {HOME}")
    scara.SCARA_MOVE_JOINTS(*HOME, move_time)


def run(host, port, move_time):
    try:
        scara = sim_scara_motion_controller(host=host, port=port)
    except ConnectionRefusedError:
        print(f"could not connect to {host}:{port} - is the simulator running with main_scene "
              "open and the server started?")
        return 1

    scara.client.settimeout(REPLY_TIMEOUT_S)
    try:
        move_home(scara, move_time)
        for axis, (low, high) in JOINT_LIMITS.items():
            move_joint(scara, axis, low, move_time)
            move_joint(scara, axis, high, move_time)
            move_joint(scara, axis, HOME[axis], move_time)
    except socket.timeout:
        print(f"no reply from the simulator within {REPLY_TIMEOUT_S} s")
        return 1
    finally:
        scara.client.close()

    print("joint test complete")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--time", type=float, default=1.0, help="seconds per move (default 1)")
    args = parser.parse_args()
    return run(args.host, args.port, args.time)


if __name__ == "__main__":
    sys.exit(main())
