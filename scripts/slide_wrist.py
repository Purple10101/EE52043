"""Finds out how the slide and wrist behave, one bounded move per run, with the camera watching.

    python scripts/slide_wrist.py status              # reads only: nothing moves
    python scripts/slide_wrist.py start               # after RESET: SCARA_INITIALISE, SCARA_AUTO_CALIBRATE
    python scripts/slide_wrist.py move slide 0        # one joint, to a firmware angle in degrees
    python scripts/slide_wrist.py move base -45
    python scripts/slide_wrist.py park                # out of the camera's way, on the side away from the bins

Run scripts/tip_view.py --snapshot alongside: it holds the camera, and logs/tips/live.jpg shows what
each move did. A run opens the port and leaves the firmware as it is, so moves can follow one another
with a look at the picture in between. Every move is added to logs/slide_wrist_<date>.csv.

The slide and wrist have no potentiometers, and are limp after a RESET. The firmware starts every move
from its last position, which is 0 after a RESET, so whatever the target, the first pulse sends a limp
joint straight to 0 at full speed. So the first move of each must be to 0, made with the gripper clear
of the plate; after that they ramp, and each move here is at most MAX_STEP (the slide only going down:
up, it goes in one move). SAFE is widened as the
real travel is found. Only start, move and park drive the arm. RESET needs a new start, which clears
the last positions kept in logs/slide_wrist_state.json.

The tip is low enough to hit the plate below the hover, and the base's arc at elbow 110 runs over the
crosshair, so no base or elbow move may pass over the plate unless the slide is up at HOVER_SLIDE, and
the slide may not come down over it past OVER_PLATE_SLIDE or take its first move there.

The non-resistor bin stands along the plate's +X edge, under the park pose, and its back wall rises 6 mm
above the plate. So with the wrist out past that edge (BIN_MM) the slide stays fully up, at PARK_SLIDE:
it may only go up there, and no base or elbow move may pass over the bin with it any lower. park does
things in the only order all of that allows: after a start the elbow out of the tower's keep-out and
the slide's first move beside the plate; then the slide fully up, the elbow, and the swing across.
Leaving the park, the arm swings out over the plate with the slide still fully up, and lowers there.

Before start, with servo power off: the slide raised (the elbow sweeps out over the plate at base 0),
the elbow folded to 100 or more (the base sweeps with the elbow limp, and folded it clears the tower
at every base angle) and the SG90 turned away from the tower (the elbow sweep crosses the CAD keep-out
at 76..88, with the wrist limp).
"""

import argparse
import contextlib
import csv
import io
import json
import statistics
import sys
import time
from pathlib import Path

import serial.tools.list_ports

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "hardware_api"))

import scara_motion_controller_api as smc_api  # noqa: E402

from robot_control.config import ArmSettings, KeepOutMap, RigSettings  # noqa: E402
from robot_control.kinematics.arm import Kinematics  # noqa: E402
from robot_control.kinematics.keep_out import KeepOut, first_hit  # noqa: E402
from robot_control.vision.rig import robot_to_scene  # noqa: E402

ST_LINK_VID = 0x0483
AXES = {"base": 0, "elbow": 1, "wrist": 2, "slide": 3}
# Degrees, widened as the real travel is found. The slide's negative angles are up (seen 2026-11-05),
# as z_max maps to -60; the tip is about 25 mm above the plate at -10, about 0.3 mm a degree.
SAFE = {"wrist": (-90.0, 120.0), "slide": (-50.0, 63.0)}
MAX_STEP = {"wrist": 180.0, "slide": 30.0}    # after the first move the firmware ramps them
# The slide's step limit is for going down, toward the plate, where it sticks and jumps (approach a height
# going down). Going up moves away from everything, so it goes in one move: each move costs about 2 s (its
# time and the firmware's settling), and the climbs in a cycle took 7 moves in 30 deg steps (2026-11-11).
ELBOW_MAX = 136.0           # the stop is 141; it reached 137.7 on 2026-11-02. SCARA_AUTO_CALIBRATE only measures it to 120
GLIDE_SLIDE = 45.0          # about 9 mm above the plate: a move may cross it this low, clear of a body 2.4 mm tall
HOVER_SLIDE = 20.0          # the tip about 16 mm above the plate (camera and eye, 2026-11-05): travel height
OVER_PLATE_SLIDE = 63.0     # the bottom level: the tip almost on the plate, set by eye (2026-11-05). Reach it going down
NUDGE_DEG = 4.0             # a base or elbow move this small may be made low over the plate: camera guidance
PLATE_MM = ((-50.0, 50.0), (-180.0, -80.0))     # scene x and y of the plate's top
PLATE_MARGIN_MM = 40.0      # the gripper's reach from its axis, the SG90 side, with some to spare
PARK = {"base": -77.0, "elbow": 110.0}          # beside the plate, away from the bins: the plate fit works here
PARK_SLIDE = SAFE["slide"][0]   # fully up (the tip roughly 37-40 mm above the plate): the park is over the non-resistor bin
# The wrist out past the plate's +X edge, over the non-resistor bin (its CAD walls x 50.5..85, y -180..-80, plus
# 10 mm along the edge): scene x beyond, y between. Robot Resources/non-res-bin has the bin's CAD.
BIN_MM = (50.0, (-190.0, -70.0))
PARK_TOLERANCE_DEG = 1.5
MOVE_S = 2.0
SWING_S = 3.0               # a base move across the plate
SLIDE_STEP_S = 1.0
QUICK_READS = 5             # potentiometer readings for the checks around a move; measuring takes the default
WRIST_START = -60.0         # the wrist has about 30 deg of play: every target is reached going up from here
WRIST_TURN_S = 2.0
LAND_TOLERANCE_DEG = 0.35   # the firmware stops within about 0.5 deg of a target, often short: land() does better
LAND_GAIN = 0.7             # of the error, each try: more overshoots and has to come back
LAND_TRIES = 5
LAND_MAX_DEG = 2.0          # the most land() adds to a target: more means the joint isn't following
LAND_S = 1.0
SETTLE_S = 1.0              # the course's value: base moves past 30 deg land short with less
STATE = ROOT / "logs" / "slide_wrist_state.json"


def quiet(f, *a):
    with contextlib.redirect_stdout(io.StringIO()):     # the course API prints every command
        return f(*a)


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == ST_LINK_VID]
    if not ports:
        sys.exit("No Nucleo found: plug its USB cable in, or give --port.")
    return ports[0]


def read_joints(scara, n=30):
    return (statistics.median(quiet(scara.SCARA_READ_ANGLE, 0) for _ in range(n)),
            statistics.median(quiet(scara.SCARA_READ_ANGLE, 1) for _ in range(n)))


def load_state():
    return json.loads(STATE.read_text()) if STATE.exists() else None


def save_state(state):
    STATE.parent.mkdir(exist_ok=True)
    STATE.write_text(json.dumps(state, indent=2))


def refusal(joint, angle, last):
    """Why the wrist or slide mustn't go to angle from last (None: not moved since RESET), or None."""
    if last is None:
        return None if angle == 0 else f"the {joint}'s first move after a RESET must be to 0: it snaps there anyway"
    low, high = SAFE[joint]
    if not low <= angle <= high:
        return f"{angle:g} is outside the {joint}'s safe range {low:g}..{high:g}"
    if joint == "slide" and angle < last:       # up (negative is up): any distance in one move
        return None
    if abs(angle - last) > MAX_STEP[joint]:
        return f"{abs(angle - last):g} deg from {last:g} is more than the {joint}'s {MAX_STEP[joint]:g} deg step"
    return None


def arm_refusal(joint, angle, base, elbow, arm, keep_outs):
    """Why the base or elbow mustn't go to angle from the pose (base, elbow), or None."""
    target = (angle, elbow) if joint == "base" else (base, angle)
    low, high = arm.j0 if joint == "base" else (arm.j1[0], ELBOW_MAX)
    if not low <= angle <= high:
        return f"{angle:g} is outside the {joint}'s range {low:g}..{high:g}"
    hit = first_hit(keep_outs, (base, elbow), target)
    if hit and not leaves(keep_outs, (base, elbow), target):
        return f"crosses the {hit[0]} keep-out at base {hit[1][0]:.1f}, elbow {hit[1][1]:.1f}"
    return None


def leaves(keep_outs, start, target, steps=200):
    """Whether a move starts in a zone and comes out of it for good: SCARA_AUTO_CALIBRATE can end in
    one (base 2, elbow 90 with the tower on), and the way out has to be allowed."""
    path = [(start[0] + t * (target[0] - start[0]), start[1] + t * (target[1] - start[1]))
            for t in (i / steps for i in range(steps + 1))]
    blocked = [any(k.blocks(*p) for k in keep_outs) for p in path]
    return blocked[0] and not blocked[-1] and blocked == sorted(blocked, reverse=True)


def over_plate(base, elbow, kinematics, rig):
    """Whether the gripper, at this pose, could be over the plate."""
    (x, y), = robot_to_scene([kinematics.fk(base, elbow)[:2]], rig)
    (x0, x1), (y0, y1) = PLATE_MM
    return x0 - PLATE_MARGIN_MM < x < x1 + PLATE_MARGIN_MM and y0 - PLATE_MARGIN_MM < y < y1 + PLATE_MARGIN_MM


def plate_refusal(joint, angle, base, elbow, slide, kinematics, rig, steps=100):
    """Why a move mustn't be made with the plate where it is, or None. slide: its last position, None
    if not moved since RESET."""
    if joint == "wrist":
        return None
    if joint == "slide":
        if not over_plate(base, elbow, kinematics, rig):
            return None
        if slide is None:
            return "the slide's first move snaps it down to plate height, so not over the plate"
        return f"over the plate the slide stays above {OVER_PLATE_SLIDE:g}" if angle > OVER_PLATE_SLIDE else None
    if slide is not None and slide <= HOVER_SLIDE:
        return None
    current = base if joint == "base" else elbow
    if slide is not None and slide <= OVER_PLATE_SLIDE and abs(angle - current) <= NUDGE_DEG:
        return None
    target = (angle, elbow) if joint == "base" else (base, angle)
    for i in range(steps + 1):
        t = i / steps
        if over_plate(base + t * (target[0] - base), elbow + t * (target[1] - elbow), kinematics, rig):
            return f"it passes over the plate with the slide below {HOVER_SLIDE:g}"
    return None


def over_bin(base, elbow, kinematics, rig):
    """Whether the wrist, at this pose, is out past the plate's +X edge, over the non-resistor bin."""
    (x, y), = robot_to_scene([kinematics.fk(base, elbow)[:2]], rig)
    return x > BIN_MM[0] and BIN_MM[1][0] < y < BIN_MM[1][1]


def passes_bin(start, end, kinematics, rig, steps=100):
    """Whether a straight joint-space move from start to end (base, elbow) goes over the non-resistor bin."""
    return any(over_bin(start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]), kinematics, rig)
               for t in (i / steps for i in range(steps + 1)))


def bin_refusal(joint, angle, base, elbow, slide, kinematics, rig):
    """Why a move mustn't be made with the non-resistor bin where it is, or None. slide: its last position, None
    if not moved since RESET. Over the bin the slide only goes up; across it, only fully up."""
    if joint == "wrist":
        return None
    if joint == "slide":
        if not over_bin(base, elbow, kinematics, rig):
            return None
        if slide is None:
            return "the slide's first move snaps it down, so not over the non-resistor bin"
        return None if angle <= max(slide, PARK_SLIDE) else f"over the non-resistor bin the slide only goes up, to {PARK_SLIDE:g}"
    if slide is not None and slide <= PARK_SLIDE:
        return None
    target = (angle, elbow) if joint == "base" else (base, angle)
    if passes_bin((base, elbow), target, kinematics, rig):
        return f"it passes over the non-resistor bin with the slide below fully up ({PARK_SLIDE:g})"
    return None


def record(joint, angle, seconds, base, elbow):
    path = ROOT / "logs" / f"slide_wrist_{time.strftime('%Y%m%d')}.csv"
    new = not path.exists()
    with path.open("a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["time", "joint", "angle", "seconds", "base", "elbow"])
        w.writerow([time.strftime("%H:%M:%S"), joint, angle, seconds, f"{base:.2f}", f"{elbow:.2f}"])


def status(scara):
    base, elbow = read_joints(scara)
    state = load_state()
    print(f"base {base:.2f}, elbow {elbow:.2f} deg (potentiometers)")
    print(f"since the last start: {state}" if state else "not started: run start after RESET")


def start(scara):
    arm = ArmSettings.from_config()
    quiet(scara.SCARA_INITIALISE, arm.link1, arm.link2, arm.z[0], arm.z[1], SETTLE_S, 1.3, 0.01, 0.001, 1.3, 0.01, 0.001)
    print("Calibrating: the base and elbow sweep end to end...")
    quiet(scara.SCARA_AUTO_CALIBRATE)
    save_state({"wrist": None, "slide": None, "started": time.strftime("%Y-%m-%d %H:%M")})
    base, elbow = read_joints(scara)
    record("start", "", "", base, elbow)
    print(f"Done: base {base:.2f}, elbow {elbow:.2f} deg")


def move(scara, joint, angle, seconds):
    state = load_state()
    if state is None:
        sys.exit("Not started: run start first.")
    base, elbow = read_joints(scara, QUICK_READS)
    arm = ArmSettings.from_config()
    if joint in SAFE:
        why = refusal(joint, angle, state[joint])
    else:
        why = arm_refusal(joint, angle, base, elbow, arm, [KeepOut(m) for m in KeepOutMap.from_config()])
    kinematics, rig = Kinematics(arm), RigSettings.from_config()
    why = (why or plate_refusal(joint, angle, base, elbow, state["slide"], kinematics, rig)
           or bin_refusal(joint, angle, base, elbow, state["slide"], kinematics, rig))
    if why:
        sys.exit(f"Refused: {why}.")
    quiet(scara.SCARA_MOVE_JOINT, AXES[joint], angle, seconds)
    if joint in SAFE:
        state[joint] = angle
        save_state(state)
    base, elbow = read_joints(scara, QUICK_READS)
    record(joint, angle, seconds, base, elbow)
    print(f"{joint} to {angle:g}: base {base:.2f}, elbow {elbow:.2f} deg")


def land(scara, targets, wrist, slide):
    """Brings the base and elbow onto targets ({"base": deg, "elbow": deg}) after a move: reads the
    potentiometers and sends both joints their target plus part of what they're missing, as the firmware
    stops within its deadband. wrist, slide: where those are, as SCARA_MOVE_JOINTS sends all four.
    Returns the (base, elbow) reached."""
    aim = dict(targets)
    for _ in range(LAND_TRIES):
        got = dict(zip(("base", "elbow"), read_joints(scara)))
        off = {j: targets[j] - got[j] for j in targets}
        if all(abs(e) <= LAND_TOLERANCE_DEG for e in off.values()):
            break
        for joint, error in off.items():
            if abs(error) > LAND_TOLERANCE_DEG:
                aim[joint] = min(max(aim[joint] + LAND_GAIN * error, targets[joint] - LAND_MAX_DEG), targets[joint] + LAND_MAX_DEG)
        quiet(scara.SCARA_MOVE_JOINTS, aim["base"], aim["elbow"], wrist, slide, LAND_S)
    return read_joints(scara)


def swing_refusal(base, elbow, wrist, start, slide, arm, keep_outs, kinematics, rig, clear=HOVER_SLIDE):
    """Why the arm mustn't go to (base, elbow, wrist) from start (base, elbow) in one move, or None. A
    move of several joints is a straight line in joint space. slide: its lowest position on the way;
    clear: the lowest it may be over the plate."""
    if not arm.j0[0] <= base <= arm.j0[1]:
        return f"base {base:g} is outside {arm.j0[0]:g}..{arm.j0[1]:g}"
    if not arm.j1[0] <= elbow <= ELBOW_MAX:
        return f"elbow {elbow:g} is outside {arm.j1[0]:g}..{ELBOW_MAX:g}"
    if not SAFE["wrist"][0] <= wrist <= SAFE["wrist"][1]:
        return f"wrist {wrist:g} is outside {SAFE['wrist'][0]:g}..{SAFE['wrist'][1]:g}"
    hit = first_hit(keep_outs, start, (base, elbow))
    if hit and not leaves(keep_outs, start, (base, elbow)):
        return f"crosses the {hit[0]} keep-out at base {hit[1][0]:.1f}, elbow {hit[1][1]:.1f}"
    if slide > clear:
        for i in range(101):
            t = i / 100
            if over_plate(start[0] + t * (base - start[0]), start[1] + t * (elbow - start[1]), kinematics, rig):
                return f"it passes over the plate with the slide below {clear:g}"
    if slide > PARK_SLIDE and passes_bin(start, (base, elbow), kinematics, rig):
        return f"it passes over the non-resistor bin with the slide below fully up ({PARK_SLIDE:g})"
    return None


def approach(scara, base, elbow, wrist, seconds):
    """To (base, elbow, wrist) with the wrist arriving from WRIST_START, so its play is taken up the same
    way every time: the same command then gives the same angle."""
    swing(scara, base, elbow, WRIST_START, seconds)
    move(scara, "wrist", wrist, WRIST_TURN_S)


def glide(scara, base, elbow, wrist, slide, seconds):
    """All four joints in one smooth move. The slide must stay at GLIDE_SLIDE or higher all the way: the
    tip is then clear of the plate and anything lying on it, so it needn't lift first."""
    state = load_state()
    if state is None or state["wrist"] is None or state["slide"] is None:
        sys.exit("Refused: the wrist and slide need their first moves before a glide.")
    arm = ArmSettings.from_config()
    start = read_joints(scara, QUICK_READS)
    why = swing_refusal(base, elbow, wrist, start, max(slide, state["slide"]), arm,
                        [KeepOut(m) for m in KeepOutMap.from_config()], Kinematics(arm), RigSettings.from_config(),
                        clear=GLIDE_SLIDE)
    if why:
        sys.exit(f"Refused: {why}.")
    quiet(scara.SCARA_MOVE_JOINTS, base, elbow, wrist, slide, seconds)
    state.update(wrist=wrist, slide=slide)
    save_state(state)
    got = read_joints(scara, QUICK_READS)
    record("glide", f"{base:.2f} {elbow:.2f} {wrist:.2f} {slide:.2f}", seconds, *got)
    print(f"glide to base {base:.2f}, elbow {elbow:.2f}, wrist {wrist:.1f}, slide {slide:g}: "
          f"base {got[0]:.2f}, elbow {got[1]:.2f} deg")


def swing(scara, base, elbow, wrist, seconds):
    """Base, elbow and wrist in one move, the slide staying where it is. After a RESET the wrist and
    slide must have had their first moves, or SCARA_MOVE_JOINTS would snap them."""
    state = load_state()
    if state is None or state["wrist"] is None or state["slide"] is None:
        sys.exit("Refused: the wrist and slide need their first moves before a swing.")
    arm = ArmSettings.from_config()
    start = read_joints(scara, QUICK_READS)
    why = swing_refusal(base, elbow, wrist, start, state["slide"], arm, [KeepOut(m) for m in KeepOutMap.from_config()],
                        Kinematics(arm), RigSettings.from_config())
    if why:
        sys.exit(f"Refused: {why}.")
    quiet(scara.SCARA_MOVE_JOINTS, base, elbow, wrist, state["slide"], seconds)
    state["wrist"] = wrist
    save_state(state)
    got = read_joints(scara, QUICK_READS)
    record("swing", f"{base:.2f} {elbow:.2f} {wrist:.2f}", seconds, *got)
    print(f"swing to base {base:.2f}, elbow {elbow:.2f}, wrist {wrist:.1f}: base {got[0]:.2f}, elbow {got[1]:.2f} deg")


def slide_steps(last, target, step=MAX_STEP["slide"], up_in_one=True):
    """Angles from last to target, at most step apart going down. None: not moved since RESET, so 0 first.
    up_in_one: going up (to a lower angle, the slide's up) is one move, however far. For the slide; the
    wrist's angles have no up, so its callers pass False."""
    steps = [0.0] if last is None else []
    now = 0.0 if last is None else last
    if up_in_one and target < now:
        return steps + [target]
    while now != target:
        now = max(now - step, target) if target < now else min(now + step, target)
        steps.append(now)
    return steps


def park(scara):
    """To PARK from wherever the arm is, in the order the plate, the tower and the non-resistor bin allow: the
    slide fully up before anything else moves (after a start, the elbow first, out of the tower's
    keep-out to where the slide's first move may be made)."""
    state = load_state()
    if state is None:
        sys.exit("Not started: run start first.")
    if state["slide"] is None and abs(read_joints(scara)[1] - PARK["elbow"]) > PARK_TOLERANCE_DEG:
        move(scara, "elbow", PARK["elbow"], MOVE_S)
    for angle in slide_steps(state["slide"], PARK_SLIDE):
        move(scara, "slide", angle, 1.0 if state["slide"] is None and angle == 0 else SLIDE_STEP_S)
    if abs(read_joints(scara)[1] - PARK["elbow"]) > PARK_TOLERANCE_DEG:
        move(scara, "elbow", PARK["elbow"], MOVE_S)
    if abs(read_joints(scara)[0] - PARK["base"]) > PARK_TOLERANCE_DEG:
        move(scara, "base", PARK["base"], SWING_S)
    base, elbow = read_joints(scara)
    print(f"Parked: base {base:.2f}, elbow {elbow:.2f} deg, slide {load_state()['slide']:g}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    commands = p.add_subparsers(dest="command", required=True)
    commands.add_parser("status")
    commands.add_parser("start")
    commands.add_parser("park")
    m = commands.add_parser("move")
    m.add_argument("joint", choices=list(AXES))
    m.add_argument("angle", type=float)
    m.add_argument("--time", type=float, default=MOVE_S, help=f"seconds ({MOVE_S:g})")
    args = p.parse_args()

    scara = smc_api.scara_motion_controller(port=args.port or find_port(), timeout=120)
    if args.command == "status":
        status(scara)
    elif args.command == "start":
        start(scara)
    elif args.command == "park":
        park(scara)
    else:
        move(scara, args.joint, args.angle, args.time)


if __name__ == "__main__":
    main()
