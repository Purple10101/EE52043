"""Teach the pick calibration one command at a time: the arm goes where it thinks a resistor is, a person
says how far off the jaws are, the arm is nudged until they are centred, and the pair is recorded.

The same pairs and fit as scripts/teach_picks.py (its keyboard window), but each step is a separate command,
so it can be driven from a terminal or by an assistant relaying what the person at the rig sees:

    python scripts/teach_step.py look                 # park, find the resistors, number them (a new session)
    python scripts/teach_step.py go 3                 # the jaws down over resistor 3, as a pick aims
    python scripts/teach_step.py nudge --along 1.5    # move the jaws: --along/--across the body (mm),
    python scripts/teach_step.py nudge --base -2      #   --base toward (+) or away from (-) the arm's base,
    python scripts/teach_step.py nudge --dx 1 --dy 0  #   or --dx/--dy in scene mm; they add up
    python scripts/teach_step.py record               # centred: record the pair (then go to the next)
    python scripts/teach_step.py status               # the session so far
    python scripts/teach_step.py fit [--dry]          # fit (vision/pick_fit.py) and save pick_calibration.json
    python scripts/teach_step.py park
    python scripts/teach_step.py read                 # the base and elbow potentiometers, nothing sent to the arm
    python scripts/teach_step.py hand 3               # the arm put over resistor 3 BY HAND (servo supply off):
                                                      #   record the pair from the pots there

By hand (the user's idea, 2026-11-12): servo supply off, the closed tip moved by hand onto the body's centre, the
wrist left as it is (it has no potentiometer: its angle is the last one sent), then `hand N`. No landing error,
stiction or slide drag. Before the supply goes back on: the arm back at the park, slide fully up (the servos jump
to the last command, the park).

Directions: "along" is the body's long axis as the camera found it (its angle is printed), "across" 90 deg
anticlockwise from it seen from above; --base is straight toward the base's axis. Every nudge lifts the jaws to
the approach height, moves, and comes back down to the pick height, so nothing on the plate is touched.

The session (the resistors, which one the arm is at, how far it has been nudged) is kept in
logs/pick_teach_session.json; the pairs in logs/pick_teach_<time>.csv (teach_picks.py's columns). Works without
the gripper board: the jaws stay as they are.
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav  # noqa: E402
import slide_wrist as bench  # noqa: E402
import sort_resistors  # noqa: E402
import teach_picks  # noqa: E402

from robot_control.vision.guide import plan_jaws  # noqa: E402

SESSION = ROOT / "logs" / "pick_teach_session.json"
# The jaws closed, the tip stopped here (going down) over the resistor: about 7 mm above the plate (52 was a little low, the user), clear of a body
# 2.4 mm tall. A closed tip is one point, easier to judge than open jaws; at the pick height (63) it would hit the body.
TEACH_SLIDE = 48.0


def close_jaws(n):
    if n.gripper is not None:
        n.gripper.close()


def say(text):
    print(text, flush=True)


def load_session():
    if not SESSION.exists():
        sys.exit("No session: run `teach_step.py look` first.")
    return json.loads(SESSION.read_text())


def save_session(s):
    SESSION.write_text(json.dumps(s, indent=2) + "\n")


def build():
    if bench.load_state() is None:
        sys.exit("Not started: CALIBRATE in the sorting window first (then close it).")
    arm, _ = sort_resistors.build_arm(None, resistor_nav.load_bins(), camera_calibration=False, need_gripper=False)
    return arm


def aim_of(s):
    r = s["resistors"][s["at"]]
    return np.asarray(r["centre"], float) + np.asarray(s["offset"], float), r["direction"]


def plan(arm, aim, direction):
    n = arm.navigator
    return plan_jaws(aim, direction, bench.SAFE["wrist"], n.kinematics, n.rig, n.wrist_centre, n.correction,
                     at_axis=n.aim_axis, jaw_offset=n.jaw_offset)


def where(s):
    r = s["resistors"][s["at"]]
    a = np.radians(r["direction"])
    u, v = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    c = np.asarray(r["centre"], float)
    towards_base = -c / np.linalg.norm(c)
    off = np.asarray(s["offset"], float)
    return (f"resistor {s['at'] + 1}: camera centre ({c[0]:.1f}, {c[1]:.1f}) mm, long axis {r['direction']:.0f} deg; "
            f"jaws nudged ({off[0]:+.2f}, {off[1]:+.2f}) mm = {off @ u:+.2f} along, {off @ v:+.2f} across, "
            f"{off @ towards_base:+.2f} toward the base")


PLATE_PICTURE = ROOT / "logs" / "pick_teach_plate.png"


def numbered_plate(view, resistors, path=PLATE_PICTURE):
    """The plate from above with each resistor's session number on it (the numbers `go`/`hand` take)."""
    import cv2
    from robot_offline.pre import rectify
    rect, _ = rectify.white_balance(rectify.rectify(view.frame, view.cal), view.cal.settings)
    for i, r in enumerate(resistors, 1):
        (x, y), = rectify.mm_to_rect(np.asarray([r.centre_mm], float), view.cal.settings)
        x, y = int(x), int(y)
        cv2.circle(rect, (x, y), 10, (0, 0, 255), -1, cv2.LINE_AA)
        for colour, thick in (((255, 255, 255), 14), ((0, 0, 255), 5)):
            cv2.putText(rect, str(i), (x + 22, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 2.4, colour, thick, cv2.LINE_AA)
    cv2.imwrite(str(path), rect)
    return path


def cmd_look(args):
    from robot_control.vision.eyes import Eyes
    if args.no_park:
        say("not parking (--no-park): the arm must be out of the camera's view already")
    else:
        build().park()
    eyes = Eyes(closeups=False, say=say)
    try:
        view = eyes.look(sharp=True)
    finally:
        eyes.close()
    if args.keep and SESSION.exists():
        pairs = Path(load_session()["pairs"])          # the resistors moved: new positions, the same pairs file
    else:
        pairs = ROOT / "logs" / f"pick_teach_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        with pairs.open("w", newline="") as f:
            csv.writer(f).writerow(teach_picks.FIELDS)
    rs = sorted(view.resistors, key=lambda r: (round(r.centre_mm[1] / 15), r.centre_mm[0]))
    picture = numbered_plate(view, rs)
    save_session({"pairs": str(pairs), "at": None, "offset": [0.0, 0.0], "recorded": [],
                  "resistors": [{"centre": [float(v) for v in r.centre_mm], "direction": float(r.direction_deg)} for r in rs]})
    say(f"{len(rs)} resistors (pairs to {pairs.relative_to(ROOT)}; numbered picture {picture.relative_to(ROOT)}):")
    for i, r in enumerate(rs, 1):
        say(f"  {i}: ({r.centre_mm[0]:.1f}, {r.centre_mm[1]:.1f}) mm, long axis {r.direction_deg:.0f} deg")


def cmd_go(args):
    s = load_session()
    if not 1 <= args.n <= len(s["resistors"]):
        sys.exit(f"resistor {args.n}? there are {len(s['resistors'])}")
    arm = build()
    s["at"], s["offset"] = args.n - 1, [0.0, 0.0]
    aim, direction = aim_of(s)
    n = arm.navigator
    if n.go(aim, direction) is None:
        sys.exit(f"not reached: {n.status}")
    base, elbow, wrist = plan(arm, aim, direction)
    bench.land(n.scara, {"base": base, "elbow": elbow}, wrist, resistor_nav.NAV_SLIDE)    # long glides land short
    close_jaws(n)
    n.drive("slide", TEACH_SLIDE, resistor_nav.SLIDE_MOVE_S)
    got = bench.read_joints(n.scara, bench.QUICK_READS)
    save_session(s)
    say(f"DOWN, jaws closed, tip about 7 mm up; landed base {got[0]:.2f} ({got[0] - base:+.2f}), elbow {got[1]:.2f} "
        f"({got[1] - elbow:+.2f}). {where(s)}")


def cmd_nudge(args):
    s = load_session()
    if s["at"] is None:
        sys.exit("not at a resistor: `go N` first")
    r = s["resistors"][s["at"]]
    a = np.radians(r["direction"])
    u, v = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    c = np.asarray(r["centre"], float)
    move = args.along * u + args.across * v + args.base * (-c / np.linalg.norm(c)) + np.array([args.dx, args.dy])
    s["offset"] = list(np.asarray(s["offset"], float) + move)
    arm = build()
    n = arm.navigator
    aim, direction = aim_of(s)
    base, elbow, wrist = plan(arm, aim, direction)
    n.drive("slide", resistor_nav.NAV_SLIDE, resistor_nav.SLIDE_MOVE_S)
    bench.glide(n.scara, base, elbow, wrist, resistor_nav.NAV_SLIDE, teach_picks.NUDGE_S)
    got = bench.land(n.scara, {"base": base, "elbow": elbow}, wrist, resistor_nav.NAV_SLIDE)   # small moves land short or long
    close_jaws(n)
    n.drive("slide", TEACH_SLIDE, resistor_nav.SLIDE_MOVE_S)
    got = bench.read_joints(n.scara, bench.QUICK_READS)
    save_session(s)
    say(f"moved ({move[0]:+.2f}, {move[1]:+.2f}) mm; landed base {got[0]:.2f} ({got[0] - base:+.2f}), elbow {got[1]:.2f} "
        f"({got[1] - elbow:+.2f}). DOWN again. {where(s)}")


def cmd_record(args):
    s = load_session()
    if s["at"] is None:
        sys.exit("not at a resistor: `go N` first")
    arm = build()
    base, elbow = (float(v) for v in bench.read_joints(arm.navigator.scara))
    wrist = float(bench.load_state()["wrist"])
    r = s["resistors"][s["at"]]
    aim, _ = aim_of(s)
    a = np.radians(r["direction"])
    off = np.asarray(s["offset"], float)
    along, across = off @ np.array([np.cos(a), np.sin(a)]), off @ np.array([-np.sin(a), np.cos(a)])
    row = [r["centre"][0], r["centre"][1], r["direction"], aim[0], aim[1], along, across, base, elbow, wrist]
    with open(s["pairs"], "a", newline="") as f:
        csv.writer(f).writerow([round(float(x), 3) for x in row])
    s["recorded"].append(s["at"] + 1)
    save_session(s)
    say(f"RECORDED resistor {s['at'] + 1}: base {base:.2f}, elbow {elbow:.2f}, wrist {wrist:.1f}. {where(s)}")
    arm.navigator.lift()


def cmd_status(args):
    s = load_session()
    say(f"pairs: {s['pairs']}; recorded: {s['recorded'] or 'none'}")
    for i, r in enumerate(s["resistors"], 1):
        say(f"  {i}: ({r['centre'][0]:.1f}, {r['centre'][1]:.1f}) mm, long axis {r['direction']:.0f} deg"
            + ("  [recorded]" if i in s["recorded"] else ""))
    if s["at"] is not None:
        say(where(s))


def cmd_fit(args):
    s = load_session()
    teach_picks.fit_and_save(teach_picks.read_pairs(s["pairs"]), Path(s["pairs"]).name, args.dry)


def read_pots():
    """(base, elbow) from the potentiometers, the Nucleo only: no gripper board, nothing moved."""
    scara = bench.smc_api.scara_motion_controller(port=bench.find_port(), timeout=120)
    return tuple(float(v) for v in bench.read_joints(scara))


def cmd_read(args):
    base, elbow = read_pots()
    say(f"pots: base {base:.2f}, elbow {elbow:.2f} deg")


def cmd_hand(args):
    s = load_session()
    if not 1 <= args.n <= len(s["resistors"]):
        sys.exit(f"resistor {args.n}? there are {len(s['resistors'])}")
    base, elbow = read_pots()
    wrist = float(bench.load_state()["wrist"])
    r = s["resistors"][args.n - 1]
    row = [r["centre"][0], r["centre"][1], r["direction"], r["centre"][0], r["centre"][1], 0.0, 0.0, base, elbow, wrist]
    with open(s["pairs"], "a", newline="") as f:
        csv.writer(f).writerow([round(float(x), 3) for x in row])
    s.setdefault("by_hand", []).append(len(open(s["pairs"]).read().strip().splitlines()) - 1)
    s["recorded"].append(args.n)
    save_session(s)
    say(f"RECORDED BY HAND resistor {args.n} at ({r['centre'][0]:.1f}, {r['centre'][1]:.1f}): base {base:.2f}, "
        f"elbow {elbow:.2f}, wrist {wrist:.1f} (the last sent)")


def cmd_park(args):
    arm = build()
    arm.navigator.lift()
    arm.park()
    if SESSION.exists():
        s = load_session()
        s["at"] = None
        save_session(s)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    lo = sub.add_parser("look")
    lo.add_argument("--keep", action="store_true", help="add to the session's pairs file (the resistors were moved)")
    lo.add_argument("--no-park", action="store_true", help="don't move the arm (servo supply off; already out of view)")
    lo.set_defaults(go=cmd_look)
    g = sub.add_parser("go")
    g.add_argument("n", type=int)
    g.set_defaults(go=cmd_go)
    nu = sub.add_parser("nudge")
    for name in ("along", "across", "base", "dx", "dy"):
        nu.add_argument(f"--{name}", type=float, default=0.0)
    nu.set_defaults(go=cmd_nudge)
    sub.add_parser("record").set_defaults(go=cmd_record)
    sub.add_parser("status").set_defaults(go=cmd_status)
    f = sub.add_parser("fit")
    f.add_argument("--dry", action="store_true")
    f.set_defaults(go=cmd_fit)
    sub.add_parser("park").set_defaults(go=cmd_park)
    sub.add_parser("read").set_defaults(go=cmd_read)
    h = sub.add_parser("hand")
    h.add_argument("n", type=int)
    h.set_defaults(go=cmd_hand)
    args = p.parse_args()
    args.go(args)


if __name__ == "__main__":
    main()
