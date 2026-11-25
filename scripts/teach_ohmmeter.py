"""Teach the arm where the ohmmeter's station is: where to put a held resistor's leads on its two pads.

One command at a time, so it can be driven from a terminal or by an assistant relaying what the person at the
rig sees. The x, y is taught by hand, the same way as the bins; the wrist and the slide with the arm powered:

    python scripts/teach_ohmmeter.py meter [--count 20]   # the meter's readings now (nothing moves)
    python scripts/teach_ohmmeter.py pick                 # park, look, pick the one resistor on the plate, park
                                                          #   holding it (the jaws hold after the script ends)
    python scripts/teach_ohmmeter.py hand                 # SERVO SUPPLY OFF, the jaws over the pads by hand:
                                                          #   the pots give the station's x, y
    python scripts/teach_ohmmeter.py go                   # powered: over the station at the travel height
    python scripts/teach_ohmmeter.py turn 5               # the wrist 5 deg more (the held resistor turns with it)
    python scripts/teach_ohmmeter.py nudge --dx 1 --dy 0  # the station point moved in scene mm (or --along/--across
                                                          #   the held resistor's axis)
    python scripts/teach_ohmmeter.py release / grip       # open the jaws to put a resistor in by hand / close them
    python scripts/teach_ohmmeter.py probe [--from 30]    # the slide down a step at a time, the meter read at each,
                                                          #   until it reads steadily: the contact height
    python scripts/teach_ohmmeter.py slide 45 / up        # the slide to 45 / back to the travel height
    python scripts/teach_ohmmeter.py save [--press 1.5]   # write config/ohmmeter_station.toml (press: slide degrees
                                                          #   past the contact, to measure at)
    python scripts/teach_ohmmeter.py try                  # the station as the run uses it: there, down, measure, up
    python scripts/teach_ohmmeter.py status / park

By hand (as the bins, 2026-11-12): first park (slide fully up) and set the wrist (`turn` works from the park
too), then the servo supply off. Move the base and elbow by hand until the jaws' centre is over the middle
of the gap between the pads, without turning the wrist (it has no potentiometer: its angle is the last one
sent), then `hand`. With a resistor in the jaws and its leads on the pads the meter's reading is shown too.
BEFORE THE SUPPLY GOES BACK ON: the arm back at the park by hand, slide fully up: the servos jump to their
last command, the park.

Powered, then: `go` shows where the taught point and wrist put the resistor, at the travel height; `turn` and
`nudge` until the leads lie across both pads, then `probe`. The held resistor's direction is kept as a scene
angle (lead_axis_deg), so the run lines it up the same way whatever pose the arm reaches the station in.
The session is kept in logs/ohmmeter_teach_session.json.
"""

import argparse
import json
import math
import shutil
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav  # noqa: E402
import slide_wrist as bench  # noqa: E402

from robot_control.config import CONFIG_DIR, ArmSettings, OhmmeterSettings, OhmmeterStation, RigSettings  # noqa: E402
from robot_control.hardware.ohmmeter import OhmmeterError, SerialOhmmeter  # noqa: E402
from robot_control.kinematics.arm import Kinematics  # noqa: E402
from robot_control.vision.guide import jaw_angle  # noqa: E402
from robot_control.vision.rig import robot_to_scene  # noqa: E402

SESSION = ROOT / "logs" / "ohmmeter_teach_session.json"
STATION = CONFIG_DIR / "ohmmeter_station.toml"
PROBE_STEP = 1.0            # slide degrees per probe step (about 0.35 mm)
PROBE_MOVE_S = 0.4
PROBE_STEADY = 2            # steps in a row with a steady reading: in contact
PRESS = 1.5                 # slide degrees past the contact to measure at, by default


def say(text):
    print(text, flush=True)


def lead_axis(base, elbow, wrist):
    """The held resistor's long axis, scene deg in -90..90: square to the line the jaws close along."""
    return (jaw_angle(base, elbow, wrist) - 90.0 + 90.0) % 180.0 - 90.0


def load_session():
    return json.loads(SESSION.read_text()) if SESSION.exists() else {}


def save_session(s):
    SESSION.parent.mkdir(exist_ok=True)
    SESSION.write_text(json.dumps(s, indent=2) + "\n")


def need(s, *keys):
    missing = [k for k in keys if s.get(k) is None]
    if missing:
        sys.exit(f"Not taught yet: {', '.join(missing)} (run `hand` first, then `probe` for the slide).")


def meter():
    settings = OhmmeterSettings.from_config()
    try:
        return SerialOhmmeter(settings)
    except OhmmeterError as why:
        sys.exit(f"The ohmmeter isn't ready: {why}")


def describe(m):
    if m.ohms is None:
        return "no reading"
    if m.is_open:
        return "open circuit"
    if m.ohms == 0.0:
        return "short circuit"
    value = f"{m.ohms:.1f} ohm"
    return f"{value} -> {m.nominal_ohm:g} ohm" if m.nominal_ohm is not None else f"{value} ({m.why})"


def navigator(need_gripper=False):
    if bench.load_state() is None:
        sys.exit("Not started: start the arm first.")
    import sort_resistors
    arm, _ = sort_resistors.build_arm(None, resistor_nav.load_bins(), camera_calibration=False,
                                      need_gripper=need_gripper)
    return arm.navigator


def at_station(n, s):
    """Whether the base and elbow are at the station's pose now (within the bins' tolerance)."""
    base, elbow, _ = n.station_pose(s["centre_mm"], s["lead_axis_deg"])
    bench.allow_deep_slide(base, elbow)
    got = bench.read_joints(n.scara, bench.QUICK_READS)
    return max(abs(got[0] - base), abs(got[1] - elbow)) <= resistor_nav.BIN_TOLERANCE_DEG


def cmd_meter(args):
    m = meter()
    try:
        for i in range(args.count):
            say(f"{i + 1}: {describe(m.measure(settle=False))}")
            time.sleep(args.every)
    finally:
        m.close()


def cmd_hand(args):
    scara = bench.smc_api.scara_motion_controller(port=bench.find_port(), timeout=120)
    base, elbow = (float(v) for v in bench.read_joints(scara, 50))
    state = bench.load_state() or {}
    wrist = state.get("wrist")
    kinematics, rig = Kinematics(ArmSettings.from_config()), RigSettings.from_config()
    (x, y), = robot_to_scene([kinematics.fk(base, elbow)[:2]], rig)
    s = load_session()
    s.update(centre_mm=[round(float(x), 2), round(float(y), 2)], hand_pots=[round(base, 2), round(elbow, 2)],
             hand_wrist=wrist, taught=time.strftime("%Y-%m-%d %H:%M"))
    if wrist is not None:
        s["lead_axis_deg"] = round(lead_axis(base, elbow, wrist), 1)
    s.pop("contact_slide", None)            # a new point: the height is probed again
    save_session(s)
    say(f"TAUGHT BY HAND: pots base {base:.2f}, elbow {elbow:.2f} deg -> scene ({x:.1f}, {y:.1f}) mm")
    if wrist is not None:
        say(f"  wrist {wrist:.1f} (the last sent): the held resistor along {s['lead_axis_deg']:.1f} deg")
    else:
        say("  the wrist hasn't been moved since the start: its direction is taught powered (`go`, then `turn`)")
    if not args.no_meter:
        m = meter()
        try:
            say(f"  meter: {describe(m.measure(settle=False))}")
        finally:
            m.close()
    say("BEFORE THE SERVO SUPPLY GOES BACK ON: the arm back at the park by hand, slide fully up.")


def cmd_go(args):
    s = load_session()
    need(s, "centre_mm")
    n = navigator()
    if s.get("lead_axis_deg") is None:      # no wrist yet: keep the jaws as they are now, where it lands
        base, elbow = n.station_pose(s["centre_mm"], 0.0)[:2]
        s["lead_axis_deg"] = round(lead_axis(base, elbow, bench.load_state()["wrist"] or 0.0), 1)
        save_session(s)
    if n.to_station(s["centre_mm"], s["lead_axis_deg"]):
        say(f"over the station at the travel height, the resistor along {s['lead_axis_deg']:.1f} deg: "
            "`turn` / `nudge` until the leads lie across both pads, then `probe`")


def cmd_turn(args):
    s = load_session()
    n = navigator()
    state = bench.load_state()
    if s.get("centre_mm") is not None and at_station(n, s):
        if state["slide"] > bench.HOVER_SLIDE:
            n.lift()                        # not dragging the leads across the pads
        s["lead_axis_deg"] = round((s["lead_axis_deg"] + args.deg + 90.0) % 180.0 - 90.0, 1)
        base, elbow, wrist = n.station_pose(s["centre_mm"], s["lead_axis_deg"])
        bench.move(n.scara, "wrist", round(wrist, 1), bench.WRIST_TURN_S)
        save_session(s)
        say(f"the held resistor along {s['lead_axis_deg']:.1f} deg (wrist {wrist:.1f})")
        return
    wrist = (state["wrist"] or 0.0) + args.deg  # away from the station (the park, before `hand`): the wrist as it is
    bench.move(n.scara, "wrist", round(wrist, 1), bench.WRIST_TURN_S)
    say(f"wrist {wrist:.1f}")


def cmd_nudge(args):
    s = load_session()
    need(s, "centre_mm", "lead_axis_deg")
    a = math.radians(s["lead_axis_deg"])
    along, across = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    step = np.array([args.dx, args.dy]) + args.along * along + args.across * across
    s["centre_mm"] = [round(float(v), 2) for v in np.asarray(s["centre_mm"], float) + step]
    s.pop("contact_slide", None)
    n = navigator()
    if bench.load_state()["slide"] > bench.HOVER_SLIDE:
        n.lift()
    save_session(s)
    if n.to_station(s["centre_mm"], s["lead_axis_deg"]):
        say(f"station point now ({s['centre_mm'][0]:.1f}, {s['centre_mm'][1]:.1f}) mm")


def cmd_jaws(args):
    n = navigator(need_gripper=True)
    (n.gripper.close if args.command == "grip" else n.gripper.open)()
    say("jaws closed" if args.command == "grip" else "jaws open: put the resistor in, then `grip`")


def cmd_slide(args):
    s = load_session()
    n = navigator()
    if args.command == "up":
        n.lift()
        say(f"slide up at {bench.load_state()['slide']:g}")
        return
    need(s, "centre_mm", "lead_axis_deg")
    if not at_station(n, s):
        sys.exit("Not at the station: `go` first.")
    n.drive("slide", args.angle, resistor_nav.SLIDE_MOVE_S)
    m = meter()
    try:
        say(f"slide {args.angle:g}: {describe(m.measure())}")
    finally:
        m.close()


def cmd_probe(args):
    s = load_session()
    need(s, "centre_mm", "lead_axis_deg")
    n = navigator()
    if not at_station(n, s) and not n.to_station(s["centre_mm"], s["lead_axis_deg"]):
        sys.exit("Couldn't get to the station.")
    m = meter()
    start = max(args.start, bench.load_state()["slide"])
    if bench.load_state()["slide"] < start:
        n.drive("slide", start, resistor_nav.SLIDE_MOVE_S)
    steady, slide, first = 0, start, None
    try:
        while True:
            r = m.measure()
            say(f"slide {slide:5.1f}: {describe(r)}")
            if r.ohms is not None and not r.is_open and not r.why.startswith("unsteady"):   # a short is contact too
                first = slide if steady == 0 else first
                steady += 1
                if steady >= PROBE_STEADY:
                    break
            else:
                steady, first = 0, None
            if slide + args.step > args.to:
                say(f"down to {args.to:g} with no steady reading: the pads are lower than the slide reaches, "
                    "or the leads aren't over them (`up`, then `nudge` / `turn`)")
                return
            slide = round(slide + args.step, 2)
            bench.move(n.scara, "slide", slide, PROBE_MOVE_S)
    finally:
        m.close()
    s["contact_slide"] = first
    save_session(s)
    say(f"CONTACT at slide {first:g} (steady to {slide:g}). `save --press {PRESS:g}` measures at {first + PRESS:g}; "
        "`up` before any turn or nudge")


def cmd_save(args):
    s = load_session()
    need(s, "centre_mm", "lead_axis_deg", "contact_slide")
    measure = round(s["contact_slide"] + args.press, 2)
    if STATION.exists():
        keep = ROOT / "logs" / f"ohmmeter_station_replaced_{time.strftime('%Y%m%d_%H%M%S')}.toml"
        shutil.copy(STATION, keep)
        say(f"the old station kept in {keep.relative_to(ROOT)}")
    pots = s.get("hand_pots") or ["?", "?"]
    STATION.write_text(
        "# The ohmmeter's station: where the arm puts a held resistor's leads on the meter's two pads.\n"
        f"# Taught {s.get('taught', '?')} by scripts/teach_ohmmeter.py (session logs/ohmmeter_teach_session.json).\n"
        "# centre_mm: the wrist's axis over the pads, scene mm, taught by hand as the bins are (servo supply off,\n"
        f"# the pots read: base {pots[0]}, elbow {pots[1]}) and turned into scene mm by the CAD rig, as\n"
        "# resistor_nav.to_station turns it back (nudged since, if a nudge was made).\n"
        "# lead_axis_deg: the held resistor's long axis, scene deg from +x toward +y, set with the arm powered.\n"
        "# contact_slide: the slide where the meter first read steadily, probing down a degree at a time.\n"
        "# measure_slide: where the run measures: pressed on a little further.\n\n"
        f"centre_mm = [{s['centre_mm'][0]}, {s['centre_mm'][1]}]\n"
        f"lead_axis_deg = {s['lead_axis_deg']}\n"
        f"contact_slide = {s['contact_slide']}\n"
        f"measure_slide = {measure}\n", encoding="utf-8")
    OhmmeterStation.from_config()           # it reads back
    say(f"saved {STATION.relative_to(ROOT)}: ({s['centre_mm'][0]}, {s['centre_mm'][1]}) mm, along "
        f"{s['lead_axis_deg']} deg, measuring at slide {measure:g} (contact {s['contact_slide']:g})")


def cmd_try(args):
    station = OhmmeterStation.from_config()
    if station is None:
        sys.exit("No station saved yet: `save` first.")
    n = navigator()
    if not n.to_station(station.centre_mm, station.lead_axis_deg, station.measure_slide):
        sys.exit("Couldn't get to the station.")
    m = meter()
    try:
        say(f"measured at slide {station.measure_slide:g}: {describe(m.measure())}")
    finally:
        m.close()
        n.lift()


def cmd_pick(args):
    """The one resistor on the plate into the jaws, as the run picks (the camera calibration's aim), then parked
    holding it. The gripper board keeps holding after the script ends."""
    if bench.load_state() is None:
        sys.exit("Not started: start the arm first.")
    import sort_resistors
    from robot_control.planning.planner import Plan
    from robot_control.task.run_log import RunLog
    from robot_control.vision.eyes import Eyes
    arm, _ = sort_resistors.build_arm(None, resistor_nav.load_bins())
    eyes = Eyes(closeups=False, say=say)
    log = RunLog()          # each look's frame, rectify fit and readings, as the sorting runs keep them
    say(f"looks saved to {log.folder.relative_to(ROOT)}")
    try:
        say("parking to look")
        arm.navigator.park()
        view = eyes.look(sharp=True)
        log.look(0, view, Plan(None, {o.number: "before the pick" for o in view.resistors}, 0))
        if len(view.resistors) != 1:
            sys.exit(f"expected one resistor on the plate, found {len(view.resistors)}: nothing picked")
        r = view.resistors[0]
        say(f"picking the resistor at ({r.centre_mm[0]:.1f}, {r.centre_mm[1]:.1f}) mm, long axis {r.direction_deg:.0f} deg")
        if not arm.pick(r):
            sys.exit(f"NOT picked: {arm.message}")
        say("jaws closed: parking holding it")
        arm.navigator.park()
        after = eyes.look(sharp=True)
        log.look(0, after, Plan(None, {o.number: "after the pick" for o in after.resistors}, 0))
        left = [o for o in after.resistors if o.distance_to(r.centre_mm) < 6.0]
        say("STILL ON THE PLATE: the pick missed (`release`, then `pick` again)" if left
            else "HOLDING IT: gone from the plate")
    finally:
        eyes.close()


def cmd_status(args):
    s = load_session()
    say(json.dumps(s, indent=2) if s else "no session yet: `hand` first")
    station = OhmmeterStation.from_config()
    say(f"saved station: {station}" if station else "no saved station (config/ohmmeter_station.toml)")


def cmd_park(args):
    n = navigator()
    n.lift()
    n.park()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    me = sub.add_parser("meter")
    me.add_argument("--count", type=int, default=5)
    me.add_argument("--every", type=float, default=0.5, help="seconds between readings")
    me.set_defaults(go=cmd_meter)
    sub.add_parser("pick").set_defaults(go=cmd_pick)
    h = sub.add_parser("hand")
    h.add_argument("--no-meter", action="store_true", help="don't read the meter")
    h.set_defaults(go=cmd_hand)
    sub.add_parser("go").set_defaults(go=cmd_go)
    t = sub.add_parser("turn")
    t.add_argument("deg", type=float)
    t.set_defaults(go=cmd_turn)
    nu = sub.add_parser("nudge")
    for name in ("dx", "dy", "along", "across"):
        nu.add_argument(f"--{name}", type=float, default=0.0)
    nu.set_defaults(go=cmd_nudge)
    sub.add_parser("grip").set_defaults(go=cmd_jaws)
    sub.add_parser("release").set_defaults(go=cmd_jaws)
    pr = sub.add_parser("probe")
    pr.add_argument("--from", dest="start", type=float, default=bench.HOVER_SLIDE, help="slide to start at")
    pr.add_argument("--to", type=float, default=bench.STATION_SLIDE_MAX, help="the lowest it goes")
    pr.add_argument("--step", type=float, default=PROBE_STEP)
    pr.set_defaults(go=cmd_probe)
    sl = sub.add_parser("slide")
    sl.add_argument("angle", type=float)
    sl.set_defaults(go=cmd_slide)
    sub.add_parser("up").set_defaults(go=cmd_slide)
    sa = sub.add_parser("save")
    sa.add_argument("--press", type=float, default=PRESS)
    sa.set_defaults(go=cmd_save)
    sub.add_parser("try").set_defaults(go=cmd_try)
    sub.add_parser("status").set_defaults(go=cmd_status)
    sub.add_parser("park").set_defaults(go=cmd_park)
    args = p.parse_args()
    args.go(args)


if __name__ == "__main__":
    main()
