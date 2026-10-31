"""Draws a keep-out map in joint space (base across, elbow up), with the joint angles the arm needs
to reach the pick plate and the trays, so you can see what the zone costs.

    python scripts\\show_keep_out.py                                        # the configured maps
    python scripts\\show_keep_out.py --map path\\to\\map.json --compare other.json --out view.png

--compare draws another map's blocked cells as outlines: a keep_out_map.py JSON, or the camera tower
README's analysis/collision_map_data.json ({"q0,q1": [hits]}).
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.config import ArmSettings, KeepOutMap, RigSettings
from robot_control.kinematics import Kinematics
from robot_control.vision.rig import scene_to_robot

PX = 4                      # px per degree
PAD = 60
# Rig bodies to show, scene mm boxes (x0, y0, x1, y1) from EduSCARA_resistorrig.obj.
TARGETS = {"pick plate": ((-50, -180, 50, -80), (90, 200, 90)),
           "tray Body1:4": ((-180, -130, -80, -30), (230, 160, 40)),
           "tray Body1:3": ((-180, -25, -80, 75), (200, 90, 200))}


def to_px(j0, j1, height):
    return int(round(PAD + (j0 + 90) * PX)), int(round(height - PAD - j1 * PX))


def cells(m):
    """(j0, j1, step0, step1) of every blocked cell of a KeepOutMap."""
    a, b = np.nonzero(m.blocked)
    return [(m.j0[0] + i * m.j0[1], m.j1[0] + j * m.j1[1], m.j0[1], m.j1[1]) for i, j in zip(a, b)]


def load_compare(path):
    d = json.loads(Path(path).read_text())
    if "blocked" in d:
        return cells(KeepOutMap.load(path))
    keys = [tuple(map(float, k.split(","))) for k, v in d.items() if v]
    return [(q0, q1, 2.5, 2.5) for q0, q1 in keys]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--map", type=Path, nargs="*", help="map JSONs (default: workspace.toml's keep_out list)")
    p.add_argument("--compare", type=Path, help="another map, drawn as outlines")
    p.add_argument("--out", type=Path, default=Path("logs/keep_out.png"))
    args = p.parse_args(argv)

    maps = [KeepOutMap.load(m) for m in args.map] if args.map else list(KeepOutMap.from_config())
    arm = ArmSettings.from_config()
    kin, rig = Kinematics(arm), RigSettings.from_config()
    w, h = 2 * PAD + 180 * PX, 2 * PAD + 145 * PX + 40      # 40 px more at the top for the legend
    img = np.full((h, w, 3), 250, np.uint8)

    for m in maps:
        for j0, j1, s0, s1 in cells(m):
            x0, y0 = to_px(j0 - s0 / 2, j1 + s1 / 2, h)
            x1, y1 = to_px(j0 + s0 / 2, j1 - s1 / 2, h)
            cv2.rectangle(img, (x0, y0), (x1, y1), (90, 90, 220), -1)
    if args.compare:
        for j0, j1, s0, s1 in load_compare(args.compare):
            x0, y0 = to_px(j0 - s0 / 2, j1 + s1 / 2, h)
            x1, y1 = to_px(j0 + s0 / 2, j1 - s1 / 2, h)
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 160, 230), 1)

    for name, ((x0, y0, x1, y1), colour) in TARGETS.items():
        xs, ys = np.meshgrid(np.arange(x0, x1 + 0.1, 2.5), np.arange(y0, y1 + 0.1, 2.5))
        for x, y in scene_to_robot(np.c_[xs.ravel(), ys.ravel()], rig):
            r = kin.ik(x, y, 0.0, arm.z[0])
            if r.ok:
                cv2.circle(img, to_px(r.joints.j0, r.joints.j1, h), 1, colour, -1)

    # Axes, the elbow limit and labels.
    cv2.rectangle(img, to_px(-90, 141, h), to_px(90, 0, h), (0, 0, 0), 1)
    cv2.line(img, to_px(-90, arm.j1[1], h), to_px(90, arm.j1[1], h), (0, 0, 0), 1, cv2.LINE_AA)
    put = lambda s, org, c=(0, 0, 0), sc=0.45: cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, c, 1, cv2.LINE_AA)
    put(f"elbow limit {arm.j1[1]:.0f}", (to_px(-88, arm.j1[1], h)[0], to_px(0, arm.j1[1], h)[1] - 4))
    for j0 in range(-90, 91, 30):
        x, y = to_px(j0, 0, h)
        put(f"{j0}", (x - 10, y + 18))
    for j1 in range(0, 141, 20):
        x, y = to_px(-90, j1, h)
        put(f"{j1}", (x - 32, y + 4))
    put("base angle (deg)", (w // 2 - 60, h - 14))
    put("elbow (deg)", (6, PAD - 20))
    title = ", ".join(f"{m.name} ({m.margin_mm:g} mm clearance)" for m in maps) or "no maps"
    put(f"blocked: {title}" + ("   outlines: " + args.compare.name if args.compare else ""), (PAD, 24), sc=0.5)
    y = 44
    for name, (_, colour) in TARGETS.items():
        put(f"dots: {name} (reachable joint angles)", (PAD, y), colour, 0.45)
        y += 16
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out), img)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
