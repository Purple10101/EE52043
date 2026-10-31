"""Builds a keep-out map: the (base, elbow) joint angles at which the arm or gripper would hit an
obstacle, here the camera tower. Writes robot_control/config/keep_out/<name>.json, which
robot_control.kinematics.KeepOut reads.

The firmware moves every joint on the same time-scaled profile (move_js_scara), so a move is a
straight line in joint space; a map over joint angles is what a path check needs.

For every base and elbow angle on the grid, the arm STEP is posed (links turn by -q about +z in the
rig scene frame, camera_tower/scripts/arm_sweep.py's convention) with the gripper STEP bolted under
the Z shaft, the slide at the bottom, middle and top of its stroke and the wrist every 30 deg. Every
part, the arm's and the obstacles', is a cloud of points sampled over its surface. A cell is blocked
if any arm or gripper point of any of those poses comes within the margin of an obstacle point. Two
solids overlap only where their surfaces cross, so with the margin larger than the point spacing no
contact is missed. Obstacles are the tower's parts, both column versions together.

Needs OpenCascade's Python bindings and scipy, kept out of the project venv:

    py -3.11 -m venv cadenv
    cadenv\\Scripts\\python -m pip install cadquery-ocp numpy scipy
    cadenv\\Scripts\\python scripts\\keep_out_map.py

Defaults point at the CAD in ../Robot Resources (see --help).
"""

import argparse
import json
import math
import time
from datetime import date
from pathlib import Path

import numpy as np

RESOURCES = Path(__file__).resolve().parents[2] / "Robot Resources"
ARM_STEP = RESOURCES / "arm_and_rig_scene" / "EduSCARA arm - no gripper (plate reach pose).step"
GRIPPER_STEP = RESOURCES / "gripper" / "v3_final" / "CAD" / "Gripper v3a.1 - Bolt-on Tips.step"
TOWER_STEPS = [RESOURCES / "camera_tower" / "print" / "camera_tower_assembly.step",
               RESOURCES / "camera_tower" / "printed_column_variant" / "camera_tower_printed_column_assembly.step"]
OUT = Path(__file__).resolve().parents[1] / "robot_control" / "config" / "keep_out" / "camera_tower.json"

# The arm STEP's pose (arm_and_rig_scene/README.txt) and its joint axes, measured from its bores.
Q0_STEP, Q1_STEP = -46.12, 110.42
ELBOW_MM = (90.1, -86.6)
SHAFT_MM = (0.0, -130.0)
SHAFT_BOTTOM_STEP = 40.0        # the shaft's bottom face in the STEP pose, scene z mm
SOCKET_FLOOR = -9.0             # the gripper socket's floor in the gripper STEP's frame (on the shaft's bottom face)
SLIDE_BOTTOMS = (37.0, 62.0, 87.0)  # shaft bottom over the ~50 mm stroke; the tips touch the plate at 42
WRISTS = range(0, 360, 30)
SPACING_MM = 2.0                # surface points about this far apart


def read_parts(path):
    """[(name, shape)] for every solid part of a STEP assembly, placed in its frame."""
    from OCP.OCP.collections import Sequence_TDF_Label
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != 1:
        raise OSError(f"cannot read {path}")
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(label):
        a = TDataStd_Name()
        return a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), a) else "?"

    out = []

    def walk(label, loc, path):
        ref = label
        if tool.IsReference_s(label):
            ref = TDF_Label()
            tool.GetReferredShape_s(label, ref)
            loc = loc * tool.GetLocation_s(label)
        path = path + [name(label)]
        if tool.IsAssembly_s(ref):
            children = Sequence_TDF_Label()
            tool.GetComponents_s(ref, children)
            for i in range(1, children.Length() + 1):
                walk(children.Value(i), loc, path)
        else:
            out.append(("+".join(path[1:]), tool.GetShape_s(ref).Moved(loc)))

    roots = Sequence_TDF_Label()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    return out


def surface_points(shape, spacing, rng):
    """Points over a shape's surface, about spacing apart (N x 3, mm), or None if it has no faces."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    BRepMesh_IncrementalMesh(shape, 0.3, False, 0.3, True)
    chunks = []
    e = TopExp_Explorer(shape, TopAbs_FACE)
    while e.More():
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(TopoDS.Face(e.Current()), loc)
        e.Next()
        if tri is None:
            continue
        t = loc.Transformation()
        nodes = []
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(t)
            nodes.append((p.X(), p.Y(), p.Z()))
        nodes = np.array(nodes)
        tris = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
        a, b, c = nodes[tris[:, 0]], nodes[tris[:, 1]], nodes[tris[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        count = np.ceil(area / (spacing * spacing)).astype(int)
        idx = np.repeat(np.arange(len(tris)), count)
        u, v = rng.random(len(idx)), rng.random(len(idx))
        flip = u + v > 1
        u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
        chunks += [nodes, a[idx] + u[:, None] * (b - a)[idx] + v[:, None] * (c - a)[idx]]
    return np.vstack(chunks) if chunks else None


def rz(deg, about=(0.0, 0.0)):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    m[:2, 3] = np.array(about) - m[:2, :2] @ np.array(about)
    return m


def tr(x, y, z):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


FLIP = np.diag([1.0, -1.0, -1.0, 1.0])     # 180 deg about x: the gripper hangs tips down


def group(name):
    if name.startswith(("base:", "servos:1+servo_0", "pots:1+pot_0")):
        return None                          # fixed to the base: never moves near the tower
    if name.startswith(("link1:", "pots:1+pot_1")):
        return "link1"
    if name.startswith(("zpos:1+zpos_0", "zpos:1+zpos_1")):
        return "slide"                       # the Z shaft and its rack move with the slide
    return "link2"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--arm", type=Path, default=ARM_STEP)
    p.add_argument("--gripper", type=Path, default=GRIPPER_STEP)
    p.add_argument("--tower", type=Path, nargs="+", default=TOWER_STEPS)
    p.add_argument("--margin", type=float, default=10.0,
                   help="clearance in mm the arm must keep from every obstacle surface (10; at least 3)")
    p.add_argument("--step", type=float, default=1.0, help="grid step in degrees (1)")
    p.add_argument("--elbow-max", type=float, default=141.0, help="top of the elbow grid: where the links touch (141)")
    p.add_argument("--no-gripper", action="store_true", help="the arm alone, for comparison")
    p.add_argument("--out", type=Path, default=OUT)
    args = p.parse_args(argv)
    if args.margin < 3.0:
        p.error("--margin must be at least 3 mm: surfaces are sampled about 2 mm apart")
    from scipy.spatial import cKDTree
    rng = np.random.default_rng(0)
    t0 = time.time()

    # Obstacles: their surfaces, densely sampled. Two solids overlap only where their surfaces cross,
    # so an arm point within the margin of an obstacle point catches every contact.
    obstacle_parts, clouds = [], []
    for path in args.tower:
        for name, shape in read_parts(path):
            pts = surface_points(shape, 1.0, rng)
            if pts is not None:
                obstacle_parts.append(f"{path.stem}: {name.split('+')[-1]}")
                clouds.append(pts)
    tree = cKDTree(np.vstack(clouds))
    lo = np.array([c.min(0) - args.margin for c in clouds])      # per part, for a quick first test
    hi = np.array([c.max(0) + args.margin for c in clouds])
    print(f"{len(clouds)} obstacle parts, {tree.n} surface points, clearance {args.margin} mm")

    def inside(points):
        """Whether any point is within the margin of an obstacle's surface."""
        return bool(np.isfinite(tree.query(points, distance_upper_bound=args.margin)[0]).any())

    parts = {"link1": [], "link2": [], "slide": [], "gripper": []}
    for name, shape in read_parts(args.arm):
        g = group(name)
        if g is not None:
            pts = surface_points(shape, SPACING_MM, rng)
            if pts is not None:
                parts[g].append((name, np.c_[pts, np.ones(len(pts))]))
    for name, shape in [] if args.no_gripper else read_parts(args.gripper):
        pts = surface_points(shape, SPACING_MM, rng)
        if pts is not None:
            parts["gripper"].append((name, np.c_[pts, np.ones(len(pts))]))
    corners = {g: np.array([[[x, y, z, 1.0] for x in (q[:, 0].min(), q[:, 0].max()) for y in (q[:, 1].min(), q[:, 1].max())
                             for z in (q[:, 2].min(), q[:, 2].max())] for _, q in items]) for g, items in parts.items()}
    print(f"points: " + ", ".join(f"{g} {sum(len(q) for _, q in items)}" for g, items in parts.items())
          + f"  ({time.time() - t0:.0f} s)")

    def hits(g, m):
        """Names of the parts of group g, posed by m, within the margin of an obstacle."""
        if not parts[g]:
            return []
        c = corners[g] @ m.T
        near = ((c[:, :, :3].max(1)[:, None, :] >= lo) & (c[:, :, :3].min(1)[:, None, :] <= hi)).all(2).any(1)
        return [name for k, (name, q) in enumerate(parts[g]) if near[k] and inside((q @ m.T)[:, :3])]

    j0s = np.arange(-90.0, 90.0 + 1e-9, args.step)
    j1s = np.arange(0.0, args.elbow_max + 1e-9, args.step)
    blocked = np.zeros((len(j0s), len(j1s)), bool)
    culprits = {}
    for a, q0 in enumerate(j0s):
        m1 = rz(-(q0 - Q0_STEP))
        for b, q1 in enumerate(j1s):
            m2 = m1 @ rz(-(q1 - Q1_STEP), ELBOW_MM)
            found = hits("link1", m1) + hits("link2", m2)
            for z in SLIDE_BOTTOMS:
                if found:
                    break
                found = hits("slide", m2 @ tr(0, 0, z - SHAFT_BOTTOM_STEP))
                for w in WRISTS:
                    if found:
                        break
                    # flipped, the socket floor (gripper z -9) lands on the shaft's bottom face at z
                    found = hits("gripper", m2 @ tr(*SHAFT_MM, z + SOCKET_FLOOR) @ rz(w) @ FLIP)
            if found:
                blocked[a, b] = True
                for n in found:
                    culprits[n] = culprits.get(n, 0) + 1
        if a % 30 == 0:
            print(f"  base {q0:6.1f}  ({time.time() - t0:.0f} s)", flush=True)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({
        "name": "camera tower",
        "made": str(date.today()),
        "by": "scripts/keep_out_map.py",
        "sources": {"arm": args.arm.name, "gripper": None if args.no_gripper else args.gripper.name,
                    "obstacles": [t.name for t in args.tower]},
        "obstacle_parts": obstacle_parts,
        "margin_mm": args.margin,           # clearance kept from every obstacle surface
        "slide_bottoms_mm": list(SLIDE_BOTTOMS),
        "wrist_step_deg": 30,
        "j0_deg": [float(j0s[0]), float(j0s[-1]), args.step],
        "j1_deg": [float(j1s[0]), float(j1s[-1]), args.step],
        "blocked": ["".join("1" if v else "0" for v in row) for row in blocked],   # one string per base angle
        "parts_hit": dict(sorted(culprits.items(), key=lambda kv: -kv[1])),
    }, indent=1))
    print(f"{blocked.sum()} of {blocked.size} cells blocked; wrote {args.out} ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
