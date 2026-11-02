"""Reach over the pick plate with the real arm + v3a.1 gripper, checked for collisions.
Frames: scene mm (origin base axis, z up). Arm STEP pose q0=-46.12, q1=110.42, shaft bottom z=40.
Links turn by -q about +z (camera_tower/scripts/arm_sweep.py convention). Robot frame = scene rotated 180."""
import sys, os, math, time, itertools
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape
from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Vec
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
S_DIR = sys.argv[1]
def load(d):
    out = []
    for k, n in [l.rstrip('\n').split('\t') for l in open(os.path.join(d, 'index.txt'))]:
        s = TopoDS_Shape(); BRepTools.Read_s(s, os.path.join(d, k + '.brep'), BRep_Builder()); out.append((n, s))
    return out
arm = load(os.path.join(S_DIR, 'arm_parts')); grip = load(os.path.join(S_DIR, 'grip_parts'))
def grp(n):
    if n.startswith(('base:', 'servos:1+servo_0', 'pots:1+pot_0')): return 'base'
    if n.startswith(('link1:', 'pots:1+pot_1')): return 'link1'
    if n.startswith('zpos:'): return 'zpos'
    return 'link2'
arm = [(n, grp(n), s) for n, s in arm]
# rig bodies as boxes (bounding boxes from the OBJ; conservative). Pick plate excluded: tips touch it.
rig = {'cradle Body1:2': (-130, -180, -50, -55, -135, -17), 'tray Body1:4': (-180, -130, -50, -80, -30, -40),
       'tray Body1:3': (-180, -25, -50, -80, 75, -40), 'corner block/tower foot': (-180, -180, -50, -135, -135, 260)}
rig = [(n, BRepPrimAPI_MakeBox(gp_Pnt(*b[:3]), gp_Pnt(*b[3:])).Shape()) for n, b in rig.items()]
L1, L2, ELBOW0, Q0S, Q1S, SB0 = 125.0, 100.0, gp_Pnt(90.1, -86.6, 0), -46.12, 110.42, 40.0
def rz(c, deg):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(c, gp_Dir(0, 0, 1)), math.radians(deg)); return t
def tr(x, y, z):
    t = gp_Trsf(); t.SetTranslation(gp_Vec(x, y, z)); return t
def ik(xr, yr):
    r = math.hypot(xr, yr)
    if not (L1 - L2) < r < (L1 + L2): return None
    phi1 = math.acos((L2*L2 - r*r - L1*L1) / (-2*r*L1)); phi2 = math.atan2(xr, yr)
    phi3 = math.acos((r*r - L1*L1 - L2*L2) / (-2*L1*L2))
    return math.degrees(phi2 - phi1), math.degrees(math.pi - phi3)
def bb(s):
    b = Bnd_Box(); BRepBndLib.Add_s(s, b, False); return b
def inter(a, b):
    c = BRepAlgoAPI_Common(a, b); g = GProp_GProps(); BRepGProp.VolumeProperties_s(c.Shape(), g); return g.Mass()
OPEN = 12.5
def gripper_shapes(px, py, sb, jaw_deg, open_):
    flip = gp_Trsf(); flip.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), math.pi)
    T = tr(px, py, sb - 9).Multiplied(rz(gp_Pnt(0, 0, 0), jaw_deg)).Multiplied(flip)
    out = []
    for n, s in grip:
        t = T
        if open_ and ('Jaw carrier' in n or 'Tip' in n):
            t = T.Multiplied(tr(OPEN if n.startswith('R ') else -OPEN, 0, 0))
        m = s.Moved(TopLoc_Location(t)); out.append((n, m, bb(m)))
    return out
arm_cache = {}
def arm_shapes(q0, q1, sb):
    t1 = rz(gp_Pnt(0, 0, 0), -(q0 - Q0S)); t2 = t1.Multiplied(rz(ELBOW0, -(q1 - Q1S)))
    tz = t2.Multiplied(tr(0, 0, sb - SB0)); out = []
    for n, g, s in arm:
        t = {'base': None, 'link1': t1, 'link2': t2, 'zpos': tz}[g]
        m = s if t is None else s.Moved(TopLoc_Location(t)); out.append((n, m, bb(m)))
    return out + [(n, s, bb(s)) for n, s in rig]
def clash(q0, q1, px, py, sb, jaw, open_):
    A = arm_shapes(q0, q1, sb); G = gripper_shapes(px, py, sb, jaw, open_)
    for gn, gs, gb in G:
        for an, as_, ab in A:
            if an.startswith('zpos:1+zpos_0'): continue  # the shaft the gripper is bolted to
            if gb.IsOut(ab): continue
            if inter(gs, as_) > 1.0: return f"{gn} x {an}"
    return None
step = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
xs = [-50 + step/2 + i*step for i in range(int(100/step))]; ys = [-180 + step/2 + i*step for i in range(int(100/step))]
JAWS = range(0, 360, 30); HEIGHTS = (42.0, 62.0, 87.0)
t0 = time.time(); n = 0; c120 = c141 = all6 = some = 0; why = {}; grid = {}; servo_side = {}
for py in ys:
    for px in xs:
        n += 1; q = ik(-px, -py)
        if not q or not (-90 <= q[0] <= 90) or not (0 <= q[1] <= 141): grid[(px, py)] = 'X'; continue
        c141 += 1; c120 += q[1] <= 120
        clear = {}
        for jaw in JAWS:
            bad = None
            for sb, op in itertools.product(HEIGHTS, (False, True)):
                bad = clash(q[0], q[1], px, py, sb, jaw, op)
                if bad: why[bad] = why.get(bad, 0) + 1; break
            clear[jaw] = bad is None
        yaws_ok = sum(clear[j] or clear[j + 180] for j in range(0, 180, 30))
        all6 += yaws_ok == 6; some += yaws_ok > 0
        grid[(px, py)] = ('o' if yaws_ok == 6 else str(yaws_ok)) + ('*' if q[1] > 120 else ' ')
    print(f"  row y={py:.0f} done ({time.time()-t0:.0f}s)", file=sys.stderr, flush=True)
print(f"cells: {n} ({step:.0f} mm grid)")
print(f"joint limits only: elbow <=120 {100*c120/n:.0f}%, elbow <=141 {100*c141/n:.0f}%")
print(f"elbow <=141, every resistor angle grippable (jaw or jaw+180 clear): {100*all6/n:.0f}%; some angles: {100*some/n:.0f}%")
print("map: rows scene x (far edge +50 at top); columns scene y -180 (left) .. -80 (base side, right)")
print("  o = all 6 resistor angles OK, digit = how many of 6 are OK, * = needs elbow 120-141")
for px in reversed(xs):
    print(f"  x={px:6.1f} " + ' '.join(grid[(px, py)] for py in ys))
print("blocking pairs:", sorted(why.items(), key=lambda kv: -kv[1])[:8])
