import sys, os, math, time
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape
from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
d = sys.argv[1]
idx = [l.rstrip('\n').split('\t') for l in open(os.path.join(d, 'index.txt'))]
def grp(n):
    if n.startswith(('base:', 'servos:1+servo_0', 'pots:1+pot_0')): return 'base'
    if n.startswith(('link1:', 'pots:1+pot_1')): return 'link1'
    return 'link2'
parts = []
for k, n in idx:
    s = TopoDS_Shape(); BRepTools.Read_s(s, os.path.join(d, k + '.brep'), BRep_Builder())
    parts.append((n, grp(n), s))
ELBOW = gp_Pnt(90.1, -86.6, 0); Q1_STEP = 110.42; Q0_STEP = -46.12
def rot(about, deg):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(about, gp_Dir(0, 0, 1)), math.radians(deg)); return t
def bbox(s):
    b = Bnd_Box(); BRepBndLib.Add_s(s, b, False); return b
def vol(a, b):
    c = BRepAlgoAPI_Common(a, b)
    if not c.IsDone(): return -1
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(c.Shape(), g); return g.Mass()
def pose(q0, q1):
    # link1 fixed; link2 turned about the elbow by -(q1 - step); base turned by +(q0 - step) about the base axis
    t2 = rot(ELBOW, -(q1 - Q1_STEP)); tb = rot(gp_Pnt(0, 0, 0), (q0 - Q0_STEP))
    out = []
    for n, g, s in parts:
        m = s.Moved(TopLoc_Location(t2)) if g == 'link2' else s.Moved(TopLoc_Location(tb)) if g == 'base' else s
        out.append((n, g, m, bbox(m)))
    return out
def clashes(q0, q1, baseline=None):
    P = pose(q0, q1); hits = {}
    for i, (n2, g2, s2, b2) in enumerate(P):
        if g2 != 'link2': continue
        for n1, g1, s1, b1 in P:
            if g1 == 'link2' or b1.IsOut(b2): continue
            v = vol(s1, s2)
            if v > 0.5: hits[(n1, n2)] = v
    if baseline is not None:
        hits = {k: v for k, v in hits.items() if k not in baseline}
    return hits
t = time.time()
base0 = clashes(Q0_STEP, Q1_STEP)
print(f"baseline overlaps at the STEP pose ({time.time()-t:.0f}s):")
for k, v in base0.items(): print(f"   {v:8.1f} mm3  {k[0]}  x  {k[1]}")
q0s = [float(a) for a in sys.argv[2].split(',')]
for q0 in q0s:
    for q1 in [x / 2 for x in range(276, 286)]:
        h = clashes(q0, q1, base0)
        if h:
            worst = max(h.items(), key=lambda kv: kv[1])
            print(f"q0 {q0:6.1f}: first clash at elbow {q1} deg ({len(h)} pairs), worst {worst[1]:.0f} mm3: {worst[0][0]} x {worst[0][1]}")
            for k, v in sorted(h.items(), key=lambda kv: -kv[1])[:5]: print(f"      {v:8.1f}  {k[0]} x {k[1]}")
            break
    else:
        print(f"q0 {q0:6.1f}: no clash up to elbow 180")
    sys.stdout.flush()
