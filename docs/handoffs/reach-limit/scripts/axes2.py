import sys, os, collections
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
d = sys.argv[1]
for k, name in [l.rstrip('\n').split('\t') for l in open(os.path.join(d, 'index.txt'))]:
    if name not in sys.argv[2:]: continue
    s = TopoDS_Shape(); BRepTools.Read_s(s, os.path.join(d, k + '.brep'), BRep_Builder())
    e = TopExp_Explorer(s, TopAbs_FACE); rows = collections.defaultdict(lambda: [1e9, -1e9])
    while e.More():
        f = TopoDS.Face(e.Current()); a = BRepAdaptor_Surface(f)
        if a.GetType() == GeomAbs_Cylinder:
            c = a.Cylinder(); dr = c.Axis().Direction(); p = c.Axis().Location()
            if abs(dr.Z()) > 0.999:
                b = Bnd_Box(); BRepBndLib.Add_s(f, b, False)
                key = (round(p.X(), 2), round(p.Y(), 2), round(c.Radius(), 2))
                rows[key][0] = min(rows[key][0], b.CornerMin().Z()); rows[key][1] = max(rows[key][1], b.CornerMax().Z())
        e.Next()
    print(name)
    for (x, y, r), (z0, z1) in sorted(rows.items(), key=lambda kv: (kv[1][0], -kv[0][2])):
        print(f"   axis ({x:6.2f},{y:6.2f})  r {r:5.2f}  z {z0:6.2f}..{z1:6.2f}")
