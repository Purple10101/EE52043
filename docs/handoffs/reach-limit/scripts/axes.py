import sys, os, collections
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
d = sys.argv[1]
idx = [l.rstrip('\n').split('\t') for l in open(os.path.join(d, 'index.txt'))]
want = sys.argv[2:]
for k, name in idx:
    if not any(w in name for w in want) or 'Button' in name or 'Nut' in name: continue
    s = TopoDS_Shape(); BRepTools.Read_s(s, os.path.join(d, k + '.brep'), BRep_Builder())
    cyl = collections.Counter()
    e = TopExp_Explorer(s, TopAbs_FACE)
    while e.More():
        a = BRepAdaptor_Surface(TopoDS.Face(e.Current()))
        if a.GetType() == GeomAbs_Cylinder:
            c = a.Cylinder(); ax = c.Axis(); dr = ax.Direction()
            if abs(dr.Z()) > 0.999:
                p = ax.Location()
                cyl[(round(p.X(), 1), round(p.Y(), 1), round(c.Radius(), 2))] += 1
        e.Next()
    big = sorted([kv for kv in cyl.items() if kv[0][2] >= 2.0], key=lambda kv: -kv[0][2])[:8]
    print(name, '->', ', '.join(f"({x},{y}) r{r}" for (x, y, r), n in big))
