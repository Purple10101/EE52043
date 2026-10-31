import sys
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopLoc import TopLoc_Location
import pickle
from OCP.BRepTools import BRepTools
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
r = STEPCAFControl_Reader(); r.SetNameMode(True)
assert r.ReadFile(sys.argv[1]) == 1
r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
def name(lab):
    a = TDataStd_Name()
    return a.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), a) else '?'
out = []
def walk(lab, loc, path, depth):
    ref = lab
    if st.IsReference_s(lab):
        from OCP.TDF import TDF_Label
        ref = TDF_Label(); st.GetReferredShape_s(lab, ref)
        loc = loc * st.GetLocation_s(lab)
    nm = name(lab)
    p = path + [nm]
    if st.IsAssembly_s(ref):
        ch = TDF_LabelSequence(); st.GetComponents_s(ref, ch)
        for i in range(1, ch.Length() + 1): walk(ch.Value(i), loc, p, depth + 1)
    else:
        s = st.GetShape_s(ref).Moved(loc)
        b = Bnd_Box(); BRepBndLib.Add_s(s, b, False)
        if b.IsVoid(): return
        (x0,y0,z0),(x1,y1,z1) = [(p.X(),p.Y(),p.Z()) for p in (b.CornerMin(), b.CornerMax())]
        out.append(('+'.join(p[1:]), s))
        if '-q' not in sys.argv: print(f"{'+'.join(p[1:]):70s} x {x0:7.1f}..{x1:7.1f} y {y0:7.1f}..{y1:7.1f} z {z0:7.1f}..{z1:7.1f}")
roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
for i in range(1, roots.Length() + 1): walk(roots.Value(i), TopLoc_Location(), [], 0)
# save each leaf as brep for later steps
import os
d = sys.argv[2]; os.makedirs(d, exist_ok=True)
with open(os.path.join(d, 'index.txt'), 'w') as f:
    for k, (n, s) in enumerate(out):
        BRepTools.Write_s(s, os.path.join(d, f'{k:03d}.brep')); f.write(f'{k:03d}\t{n}\n')
