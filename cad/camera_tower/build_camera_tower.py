"""Build the camera tower in the active Fusion design.

Everything is in the rig scene frame (mm in this file, cm inside Fusion), z up, origin at the
centre of the robot base. See the camera tower brief for the requirements.

Parts:
  Base (print)          45 x 45 x 5 foot on Body1:5's holes, 75 mm socket for a 2020 extrusion
  2020 extrusion        167 mm at a 200 mm lens height, slides +-25 mm in the socket
  Camera head (print)   one piece: sleeve over the extrusion top (4 x M5 into T-nuts) and a pocket
                        at the fixed aim angle; the camera (legs removed) slides in from the upper end
                        against a front frame (3 mm border) and two 3 mm side rails on the back,
                        leaving the rest of the back (and its connector, bottom right) uncovered
  Camera placeholder    36 x 36 x 10 body with lens, lens centre on the required point
"""
import math
import adsk.core
import adsk.fusion

tbm = adsk.fusion.TemporaryBRepManager.get()
UN = adsk.fusion.BooleanTypes.UnionBooleanType
DF = adsk.fusion.BooleanTypes.DifferenceBooleanType
IN = adsk.fusion.BooleanTypes.IntersectionBooleanType


# ---------- vector helpers (mm) ----------
def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def mul(a, k): return (a[0] * k, a[1] * k, a[2] * k)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def norm(a): return math.sqrt(dot(a, a))
def unit(a): n = norm(a); return (a[0] / n, a[1] / n, a[2] / n)
def lin(*terms):
    out = (0.0, 0.0, 0.0)
    for k, v in terms:
        out = add(out, mul(v, k))
    return out


def P(v): return adsk.core.Point3D.create(v[0] / 10, v[1] / 10, v[2] / 10)
def V(v): return adsk.core.Vector3D.create(v[0], v[1], v[2])


# ---------- solid helpers ----------
def box(c, ldir, wdir, Lmm, Wmm, Hmm):
    """Box centred at c. Length along ldir, width along wdir, height along ldir x wdir."""
    obb = adsk.core.OrientedBoundingBox3D.create(P(c), V(unit(ldir)), V(unit(wdir)), Lmm / 10, Wmm / 10, Hmm / 10)
    return tbm.createBox(obb)


def abox(x0, y0, z0, x1, y1, z1):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (1, 0, 0), (0, 1, 0), x1 - x0, y1 - y0, z1 - z0)


def cyl(p1, p2, r):
    return tbm.createCylinderOrCone(P(p1), r / 10, P(p2), r / 10)


def U(a, *bs):
    for b in bs:
        tbm.booleanOperation(a, b, UN)
    return a


def D(a, *bs):
    for b in bs:
        tbm.booleanOperation(a, b, DF)
    return a


def slot(c, sdir, half, axis, r, length):
    """Stadium slot centred at c, long along sdir (centre-to-centre 2*half), through along axis."""
    a = sub(c, mul(sdir, half))
    b = add(c, mul(sdir, half))
    ha = mul(axis, length / 2)
    s = cyl(sub(a, ha), add(a, ha), r)
    U(s, cyl(sub(b, ha), add(b, ha), r))
    if half > 0:
        U(s, box(c, sdir, axis, 2 * half, length, 2 * r))
    return s


def overlap(a, b):
    """Volume (mm^3) shared by two bodies."""
    c = tbm.copy(a)
    try:
        ok = tbm.booleanOperation(c, b, IN)
    except Exception:
        return 0.0
    if not ok:
        return 0.0
    return c.volume * 1000.0


def rotated(body, angle_deg, origin, axis):
    c = tbm.copy(body)
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.radians(angle_deg), V(axis), P(origin))
    tbm.transform(c, m)
    return c


# ---------- requirement geometry (from the brief) ----------
X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
LENS_HEIGHT = 200.0                   # lens above the plate top (brief: 250)
L = (-145.0, -145.0, -22.0 + LENS_HEIGHT)   # lens centre
T = (0.0, -130.0, -22.0)              # plate centre (aim point)
C = (-157.5, -157.5)                  # footprint / column centre
d = unit(sub(T, L))                   # optical axis (down toward the plate)
h = unit((d[0], d[1], 0.0))           # horizontal azimuth of the axis
u = (-h[1], h[0], 0.0)                # horizontal, across the view
e = cross(d, u)                       # image "up" direction (forward-up), perpendicular to d and u
assert e[2] > 0

# ---------- design parameters ----------
Z_PLATE_TOP = -45.0                   # top of the base plate (= Body1:5 top)
SOCKET_TOP = 30.0                     # socket for the extrusion, 75 mm tall
EXT_LEN = 217.0 - (250.0 - LENS_HEIGHT)   # 2020 extrusion length (217 mm at the brief's 250 mm)
EXT_BOTTOM = -20.0                    # nominal: 50 mm in the socket, can slide +-25 mm
EXT_TOP = EXT_BOTTOM + EXT_LEN
CAM_DEPTH, CAM_W, LENS_PROTRUDE = 10.0, 36.0, 4.0   # B030401 body with its legs removed
FIT = 0.2                             # clearance per side in the holder pocket
POCKET_DEPTH = 12.0                   # pocket takes a camera body up to 12 mm deep (measured 10); the camera rests on the front frame
WALL, LIP, BACK = 3.0, 2.0, 3.0       # holder side walls, front lip, back wall
RIM = 3.0                             # front frame border; the back is two side rails of this width
CAP_WALL = 8.0                        # sleeve wall over the extrusion end
NECK_BACK, NECK_FWD = 15.0, 15.0      # neck extent along the view azimuth either side of the column
BORE = 10.25                          # half of the 2020 bore: 20.5 mm, 0.25 mm per side for FDM
M3_R, M5_R = 1.7, 2.75                # clearance holes for FDM: 3.4 and 5.5 mm
PCOL_HALF = 14.0                      # printed column: 28 x 28 mm between the base socket and the head sleeve
PCOL_TENON = 20.0                     # printed column tenons (bore 20.5)
INSERT_R, INSERT_DEPTH = 3.2, 6.5     # M5 heat-set insert pilot (6.4 mm), for 5.8 mm long inserts
FOV_CLEAR = 18.0                      # half-angle of the cone kept clear in front of the lens (plate corners are at 14.1 deg)


def build(root, rig_comp=None, column='extrusion'):
    """column: 'extrusion' (2020 aluminium, M5 into T-nuts) or 'printed' (printed column, M5 into heat-set inserts)."""
    log = []
    Cxyz = (C[0], C[1], 0.0)

    # ---- camera placeholder ----
    B = lin((1, L), (-(LENS_PROTRUDE + CAM_DEPTH / 2), d))
    cam = box(B, d, u, CAM_DEPTH, CAM_W, CAM_W)
    U(cam, cyl(sub(L, mul(d, LENS_PROTRUDE + 0.5)), L, 7.0))

    # ---- camera head: holder pocket fixed at the required angle, on a block over the column ----
    # Local frame at the lens: u across, e image-up (forward-up), d along the axis (positive forward).
    def fbox(u0, u1, e0, e1, d0, d1):
        c = lin((1, L), ((u0 + u1) / 2, u), ((e0 + e1) / 2, e), ((d0 + d1) / 2, d))
        return box(c, u, e, u1 - u0, e1 - e0, d1 - d0)     # height dir u x e = d
    hw = CAM_W / 2 + FIT                                   # pocket half width (u and e)
    pf = -LENS_PROTRUDE + FIT                              # pocket front (d), camera face at -LENS_PROTRUDE
    pb = -LENS_PROTRUDE - POCKET_DEPTH - FIT               # pocket back (d): room for a body up to POCKET_DEPTH deep
    ow = hw + WALL                                         # outer half width
    e_top = hw                                             # entry end flush with the camera's top edge
    head = fbox(-ow, ow, -ow, e_top, pb - BACK, pf + LIP)
    D(head, fbox(-hw, hw, -hw, 80, pb, pf))                                          # pocket + entry path
    ho = CAM_W / 2 - RIM                                   # frame opening half width
    D(head, fbox(-ho, ho, -ho, 80, pf - 1, pf + LIP + 1))                            # front frame, open at +e
    D(head, fbox(-ho, ho, -hw, 80, pb - BACK - 1, pb + 1))                           # back: two 3 mm side rails only
    # neck: vertical block from the sleeve up to the holder's lower wall, trimmed to that wall's plane
    Ch = dot(sub((C[0], C[1], 0), (L[0], L[1], 0)), h)   # column centre along h, relative to the lens
    h0, h1 = Ch - NECK_BACK, Ch + NECK_FWD
    ztop = L[2] + 12.0
    zbot = EXT_TOP - 25.0                                  # down to the sleeve bottom, tapered at 45 deg (no flat overhangs)
    nc = lin((1, (L[0], L[1], 0)), ((h0 + h1) / 2, h), ((zbot + ztop) / 2, Z))
    neck = box(nc, h, u, h1 - h0, 2 * ow, ztop - zbot)
    r2 = 1 / math.sqrt(2)
    for pt, l, w, n in (((C[0], C[1] + 15, EXT_TOP - 20.0), X, (0, -r2, -r2), (0, r2, -r2)),
                        ((C[0] - 15, C[1], EXT_TOP + CAP_WALL), Y, (r2, 0, -r2), (-r2, 0, -r2))):   # -x face stays flat for the bolt heads
        D(neck, box(add(pt, mul(n, 100)), l, w, 400, 400, 200))     # 45 deg taper back to the sleeve walls
    D(neck, abox(C[0] + 15, -400, zbot - 1, 400, 400, EXT_TOP))     # no sliver past the sleeve's front face
    D(neck, fbox(-80, 80, -ow, 200, -200, 200))                                      # keep only below the lower wall
    D(neck, fbox(-80, 80, -200, 200, -200, pb - BACK))                                # nothing behind the back wall
    # sleeve over the extrusion top
    sleeve = abox(C[0] - 15, C[1] - 15, EXT_TOP - 25, C[0] + 15, C[1] + 15, EXT_TOP + CAP_WALL)
    U(head, neck, sleeve)
    D(head, abox(C[0] - BORE, C[1] - BORE, EXT_TOP - 26, C[0] + BORE, C[1] + BORE, EXT_TOP))
    D(head, tbm.createCylinderOrCone(P(sub(L, mul(d, 2))), 0.3, P(add(L, mul(d, 150))),
                                     (3.0 + 152 * math.tan(math.radians(FOV_CLEAR))) / 10))
    zt = EXT_TOP - 12.5
    for z in (zt - 6, zt + 6):   # M5 into T-nuts, -x and -y faces
        D(head, cyl((-185, C[1], z), (-160, C[1], z), M5_R))
        D(head, cyl((C[0], -185, z), (C[0], -160, z), M5_R))

    # ---- base ----
    base = abox(-180, -180, -50, -135, -135, Z_PLATE_TOP)
    U(base, abox(C[0] - 14, C[1] - 14, Z_PLATE_TOP - 0.01, C[0] + 14, C[1] + 14, SOCKET_TOP))
    # ribs on the +x and +y faces
    rx = abox(-143.6, C[1] - 1.5, Z_PLATE_TOP - 0.01, -135, C[1] + 1.5, 0)
    l = unit((8.5, 0, -40)); n = cross(l, Y)
    D(rx, box(add(((-143.5 - 135) / 2, C[1], -20), mul(n, 20)), l, Y, 100, 10, 40))
    ry = abox(C[0] - 1.5, -143.6, Z_PLATE_TOP - 0.01, C[0] + 1.5, -135, 0)
    l = unit((0, 8.5, -40)); n = cross(l, mul(X, -1))
    D(ry, box(add((C[0], (-143.5 - 135) / 2, -20), mul(n, 20)), l, mul(X, -1), 100, 10, 40))
    U(base, rx, ry)
    D(base, abox(C[0] - BORE, C[1] - BORE, Z_PLATE_TOP, C[0] + BORE, C[1] + BORE, SOCKET_TOP + 1))
    for hx, hy in ((-175, -175), (-140, -175), (-140, -140), (-175, -140)):
        D(base, cyl((hx, hy, -51), (hx, hy, -44), M3_R))
    for z in (10.0, 22.0):   # M5 into T-nuts, -x and -y walls
        D(base, cyl((-176, C[1], z), (-160, C[1], z), M5_R))
        D(base, cyl((C[0], -176, z), (C[0], -160, z), M5_R))

    # ---- 2020 extrusion (simplified profile) ----
    ext = abox(C[0] - 10, C[1] - 10, EXT_BOTTOM, C[0] + 10, C[1] + 10, EXT_TOP)
    for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        if sx:
            x0, x1 = sorted((C[0] + sx * 5.5, C[0] + sx * 10.5)); y0, y1 = C[1] - 3.1, C[1] + 3.1
        else:
            y0, y1 = sorted((C[1] + sy * 5.5, C[1] + sy * 10.5)); x0, x1 = C[0] - 3.1, C[0] + 3.1
        D(ext, abox(x0, y0, EXT_BOTTOM - 1, x1, y1, EXT_TOP + 1))
    D(ext, cyl((C[0], C[1], EXT_BOTTOM - 1), (C[0], C[1], EXT_TOP + 1), 2.1))

    if column == 'printed':
        # printed column: 20 x 20 tenons into the base socket and the head sleeve, 28 x 28 between them.
        # The shoulders set the height (socket top and sleeve bottom), so no spacer is needed.
        t = PCOL_TENON / 2
        sleeve_bot = EXT_TOP - 25.0
        ext = abox(C[0] - t, C[1] - t, EXT_BOTTOM, C[0] + t, C[1] + t, SOCKET_TOP + 1)
        U(ext, abox(C[0] - PCOL_HALF, C[1] - PCOL_HALF, SOCKET_TOP, C[0] + PCOL_HALF, C[1] + PCOL_HALF, sleeve_bot))
        U(ext, abox(C[0] - t, C[1] - t, sleeve_bot - 1, C[0] + t, C[1] + t, EXT_TOP))
        for z in (10.0, 22.0, EXT_TOP - 12.5 - 6, EXT_TOP - 12.5 + 6):   # heat-set insert holes behind the M5 bolts
            D(ext, cyl((C[0] - t - 1, C[1], z), (C[0] - t + INSERT_DEPTH, C[1], z), INSERT_R))
            D(ext, cyl((C[0], C[1] - t - 1, z), (C[0], C[1] - t + INSERT_DEPTH, z), INSERT_R))

    parts = [
        ('Base (print)', base),
        (('2020 extrusion %d mm' % EXT_LEN) if column == 'extrusion' else 'Column (print)', ext),
        ('Camera head (print)', head),
        ('Camera placeholder (B030401, 36 x 36 x 10)', cam),
    ]

    # ---- checks ----
    for n1, b1, n2, b2 in (('cam', cam, 'head', head), ('cam', cam, 'ext', ext), ('head', head, 'ext', ext)):
        v = overlap(b1, b2)
        if v > 0.01:
            log.append('CLASH %s x %s = %.1f mm3' % (n1, n2, v))
    sweep = fbox(-CAM_W / 2, CAM_W / 2, -CAM_W / 2, 80, -LENS_PROTRUDE - POCKET_DEPTH, -LENS_PROTRUDE)   # a 12 mm body
    U(sweep, fbox(-7, 7, -7, 80, -LENS_PROTRUDE - 0.5, 0))
    v = overlap(sweep, head)
    log.append('insertion path %s (%.2f mm3)' % ('clear' if v < 0.01 else 'BLOCKED', v))
    log.append('clash check done')

    rays = None
    start = add(L, mul(d, 6.0))
    for gx in (-50, -25, 0, 25, 50):
        for gy in (-180, -155, -130, -105, -80):
            r = cyl(start, (gx, gy, -22.0), 0.3)
            if rays is None:
                rays = r
            else:
                U(rays, r)
    for name, b in parts:
        v = overlap(rays, b)
        if v > 0.0001:
            log.append('SIGHTLINE BLOCKED by %s: %.3f mm3' % (name, v))
    log.append('sightline check done (25 rays to the plate)')

    # ---- add to the design ----
    top_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    top = top_occ.component
    top.name = 'Camera tower'
    for name, body in parts:
        occ = top.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = name
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        comp.bRepBodies.add(body, bf)
        bf.finishEdit()
        b = comp.bRepBodies.item(0)
        b.name = name
        bb = b.boundingBox
        log.append('%-40s x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f  vol %.1f cm3' % (
            name, bb.minPoint.x * 10, bb.maxPoint.x * 10, bb.minPoint.y * 10, bb.maxPoint.y * 10,
            bb.minPoint.z * 10, bb.maxPoint.z * 10, b.volume))

    # optical axis sketch for review
    sk = top.sketches.add(top.xYConstructionPlane)
    sk.name = 'Optical axis (lens to plate centre)'
    sk.sketchPoints.add(sk.modelToSketchSpace(P(L)))
    sk.sketchCurves.sketchLines.addByTwoPoints(sk.modelToSketchSpace(P(L)), sk.modelToSketchSpace(P(T)))

    log.append('d=%s u=%s e=%s' % tuple(str(tuple(round(c, 4) for c in v)) for v in (d, u, e)))
    log.append('tilt from vertical %.2f deg, pan %.2f deg, lens to aim %.1f mm' % (
        math.degrees(math.acos(-d[2])), math.degrees(math.atan2(h[1], h[0])), norm(sub(T, L))))
    return log
