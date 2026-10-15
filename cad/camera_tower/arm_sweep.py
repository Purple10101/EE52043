"""Sweep the EduSCARA through its joint limits and check for collisions with the camera tower.

Run inside Fusion (scene document with the STEP arm, rig and camera tower).
Kinematics (rig frame, mm): base axis at (0, 0), elbow axis at (0, -125), tip at L1 = 125, L2 = 100.
In the Unity sim (course_files/scara/mini_scara_simulator) the robot's servo frame and the rig model
share one transform, and at zero the arm lies straight along -y. The STEP pose is q0 = 0, q1 = 90
(link 2 along -x). Sim joint limits: base -90..90, elbow 0..120, wrist -180..180, Z 10..100 mm.
The sim's x, y (FK) are the rig frame rotated 180 deg, so a joint angle q turns the link by -q.
"""
import math
import adsk.core
import adsk.fusion

tbm = adsk.fusion.TemporaryBRepManager.get()
ELBOW = (0.0, -125.0)
Q1_STEP = 90.0

LINK1 = {'link1:1', 'pots:1+pot_1:1'}
LINK2 = {'link2:1', 'zrot:1', 'zpos:1', 'shell:1', 'Gripper v3 - Housing:1',
         'servos:1+servo_1:1', 'servos:1+servo_2:1', 'servos:1+servo_3:1'}
ZMOVE = {'zpos:1+zpos_0:1', 'Gripper v3 - Housing:1'}   # carried up and down by the Z axis
WRIST = ZMOVE                                           # turned by the wrist about the tip axis
TIP = (-100.0, -125.0)                                  # tip axis in the STEP pose


def group_of(path):
    for g, names in (('link2', LINK2), ('link1', LINK1)):
        for n in names:
            if path == n or path.startswith(n + '+'):
                return g
    return None


def rot_pt(p, ang, cx, cy):
    c, s = math.cos(ang), math.sin(ang)
    x, y = p[0] - cx, p[1] - cy
    return (cx + c * x - s * y, cy + s * x + c * y, p[2])


def pose_point(p, g, q0, q1, dz, zmove, w=0.0):
    if g == 'link2':
        if zmove:
            p = rot_pt((p[0], p[1], p[2] + dz), math.radians(w), TIP[0], TIP[1])
        p = rot_pt(p, math.radians(-(q1 - Q1_STEP)), ELBOW[0], ELBOW[1])
    return rot_pt(p, math.radians(-q0), 0.0, 0.0)


def pose_matrix(g, q0, q1, dz, zmove, w=0.0):
    m = adsk.core.Matrix3D.create()
    if g == 'link2':
        if zmove:
            t = adsk.core.Matrix3D.create()
            t.translation = adsk.core.Vector3D.create(0, 0, dz / 10)
            m.transformBy(t)
            r = adsk.core.Matrix3D.create()
            r.setToRotation(math.radians(w), adsk.core.Vector3D.create(0, 0, 1),
                            adsk.core.Point3D.create(TIP[0] / 10, TIP[1] / 10, 0))
            m.transformBy(r)
        r = adsk.core.Matrix3D.create()
        r.setToRotation(math.radians(-(q1 - Q1_STEP)), adsk.core.Vector3D.create(0, 0, 1),
                        adsk.core.Point3D.create(ELBOW[0] / 10, ELBOW[1] / 10, 0))
        m.transformBy(r)
    r = adsk.core.Matrix3D.create()
    r.setToRotation(math.radians(-q0), adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    m.transformBy(r)
    return m


def bbox_mm(b):
    bb = b.boundingBox
    return (bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10,
            bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10)


def collect(root):
    arm, tower = [], []
    for o in root.allOccurrences:
        path = o.fullPathName
        for b in o.bRepBodies:
            if path.startswith('Camera tower'):
                tower.append((b.name, tbm.copy(b), bbox_mm(b)))
            else:
                g = group_of(path)
                if g:
                    zm = any(path == n or path.startswith(n + '+') for n in ZMOVE)
                    arm.append((path, g, zm, tbm.copy(b), bbox_mm(b)))
    return arm, tower


def overlap_aabb(a, b, pad=0.5):
    return all(a[i] - pad <= b[i + 3] and b[i] - pad <= a[i + 3] for i in range(3))


def posed_aabb(bb, g, q0, q1, dz, zm, w=0.0):
    pts = [pose_point((x, y, z), g, q0, q1, dz, zm, w) for x in (bb[0], bb[3]) for y in (bb[1], bb[4]) for z in (bb[2], bb[5])]
    return (min(p[0] for p in pts), min(p[1] for p in pts), min(p[2] for p in pts),
            max(p[0] for p in pts), max(p[1] for p in pts), max(p[2] for p in pts))


def sweep(root, q0s, q1s, dzs):
    arm, tower = collect(root)
    hits = {}
    for dz in dzs:
        for q0 in q0s:
            for q1 in q1s:
                for path, g, zm, body, bb in arm:
                    pb = posed_aabb(bb, g, q0, q1, dz, zm)
                    for tname, tb, tbb in tower:
                        if not overlap_aabb(pb, tbb):
                            continue
                        c = tbm.copy(body)
                        tbm.transform(c, pose_matrix(g, q0, q1, dz, zm))
                        try:
                            ok = tbm.booleanOperation(c, tbm.copy(tb), adsk.fusion.BooleanTypes.IntersectionBooleanType)
                        except Exception:
                            ok = False
                        if ok and c.volume > 1e-6:
                            hits.setdefault((q0, q1, dz), set()).add((path.split('+')[0], tname))
    return hits


def pose_hits(arm, tower, q0, q1, dzs=(0.0,), ws=(0.0,), first=True):
    """Tower parts hit at (q0, q1) for any Z lift in dzs and wrist angle in ws."""
    hits = set()
    for dz in dzs:
        for w in ws:
            for path, g, zm, body, bb in arm:
                if not zm and (dz != dzs[0] or w != ws[0]):
                    continue                     # only the Z/wrist parts change with dz and w
                pb = posed_aabb(bb, g, q0, q1, dz, zm, w)
                for tname, tb, tbb in tower:
                    if not overlap_aabb(pb, tbb):
                        continue
                    c = tbm.copy(body)
                    tbm.transform(c, pose_matrix(g, q0, q1, dz, zm, w))
                    try:
                        ok = tbm.booleanOperation(c, tbm.copy(tb), adsk.fusion.BooleanTypes.IntersectionBooleanType)
                    except Exception:
                        ok = False
                    if ok and c.volume > 1e-6:
                        hits.add((path.split('+')[0], tname))
                        if first:
                            return hits
    return hits
