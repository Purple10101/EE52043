"""Pose the imported EduSCARA (STEP) in the scene by moving its occurrences, and save frames.

Uses the kinematics in arm_sweep.py. The original occurrence transforms are stored in
arm_pose_originals.json next to this file the first time, and restore() puts them back.
"""
import json
import os
import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else r'C:\Users\joshp\Y4S1\EE52043\cad\camera_tower'
ORIG = os.path.join(HERE, 'arm_pose_originals.json')


def _moving(root, ns):
    out = []
    for o in root.allOccurrences:
        path = o.fullPathName
        names = [n for n in ns['LINK1'] | ns['LINK2'] if path == n]
        if names:
            g = ns['group_of'](path)
            zm = path in ns['ZMOVE']
            out.append((path, o, g, zm))
    # Z-moving children inside a moving parent (zpos_0 under zpos) are handled by moving them extra
    for o in root.allOccurrences:
        if o.fullPathName in ns['ZMOVE'] and not any(o.fullPathName == p for p, _, _, _ in out):
            out.append((o.fullPathName, o, 'link2', True))   # proxies take world transforms
    return out


def _save_originals(items):
    if os.path.exists(ORIG):
        return json.load(open(ORIG))
    data = {p: list(o.transform2.asArray()) for p, o, g, zm in items}
    json.dump(data, open(ORIG, 'w'))
    return data


def set_pose(root, ns, q0, q1, dz=0.0):
    items = _moving(root, ns)
    orig = _save_originals(items)
    for path, o, g, zm in items:
        m0 = adsk.core.Matrix3D.create()
        m0.setWithArray(orig[path])
        m0.transformBy(ns['pose_matrix'](g, q0, q1, dz, zm))   # world pose
        o.transform2 = m0


def restore(root, ns):
    items = _moving(root, ns)
    if not os.path.exists(ORIG):
        return
    orig = json.load(open(ORIG))
    for path, o, g, zm in items:
        m0 = adsk.core.Matrix3D.create()
        m0.setWithArray(orig[path])
        o.transform2 = m0
