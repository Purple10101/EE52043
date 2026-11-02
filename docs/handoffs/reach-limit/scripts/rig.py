import sys, numpy as np, collections
path = sys.argv[1]
verts = []; faces = collections.defaultdict(list); cur = None
for line in open(path):
    if line.startswith('v '): verts.append([float(t) for t in line.split()[1:4]])
    elif line[:2] in ('o ', 'g '): cur = line[2:].strip()
    elif line.startswith('f '): faces[cur].append([int(t.split('/')[0]) for t in line.split()[1:]])
V = np.array(verts) * 10.0  # cm -> mm
for name, fs in faces.items():
    idx = np.unique(np.concatenate([np.array(f) for f in fs])) - 1
    P = V[idx]; lo, hi = P.min(0), P.max(0)
    print(f"{name:12s} x {lo[0]:8.2f}..{hi[0]:8.2f}  y {lo[1]:8.2f}..{hi[1]:8.2f}  z {lo[2]:8.2f}..{hi[2]:8.2f}  ({len(fs)} faces)")
