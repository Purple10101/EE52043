import math
L1, L2 = 125.0, 100.0
def ik(xr, yr):  # firmware scara.c, robot frame, mm
    r = math.hypot(xr, yr)
    if not (L1 - L2) <= r <= (L1 + L2): return None
    phi1 = math.acos((L2*L2 - r*r - L1*L1) / (-2*r*L1))
    phi2 = math.atan2(xr, yr)
    phi3 = math.acos((r*r - L1*L1 - L2*L2) / (-2*L1*L2))
    return math.degrees(phi2 - phi1), math.degrees(math.pi - phi3)
def on_plate(x, y, rad=5.0):  # scene mm, rounded corners
    cx = min(max(x, -50 + rad), 50 - rad); cy = min(max(y, -180 + rad), -80 - rad)
    return -50 <= x <= 50 and -180 <= y <= -80 and math.hypot(x - cx, y - cy) <= rad
step = 0.5
for elbow_max in (90, 120, 130, 134.4, 140, 150):
    n = ok = 0; rmin = 1e9
    y = -180.0
    while y <= -80:
        x = -50.0
        while x <= 50:
            if on_plate(x, y):
                n += 1
                q = ik(-x, -y)
                if q and -90 <= q[0] <= 90 and 0 <= q[1] <= elbow_max:
                    ok += 1
            x += step
        y += step
    print(f"elbow 0-{elbow_max}: {100*ok/n:.1f}% of plate reachable, {100-100*ok/n:.1f}% not")
print("plate centre (0,-130):", [round(v, 2) for v in ik(0, 130)])
print("min reach elbow 120: %.1f mm, elbow 90: %.1f mm" % (math.hypot(L1 + L2*math.cos(math.radians(120)), L2*math.sin(math.radians(120))), math.hypot(L1, L2)))
