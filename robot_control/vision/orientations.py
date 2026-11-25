"""Segmentation run on each frame two ways round, the scores averaged.

seg_grey_v2 is less sure of bodies lying at about 120-140 deg in the rectified view (2026-11-23: the saved plate
rotated in 15 deg steps, three different bodies dipped to 0.46-0.71 there and scored 0.89-0.94 at every other
angle). So the model also sees the frame turned a quarter turn, where those bodies lie at 30-50 deg, and each
body's score is the mean of its two (COMBINE): a body found only one way round counts 0 for the other.

Both runs go down to FLOOR, so a body's "0" means the model gave it under FLOOR that way round, not that it was
cut off at the caller's threshold. Outlines are the better-scoring orientation's, in the frame's own pixels.

    polygons, scores = predict_two_ways(model, image, work_area, tile_size, overlap, conf=0.5, grey=True)
"""
import numpy as np
from shapely.geometry import Polygon

from robot_offline.inf import tiling

FLOOR = 0.05            # each orientation is run this low (body_memory.FLOOR, robot_offline inf/steady.py)
SAME_BODY_IOU = 0.3     # two outlines this much overlapped are one body seen both ways round
# How a body's two scores become one. On the 241 labelled real bodies (test sessions, 2026-11-23), found / false /
# missed at 0.5: one way only 231/1/10, "mean" 233/0/8, "max" 236/1/5. Mean chosen by the user (2026-11-23).
COMBINE = "mean"
_COMBINE = {"mean": lambda a, b: (a + b) / 2, "max": max}


def rotate_area(work_area, size):
    """work_area (x0, y0, w, h) in a size x size image, after np.rot90 (a quarter turn anticlockwise)."""
    x0, y0, w, h = work_area
    return (y0, size - x0 - w, h, w)


def unrotate_points(points, size):
    """Points (x, y) in the np.rot90'd image back to the original: (x, y) there came from (size-1-y, x)."""
    p = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    return np.column_stack([size - 1 - p[:, 1], p[:, 0]])


def _shape(points):
    shape = Polygon(np.asarray(points).reshape(-1, 2))
    return shape if shape.is_valid else shape.buffer(0)


def merge_mean(first, second, combine="mean"):
    """Two orientations' (polygons, scores), in the same pixels, as one: (polygons, scores, pairs), each body's
    score its two combined ("mean", or "max"; 0 for a way round it wasn't found), its outline the better-scoring
    one's. pairs: per body, (score first way, score second way), for logging."""
    join = _COMBINE[combine]
    shapes_a = [_shape(p) for p in first[0]]
    shapes_b = [_shape(p) for p in second[0]]
    candidates = []
    for i, a in enumerate(shapes_a):
        for j, b in enumerate(shapes_b):
            if not a.intersects(b):
                continue
            union = a.union(b).area
            iou = a.intersection(b).area / union if union > 0 else 0.0
            if iou >= SAME_BODY_IOU:
                candidates.append((iou, i, j))
    used_a, used_b, matched = set(), set(), []
    for iou, i, j in sorted(candidates, reverse=True):
        if i not in used_a and j not in used_b:
            used_a.add(i), used_b.add(j), matched.append((i, j))
    bodies = [(i, j) for i, j in matched]
    bodies += [(i, None) for i in range(len(shapes_a)) if i not in used_a]
    bodies += [(None, j) for j in range(len(shapes_b)) if j not in used_b]
    polygons, scores, pairs = [], [], []
    for i, j in bodies:
        sa = first[1][i] if i is not None else 0.0
        sb = second[1][j] if j is not None else 0.0
        polygons.append(np.asarray(first[0][i] if (j is None or (i is not None and sa >= sb)) else second[0][j],
                                   dtype=float))
        scores.append(join(sa, sb))
        pairs.append((sa, sb))
    return polygons, scores, pairs


def predict_two_ways(model, image, work_area, tile_size, overlap, conf, grey, predict=tiling.predict_frame,
                     with_pairs=False, combine=None):
    """(polygons, scores) for a square rectified image: the model run on it as it is and turned a quarter turn,
    each body's two scores combined (COMBINE unless given), kept if that is at least conf. with_pairs: also each
    body's two scores. predict: tiling.predict_frame, or a stand-in for tests."""
    h, w = image.shape[:2]
    if h != w:
        raise ValueError(f"predict_two_ways needs a square image, not {w} x {h}")
    floor = min(FLOOR, conf)
    straight = predict(model, image, tiling.tiles(work_area, (w, h), tile_size, overlap), conf=floor, grey=grey)
    turned_image = np.ascontiguousarray(np.rot90(image))
    turned_area = rotate_area(work_area, w)
    turned = predict(model, turned_image, tiling.tiles(turned_area, (w, h), tile_size, overlap), conf=floor, grey=grey)
    turned = ([unrotate_points(p, w) for p in turned[0]], list(turned[1]))
    polygons, scores, pairs = merge_mean((list(straight[0]), list(straight[1])), turned, combine or COMBINE)
    keep = sorted((k for k, s in enumerate(scores) if s >= conf), key=lambda k: -scores[k])   # best first, as tiling
    out = [polygons[k] for k in keep], [scores[k] for k in keep]
    return (*out, [pairs[k] for k in keep]) if with_pairs else out
