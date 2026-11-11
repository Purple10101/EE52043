"""Where a body is and which way it lies, in scene mm, from its outline in the rectified view.

Both take robot_offline's skeleton of the outline (post.skeleton.body_skeleton): its centroid is the
point the jaws aim for, and its spine gives the body's long axis.
"""

import numpy as np
from robot_offline.pre import rectify

from robot_control.vision.rig import correct_height


def centroid_mm(poly, body, cal, lens):
    """A body's centre in scene mm: its skeleton's centroid, or the outline's middle, moved for its height."""
    st = cal.settings
    centre = body.centroid if body is not None else np.asarray(poly, float).reshape(-1, 2).mean(0)
    scene = rectify.rect_to_mm(np.asarray(centre, float).reshape(1, 2), st)
    width_mm = (body.width / st.px_per_mm) if body is not None else 2.4
    return correct_height(scene, lens, st.top_z_mm, width_mm / 2)[0]


def body_direction(body, settings):
    """A body's long axis, scene deg from +x anticlockwise, from the ends of its spine."""
    a, b = rectify.rect_to_mm(np.asarray(body.spine, float)[[0, -1]], settings)
    return float(np.degrees(np.arctan2(*(b - a)[::-1])))
