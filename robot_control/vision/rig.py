"""Frames around the rig, pure maths.

Scene frame: the arm/rig CAD's, mm, origin at the robot base axis, z up. robot_offline's rectify
maps the camera image onto the plate's top in this frame. Robot frame: the firmware's, metres.
"""

import math

import numpy as np


def scene_to_robot(points_mm, rig):
    """Scene mm (N × 2) to robot m. rig: config.RigSettings."""
    p = np.asarray(points_mm, float).reshape(-1, 2) / 1000.0
    c, s = math.cos(math.radians(rig.rotation_deg)), math.sin(math.radians(rig.rotation_deg))
    return p @ np.array([[c, s], [-s, c]]) + np.asarray(rig.translation_m, float)


def robot_to_scene(points_m, rig):
    """Robot m (N × 2) to scene mm."""
    p = np.asarray(points_m, float).reshape(-1, 2) - np.asarray(rig.translation_m, float)
    c, s = math.cos(math.radians(rig.rotation_deg)), math.sin(math.radians(rig.rotation_deg))
    return p @ np.array([[c, -s], [s, c]]) * 1000.0


def correct_height(points_mm, camera_mm, surface_z_mm, height_mm):
    """Points seen on the plate surface (scene mm, N × 2) moved to where something height_mm above
    the surface really is. The camera sees it along its ray, which meets the surface further from the
    camera's foot: by height / (camera height) of the distance."""
    p = np.asarray(points_mm, float).reshape(-1, 2)
    foot = np.asarray(camera_mm[:2], float)
    above = camera_mm[2] - surface_z_mm
    return foot + (p - foot) * (above - height_mm) / above
