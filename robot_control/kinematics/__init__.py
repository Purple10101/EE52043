"""IK/FK mirror of the firmware, joint limits, reach and wrist choice. Pure maths, no I/O.

Plan: docs/plans/03-kinematics.md
"""

from robot_control.kinematics.arm import IKResult, Kinematics
from robot_control.kinematics.keep_out import KeepOut, first_hit, route

__all__ = ["IKResult", "Kinematics", "KeepOut", "first_hit", "route"]
