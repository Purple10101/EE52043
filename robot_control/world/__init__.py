"""Shared data types (poses, detections, resistors, looks at the plate). Depends on nothing else in robot_control.
"""

from robot_control.world.types import Detection, JointState, PlateView, Resistor, Snapshot

__all__ = ["Detection", "JointState", "PlateView", "Resistor", "Snapshot"]
