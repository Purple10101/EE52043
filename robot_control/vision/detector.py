"""A raw camera frame to Detections in the robot frame.

1. Calibrate: find the plate in the frame (robot_offline rectify), from scratch or near the last fit.
2. Detect: the frame rectified to a top-down view of the plate, white-balanced and masked as in
   training, through the segmentation model on tiles. Outlines come out in rectified px.
3. Map: rectified px to scene mm on the plate surface; each body's outline moved for its height
   above the plate (the camera sees it at an angle); scene mm to robot m.
"""

import cv2
import numpy as np

from robot_offline import config as offline_config
from robot_offline.inf import tiling
from robot_offline.pre import rectify

from robot_control.vision.rig import correct_height, scene_to_robot
from robot_control.world import Detection


class Detector:
    def __init__(self, vision, rig, workspace, model=None):
        """vision, rig, workspace: config settings. model: an ultralytics YOLO, loaded from
        vision.weights if None."""
        self.vision, self.rig, self.workspace = vision, rig, workspace
        dataset = offline_config.load("dataset")
        w = dataset["work_area"]
        self.work_area = (w["x0"], w["y0"], w["width"], w["height"])
        self.tile_size, self.tile_overlap = dataset["tiles"]["size"], dataset["tiles"]["overlap"]
        self._model = model

    @property
    def model(self):
        if self._model is None:
            from ultralytics import YOLO
            weights = self.vision.weights
            if not weights.is_absolute():
                weights = offline_config.ROOT / weights
            if not weights.exists():
                raise FileNotFoundError(f"{weights} not found (in the robot_offline repo, run git lfs pull)")
            self._model = YOLO(str(weights))
        return self._model

    def calibrate(self, frame, previous=None):
        """The plate found in frame: near previous (a rectify.Calibration) if given, else from
        scratch. Raises rectify.PlateNotFound."""
        cal, _ = rectify.recalibrate(frame, previous) if previous is not None else rectify.calibrate(frame)
        return cal

    def detect(self, frame, cal, snapshot_id):
        """(Detections, rectified image) for frame; the image is white-balanced, for overlays."""
        image, balanced = rectify.model_input(frame, cal)
        frame_tiles = tiling.tiles(self.work_area, cal.settings.size, self.tile_size, self.tile_overlap)
        polygons, scores = tiling.predict_frame(self.model, image, frame_tiles, conf=self.vision.conf)
        return to_detections(polygons, scores, cal, self.rig, self.workspace, snapshot_id), balanced


def camera_mm(cal):
    """The lens in scene mm: where the plate fit put it, or the tower's design position."""
    if cal.fit and "camera_mm" in cal.fit:
        return tuple(cal.fit["camera_mm"])
    return cal.settings.camera_mm


def to_detections(polygons, scores, cal, rig, workspace, snapshot_id):
    """Detections from outlines in rectified px."""
    st = cal.settings
    lens = camera_mm(cal)
    out = []
    for poly, score in zip(polygons, scores):
        poly = np.asarray(poly, float).reshape(-1, 2)
        scene = rectify.rect_to_mm(poly, st)
        # A lying body's axis is half its width above the plate.
        width_mm = min(cv2.minAreaRect(scene.astype(np.float32))[1])
        scene = correct_height(scene, lens, st.top_z_mm, width_mm / 2)
        out.append(Detection.from_outlines(snapshot_id, rectify.to_full(poly, cal), scene_to_robot(scene, rig),
                                           workspace.packages, workspace.size_tolerance, score))
    return out
