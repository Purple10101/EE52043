"""Camera (live/replay), Detector and the mapping to the robot frame.

The pipeline from a raw frame to plate outlines (rectification, the segmentation model, tiling) is
the robot_offline package, installed with `pip install -e`; this module wraps it and turns its
outlines into Detections in the robot frame.

    vision.rig        scene mm to robot m, body-height correction (pure maths)
    vision.camera     LiveCamera, ReplayCamera
    vision.detector   Detector: frame to Detections
"""
