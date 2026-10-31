"""Frame sources: the overhead camera, or saved frames for working without the rig.

Both give raw 4608 × 2592 BGR frames, as the camera takes them, named for the snapshot id.
"""

import time
from pathlib import Path

import cv2

STALE_FRAMES = 5            # frames DirectShow may have buffered from before the arm parked


class LiveCamera:
    """The tower camera, opened with robot_offline's configs/camera.toml."""

    def __init__(self):
        from robot_offline.pre.camera import open_camera
        self._cap = open_camera()

    def grab(self):
        """(id, frame): a frame taken now, not one left in the buffer."""
        for _ in range(STALE_FRAMES):
            self._cap.grab()
        ok, frame = self._cap.read()
        if not ok:
            raise RuntimeError("the camera returned no frame")
        return time.strftime("%Y%m%d-%H%M%S"), frame

    def close(self):
        self._cap.release()


class ReplayCamera:
    """Saved raw frames: one image, or every .png / .jpg in a folder, in name order."""

    def __init__(self, path):
        path = Path(path)
        self.paths = sorted(p for p in path.iterdir() if p.suffix.lower() in (".png", ".jpg")) if path.is_dir() else [path]
        if not self.paths:
            raise FileNotFoundError(f"no .png or .jpg frames in {path}")

    def frames(self):
        """(id, frame) for each saved frame."""
        for p in self.paths:
            frame = cv2.imread(str(p))
            if frame is None:
                raise OSError(f"cannot read {p}")
            yield p.stem, frame

    def close(self):
        pass
