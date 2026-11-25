"""SharedCamera: one camera read from two threads, a job's and a window's live view.

    camera = SharedCamera(cap)      # takes cv2.VideoCapture's read, set, get and release, and passes them on
    camera.read()                   # as cap.read(), one thread at a time; the frame is kept as the newest
    frame = camera.newest()         # for a live view: the newest frame, read here only if nothing else reads

A live view mustn't slow a job that is reading (a look's average, a focus sweep): every frame read for the
view is one the job waits for. So newest() reads a frame itself only when no other thread has read one in the
last IDLE_S; while a job reads, the view shows the job's own frames. It never sets anything (focus, exposure).
"""

import threading
import time

IDLE_S = 0.4            # the camera is free for a live view when no other thread has read it for this long (~4 frames)


class SharedCamera:
    def __init__(self, cap):
        self.cap = cap
        self._lock = threading.Lock()
        self.frame = None           # the newest frame anyone read
        self.read_at = 0.0          # time.monotonic() of that read
        self.read_by = None         # the thread that read it

    def read(self):
        with self._lock:
            ok, frame = self.cap.read()
        if ok and frame is not None:
            self.frame, self.read_at, self.read_by = frame, time.monotonic(), threading.get_ident()
        return ok, frame

    def newest(self, idle_s=IDLE_S):
        """The newest frame (None before the first): read now unless another thread read one in the last idle_s."""
        if self.read_by == threading.get_ident() or time.monotonic() - self.read_at > idle_s:
            self.read()
        return self.frame

    def __getattr__(self, name):        # set, get, release, isOpened: the camera's own
        return getattr(self.cap, name)
