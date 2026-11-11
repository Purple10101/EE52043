"""RunWindow: what the sorting program shows, and the buttons that drive it.

    +----------------------------------------+------------------+
    | what it is doing now                   |  TRAYS           |
    | the arm: when it was started           |  A1  1k5    3 in |
    | the plate: what the last look found    |  ...  B4 empty   |
    +----------------------------------------+------------------+
    |  the plate from above at the last look |  WHERE THE BINS  |
    |  each resistor outlined and labelled   |  ARE: the rig    |
    |                                        |  from above      |
    +----------------------------------------+------------------+
    |  CALIBRATE ARM  |  CHECK AIM  |  LOOK  |  START  |  STOP   |
    +-----------------------------------------------------------+

Outlines: green the next pick, orange picked later, grey left on the plate. A tray with an orange
border is the one the arm is taking a resistor to. On the map the arm moves as the real one does
(hardware.ArmMonitor), redrawn every time the window is.

The run works in another thread and calls show, show_trays and say whenever it likes; they only store
what to draw. The program's own thread calls poll() over and over: it draws the window and returns the
button or key pressed. So the window stays alive while the arm moves.
"""

import textwrap
import threading

import cv2
import numpy as np
from robot_offline.post.bands.learned import ohms_text
from robot_offline.pre import rectify

TITLE = "resistor sorting"
PLATE_PX = 760                  # the plate view's height and width
TRAY_W = 440
STATUS_H = 172
STATUS_CHARS = 62             # characters a status line holds
BUTTON_H = 80
BUTTONS = ["CALIBRATE ARM", "CHECK AIM", "LOOK", "START", "STOP"]
NEXT, WILL_PICK, STAYS = (0, 230, 0), (0, 160, 255), (150, 150, 150)     # BGR
FONT = cv2.FONT_HERSHEY_SIMPLEX


class RunWindow:
    def __init__(self, title=TITLE, rig_map=None, monitor=None):
        """rig_map: task.RigMap, drawn under the tray list to show where each bin is; None for no map.
        monitor: hardware.ArmMonitor, to draw the arm on the map as it moves; None to leave it out."""
        self.title = title
        self.rig_map = rig_map
        self.monitor = monitor
        self._lock = threading.Lock()
        self._plate = None              # the last look, drawn, PLATE_PX square
        self._plate_lines = []          # what the last look found
        self._trays, self._target = [], None
        self._status, self._arm_line, self._aim_line = "", "", ""
        self._click = None
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.moveWindow(title, 40, 20)   # else it can open off the screen
        cv2.setMouseCallback(title, self._on_mouse)
        self._size = None

    # Called by the run, from any thread.

    def show(self, view, plan, lines):
        """A look and its plan. Returns the plate image, for the log."""
        image = draw_plate(view, plan)
        with self._lock:
            self._plate, self._plate_lines = image, list(lines)
        return image

    def show_trays(self, trays, target=None):
        """What each bin holds (planning.Tray), and the bin the arm is heading for, or None."""
        with self._lock:
            self._trays, self._target = list(trays), target

    def say(self, text):
        """What is happening now."""
        with self._lock:
            self._status = text

    def show_aim(self, text):
        """The arm-to-plate calibration: when it was fitted, when its aim was last checked."""
        with self._lock:
            self._aim_line = text

    def show_arm(self, text):
        """The arm's state: when it was started, how its check went."""
        with self._lock:
            self._arm_line = text

    # Called by the program's own thread.

    def poll(self, enabled):
        """Draws the window and waits a moment for input. enabled: the buttons that may be pressed now.
        Returns the name of a button clicked, "QUIT" for q, Esc or the window closed, or None."""
        with self._lock:
            image = self._compose(enabled)
        size = image.shape[1], image.shape[0]
        if self._size != size:
            cv2.resizeWindow(self.title, *size)
            self._size = size
        cv2.imshow(self.title, image)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) < 1:
            return "QUIT"
        click, self._click = self._click, None
        if click is None or click[1] < STATUS_H + PLATE_PX:
            return None
        name = BUTTONS[min(click[0] * len(BUTTONS) // size[0], len(BUTTONS) - 1)]
        return name if name in enabled else None

    def close(self):
        cv2.destroyWindow(self.title)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._click = (x, y)

    def _compose(self, enabled):
        width = PLATE_PX + TRAY_W
        status = np.full((STATUS_H, PLATE_PX, 3), 25, np.uint8)
        now = textwrap.wrap(self._status, STATUS_CHARS)[:3]       # what is happening: up to three lines, bold
        rest = [self._arm_line, self._aim_line, self._plate_lines[0] if self._plate_lines else ""]
        for i, text in enumerate(now):
            cv2.putText(status, fit(text, STATUS_CHARS), (12, 28 + 26 * i), FONT, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        for i, text in enumerate(rest):
            cv2.putText(status, fit(text, STATUS_CHARS + 8), (12, 108 + 24 * i), FONT, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        plate = self._plate if self._plate is not None else empty_plate()
        left = np.vstack([status, plate])
        height = STATUS_H + PLATE_PX
        if self.rig_map is not None:
            pose = self.monitor.pose() if self.monitor is not None else None
            where = self.rig_map.draw(self._trays, self._target, TRAY_W, pose)[:height // 2 + 40]
            right = np.vstack([draw_trays(self._trays, self._target, height - where.shape[0]), where])
        else:
            right = draw_trays(self._trays, self._target, height)
        return np.vstack([np.hstack([left, right]), draw_buttons(width, enabled)])


def draw_plate(view, plan):
    """The plate from above, white-balanced, every resistor outlined and labelled with its fate. Detections
    too faint to be a new body get a thin grey outline and "?"; a body seen before but missing from this
    look gets a dashed orange ring, "missed?" (the run looks again before it ends)."""
    rect, _ = rectify.white_balance(rectify.rectify(view.frame, view.cal), view.cal.settings)
    for poly, score, _ in getattr(view, "faint", []):
        outline = np.round(poly).astype(np.int32)
        cv2.polylines(rect, [outline], True, STAYS, 1, cv2.LINE_AA)
        cv2.putText(rect, f"? {score:.2f}", (int(outline[:, 0].max()) + 6, int(outline[:, 1].mean())), FONT, 0.7,
                    STAYS, 2, cv2.LINE_AA)
    for centre in getattr(view, "missing", []):
        (cx, cy), = rectify.mm_to_rect(np.asarray([centre], float), view.cal.settings)
        for a in range(0, 360, 30):                     # a dashed ring
            cv2.ellipse(rect, (int(cx), int(cy)), (60, 60), 0, a, a + 18, WILL_PICK, 4, cv2.LINE_AA)
        cv2.putText(rect, "missed?", (int(cx) + 66, int(cy)), FONT, 0.9, WILL_PICK, 2, cv2.LINE_AA)
    for r in view.resistors:
        fate = plan.fates.get(r.number, "")
        colour = NEXT if fate.startswith("NEXT") else STAYS if fate.startswith("stays") else WILL_PICK
        outline = np.round(r.outline_px).astype(np.int32)
        cv2.polylines(rect, [outline], True, colour, 6 if colour is NEXT else 3, cv2.LINE_AA)
        x, y = int(outline[:, 0].max()) + 8, int(outline[:, 1].mean())
        cv2.putText(rect, f"#{r.number} {fate}", (x, y), FONT, 0.9, colour, 2, cv2.LINE_AA)
    return cv2.resize(rect, (PLATE_PX, PLATE_PX), interpolation=cv2.INTER_AREA)


def empty_plate():
    plate = np.full((PLATE_PX, PLATE_PX, 3), 40, np.uint8)
    cv2.putText(plate, "No look yet: press LOOK or START", (90, PLATE_PX // 2), FONT, 0.9, (200, 200, 200), 2, cv2.LINE_AA)
    return plate


def draw_trays(trays, target, height):
    """The bins, in two columns as their ids run (A1-A4, B1-B4): each one's value and how many it holds."""
    panel = np.full((height, TRAY_W, 3), 30, np.uint8)
    cv2.putText(panel, "TRAYS", (16, 40), FONT, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(panel, "each bin's value, and how many are in", (130, 40), FONT, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
    if not trays:
        return panel
    columns = sorted({t.bin_id[0] for t in trays})
    rows = max(sum(t.bin_id[0] == c for t in trays) for c in columns)
    box_w, box_h, top = (TRAY_W - 12) // len(columns), (height - 64) // rows, 60
    for t in trays:
        col, row = columns.index(t.bin_id[0]), [u.bin_id for u in trays if u.bin_id[0] == t.bin_id[0]].index(t.bin_id)
        x0, y0 = 6 + col * box_w, top + row * box_h
        x1, y1 = x0 + box_w - 6, y0 + box_h - 6
        has = t.count > 0 or t.value_ohm is not None
        fill = (40, 70, 40) if has else (45, 45, 45)
        cv2.rectangle(panel, (x0, y0), (x1, y1), fill, -1)
        if t.bin_id == target:
            cv2.rectangle(panel, (x0, y0), (x1, y1), WILL_PICK, 5)
        cv2.putText(panel, t.bin_id, (x0 + 10, y0 + 30), FONT, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        main = ohms_text(t.value_ohm) if t.value_ohm is not None else "empty"
        cv2.putText(panel, main, (x0 + 10, y0 + box_h // 2 + 12), FONT, 1.2,
                    (255, 255, 255) if has else (130, 130, 130), 2, cv2.LINE_AA)
        count = f"{t.count} in"
        (count_w, _), _ = cv2.getTextSize(count, FONT, 0.8, 2)
        cv2.putText(panel, count, (x1 - 12 - count_w, y0 + 30), FONT, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return panel


def draw_buttons(width, enabled):
    bar = np.full((BUTTON_H, width, 3), 20, np.uint8)
    for i, name in enumerate(BUTTONS):
        x0, x1 = i * width // len(BUTTONS) + 6, (i + 1) * width // len(BUTTONS) - 6
        live = name in enabled
        colour = ((40, 40, 170) if name == "STOP" else (60, 150, 60)) if live else (60, 60, 60)
        cv2.rectangle(bar, (x0, 10), (x1, BUTTON_H - 10), colour, -1)
        (text_w, _), _ = cv2.getTextSize(name, FONT, 0.8, 2)
        cv2.putText(bar, name, (x0 + (x1 - x0 - text_w) // 2, BUTTON_H // 2 + 9), FONT, 0.8,
                    (255, 255, 255) if live else (140, 140, 140), 2, cv2.LINE_AA)
    return bar


def fit(text, chars):
    return text if len(text) <= chars else text[:chars - 3] + "..."
