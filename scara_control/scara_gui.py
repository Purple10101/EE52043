import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

from scara_controller import ScaraController, SocketTransport
from scara_workspace import WorkspaceLimits, WorkspaceLimitError

# ---------------------------------------------------------------- config ----
COM_PORT = "COM3"
BAUDRATE = 115200
SIM_HOST, SIM_PORT = "127.0.0.1", 9000

LINK_1, LINK_2 = 0.125, 0.1  # metres

# Z travel (metres) differs between backends. Adjust the sim values to match
# your Unity model.
PROFILES = {
    "Simulator": dict(z_min=0.025, z_max=0.075),
    "Hardware": dict(z_min=0.095, z_max=0.15),
}

MIN_SEND_INTERVAL = 0.05  # s, floor between consecutive live commands

# Working limits (joint ranges + reach) come from scara_workspace.py.
# The controller enforces them on every command; the sliders just reflect them.
LIMITS = WorkspaceLimits(l1=LINK_1, l2=LINK_2)
R_MIN, R_MAX = LIMITS.reach_range()


class ScaraLiveGuiApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Live Joint Control")
        self.root.geometry("580x700")
        self.root.resizable(False, False)

        self.scara = None
        self.connected = False

        # State variables
        self.backend_var = tk.StringVar(value="Simulator")
        self.j0_val = tk.DoubleVar(value=0.0)  # base, deg
        self.j1_val = tk.DoubleVar(value=0.0)  # elbow, deg
        self.j2_val = tk.DoubleVar(value=0.0)  # wrist, deg
        self.j3_val = tk.DoubleVar(value=0.05)  # Z height, metres
        self.move_time_val = tk.DoubleVar(value=0.5)

        self.live_mode_val = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="Status: Not connected")
        self.feedback_var = tk.StringVar(value="X: -- | Y: -- | Z_Angle: -- | Z: --")
        self.pred_var = tk.StringVar(value="")

        # "Latest target wins" sender state
        self._state_lock = threading.Lock()
        self._pending = None
        self._busy = False

        self._build_ui()
        self._apply_z_range(PROFILES[self.backend_var.get()])
        self._update_prediction()

    # ------------------------------------------------------------------ UI --
    def _build_ui(self):
        top = ttk.LabelFrame(self.root, text=" Connection & Settings ", padding=10)
        top.pack(fill="x", padx=15, pady=10)

        row1 = ttk.Frame(top)
        row1.pack(fill="x")
        ttk.Label(row1, text="Backend:").pack(side="left")
        self.backend_box = ttk.Combobox(
            row1, textvariable=self.backend_var, values=list(PROFILES),
            state="readonly", width=12,
        )
        self.backend_box.pack(side="left", padx=6)
        self.connect_btn = ttk.Button(row1, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=6)
        ttk.Checkbutton(row1, text="Live Slider Mode", variable=self.live_mode_val).pack(side="right")

        ttk.Label(top, textvariable=self.status_var, font=("Helvetica", 10, "bold")).pack(
            anchor="w", pady=(8, 0)
        )

        sliders = ttk.LabelFrame(self.root, text=" Joint Controls ", padding=15)
        sliders.pack(fill="both", expand=True, padx=15, pady=5)

        joints = [
            ("Joint 0 (Base °)", self.j0_val, *LIMITS.j0, 1.0, "0"),
            ("Joint 1 (Elbow °)", self.j1_val, *LIMITS.j1, 1.0, "1"),
            ("Joint 2 (Wrist °)", self.j2_val, *LIMITS.j2, 1.0, "2"),
            ("Joint 3 (Z Height m)", self.j3_val, 0.025, 0.075, 0.005, "3"),
        ]
        for label, var, lo, hi, inc, key in joints:
            row = ttk.Frame(sliders)
            row.pack(fill="x", pady=8)
            ttk.Label(row, text=label, width=22, anchor="w").pack(side="left")

            scale = ttk.Scale(row, from_=lo, to=hi, variable=var, orient="horizontal",
                              command=self._on_slider_move)
            scale.pack(side="left", fill="x", expand=True, padx=5)

            spin = ttk.Spinbox(row, from_=lo, to=hi, increment=inc, textvariable=var,
                               width=7, command=self._on_spinbox_change)
            spin.pack(side="right")
            spin.bind("<Return>", lambda e: self._on_spinbox_change())
            spin.bind("<FocusOut>", lambda e: self._on_spinbox_change())

            if key == "3":
                self.z_scale, self.z_spin = scale, spin

        settings = ttk.Frame(sliders)
        settings.pack(fill="x", pady=12)
        ttk.Label(settings, text="Trajectory Time (s):").pack(side="left")
        ttk.Spinbox(settings, from_=0.1, to=10.0, increment=0.1,
                    textvariable=self.move_time_val, width=6).pack(side="left", padx=10)

        ttk.Label(
            sliders,
            text=(f"Limits: base {LIMITS.j0[0]:.0f}..{LIMITS.j0[1]:.0f}°, "
                  f"elbow {LIMITS.j1[0]:.0f}..{LIMITS.j1[1]:.0f}°, "
                  f"reach {R_MIN:.3f}..{R_MAX:.3f} m"),
            foreground="gray",
        ).pack(anchor="w")

        btns = ttk.Frame(self.root, padding=10)
        btns.pack(fill="x", padx=15)
        ttk.Button(btns, text="Send Current Values", command=self.request_move).pack(
            side="left", fill="x", expand=True, padx=5)
        ttk.Button(btns, text="Zero All Joints", command=self.zero_joints).pack(
            side="left", fill="x", expand=True, padx=5)
        ttk.Button(btns, text="Read Feedback", command=self.read_feedback).pack(
            side="left", fill="x", expand=True, padx=5)

        fb = ttk.LabelFrame(self.root, text=" Live Feedback ", padding=10)
        fb.pack(fill="x", padx=15, pady=10)
        ttk.Label(fb, textvariable=self.pred_var, font=("Consolas", 10)).pack()
        ttk.Label(fb, textvariable=self.feedback_var, font=("Consolas", 10)).pack()

    def _apply_z_range(self, profile):
        z_min, z_max = profile["z_min"], profile["z_max"]
        self.z_scale.configure(from_=z_min, to=z_max)
        self.z_spin.configure(from_=z_min, to=z_max)
        self.j3_val.set(round((z_min + z_max) / 2, 4))

    def _set_status(self, text):
        self.status_var.set(text)

    # ----------------------------------------------------------- connection --
    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        backend = self.backend_var.get()
        self._set_status(f"Status: Connecting to {backend}...")
        self.connect_btn.state(["disabled"])
        threading.Thread(target=self._connect_worker, args=(backend,), daemon=True).start()

    def _connect_worker(self, backend):
        # Runs off the main thread; touch the UI only via root.after.
        try:
            prof = PROFILES[backend]
            limits = WorkspaceLimits(l1=LINK_1, l2=LINK_2, z=(prof["z_min"], prof["z_max"]))
            if backend == "Simulator":
                scara = ScaraController(transport=SocketTransport(SIM_HOST, SIM_PORT),
                                        limits=limits)
            else:
                scara = ScaraController(port=COM_PORT, baudrate=BAUDRATE, limits=limits)
            ok = scara.initialize_robot(link1=LINK_1, link2=LINK_2,
                                        z_min=prof["z_min"], z_max=prof["z_max"])
        except Exception as e:
            self.root.after(0, self._on_connect_failed, backend, e)
            return
        self.root.after(0, self._on_connected, scara, backend, ok)

    def _on_connected(self, scara, backend, init_ok):
        self.scara = scara
        self.connected = True
        self._apply_z_range(PROFILES[backend])
        self.backend_box.configure(state="disabled")
        self.connect_btn.configure(text="Disconnect")
        self.connect_btn.state(["!disabled"])
        if init_ok:
            self._set_status(f"Status: Connected ({backend}), initialised")
        else:
            self._set_status(f"Status: Connected ({backend}), INIT NOT ACKNOWLEDGED")

    def _on_connect_failed(self, backend, err):
        self.connect_btn.state(["!disabled"])
        self._set_status("Status: Not connected")
        messagebox.showwarning("Connection Warning", f"Could not connect to {backend}:\n{err}")

    def disconnect(self):
        scara, self.scara, self.connected = self.scara, None, False
        with self._state_lock:
            self._pending = None
        if scara:
            try:
                scara.close()
            except Exception:
                pass
        self.backend_box.configure(state="readonly")
        self.connect_btn.configure(text="Connect")
        self._set_status("Status: Not connected")

    def _on_link_error(self, err):
        if self.connected:
            self.disconnect()
        self._set_status(f"Status: Link error ({err})")

    # -------------------------------------------------------------- sending --
    def _update_prediction(self):
        """Show where the tip will be for the current base/elbow angles."""
        try:
            x, y = LIMITS.fk(self.j0_val.get(), self.j1_val.get())
        except tk.TclError:
            return
        self.pred_var.set(f"Predicted tip: X {x:.3f} m | Y {y:.3f} m")

    def _on_slider_move(self, _value=None):
        self._update_prediction()
        if self.live_mode_val.get():
            self.request_move()

    def _on_spinbox_change(self):
        self._update_prediction()
        if self.live_mode_val.get():
            self.request_move()

    def request_move(self):
        """Record the latest target; a single worker sends it (never overlaps)."""
        if not self.connected:
            return
        try:  # read Tk variables on the main thread only
            target = (self.j0_val.get(), self.j1_val.get(), self.j2_val.get(),
                      self.j3_val.get(), self.move_time_val.get())
        except tk.TclError:
            return  # a spinbox is mid-edit with invalid text

        with self._state_lock:
            self._pending = target
            if self._busy:
                return  # worker will pick up the newest target
            self._busy = True
        threading.Thread(target=self._send_loop, daemon=True).start()

    def _send_loop(self):
        while True:
            with self._state_lock:
                target = self._pending
                self._pending = None
                if target is None:
                    self._busy = False
                    return
            scara = self.scara
            if scara is None:
                with self._state_lock:
                    self._busy = False
                return
            try:
                ok = scara.move_joints(*target)
            except WorkspaceLimitError as e:
                self.root.after(0, self._set_status, f"Rejected: {e}")
                time.sleep(MIN_SEND_INTERVAL)
                continue
            except Exception as e:
                with self._state_lock:
                    self._busy = False
                    self._pending = None
                self.root.after(0, self._on_link_error, e)
                return

            j0, j1, j2, z, _t = target
            if ok:
                msg = f"Target: [{j0:.1f}°, {j1:.1f}°, {j2:.1f}°, Z {z:.3f} m]"
            else:
                msg = "Status: No ack (timeout or rejected)"
            self.root.after(0, self._set_status, msg)
            time.sleep(MIN_SEND_INTERVAL)

    def zero_joints(self):
        self.j0_val.set(0.0)
        self.j1_val.set(0.0)
        self.j2_val.set(0.0)
        z_min, z_max = (float(self.z_scale.cget("from")), float(self.z_scale.cget("to")))
        self.j3_val.set(round((z_min + z_max) / 2, 4))
        self._update_prediction()
        self.request_move()

    # ------------------------------------------------------------- feedback --
    def read_feedback(self):
        if not self.connected:
            return
        scara = self.scara

        def worker():
            try:
                pos = scara.read_coordinates()
            except Exception as e:
                self.root.after(0, self._on_link_error, e)
                return
            if pos:
                text = (f"X: {pos[0]:.3f}m | Y: {pos[1]:.3f}m | "
                        f"Z_Angle: {pos[2]:.1f}° | Z: {pos[3]:.3f}m")
            else:
                text = "No feedback (timeout, or backend lacks read support)"
            self.root.after(0, self.feedback_var.set, text)

        threading.Thread(target=worker, daemon=True).start()

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ScaraLiveGuiApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()