"""
scara_controller.py

PC-side client for the 48-byte SCARA binary protocol (see application.c).
The same protocol is spoken by the real STM32 firmware (over a serial port)
and by the Unity simulator (over TCP), so ScaraController works with either:

    ScaraController(port="COM3")                                  # hardware
    ScaraController(transport=SocketTransport("127.0.0.1", 9000)) # simulator

Frame (48 bytes, little-endian):  int32 command_id + 11 x (float32 | int32)
Reply: 4-byte int32 success flag, optionally followed by a payload.
"""

import socket
import struct
import threading
import time
from dataclasses import replace

from scara_workspace import WorkspaceLimits, WorkspaceLimitError  # noqa: F401


class SocketTransport:
    """Provides the subset of serial.Serial that ScaraController uses, over TCP."""

    def __init__(self, host="127.0.0.1", port=9000, timeout=5.0):
        self._timeout = timeout
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        self._timeout = value
        self.sock.settimeout(value)

    def write(self, data: bytes):
        self.sock.sendall(data)

    def flush(self):
        pass

    def read(self, n: int) -> bytes:
        """Read exactly n bytes, or fewer if the timeout/connection ends first."""
        buf = b""
        try:
            while len(buf) < n:  # TCP may deliver partial reads
                chunk = self.sock.recv(n - len(buf))
                if not chunk:  # connection closed by peer
                    break
                buf += chunk
        except socket.timeout:
            pass
        return buf

    def reset_input_buffer(self):
        """Discard any stale bytes (e.g. a late ack from a timed-out command)."""
        self.sock.setblocking(False)
        try:
            while self.sock.recv(4096):
                pass
        except OSError:  # BlockingIOError: nothing left to read
            pass
        finally:
            self.sock.settimeout(self._timeout)

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


class ScaraController:
    # Command IDs matching application.c switch-case
    CMD_INIT = 1
    CMD_CALIBRATE = 2
    CMD_MOVE_JOINT = 3
    CMD_MOVE_JOINTS = 4
    CMD_MOVE_COORD = 5
    CMD_READ_ANGLE = 6
    CMD_READ_COORD = 7

    FRAME_SIZE = 48
    CALIBRATE_TIMEOUT = 30.0  # auto-calibration may take a while

    def __init__(self, port=None, baudrate=115200, timeout=2.0, transport=None,
                 limits=None, enforce_limits=True):
        """
        Pass either a serial `port` (hardware) or a ready-made `transport`
        (e.g. SocketTransport for the simulator).

        Working limits (scara_workspace.WorkspaceLimits) are checked BEFORE a
        command is sent; violations raise WorkspaceLimitError and nothing goes
        out on the wire. Pass `limits=` to override the defaults (e.g. the sim's
        Z range), or `enforce_limits=False` to disable the checks entirely.
        """
        self._lock = threading.Lock()  # one command/response at a time
        self.limits = (limits or WorkspaceLimits()) if enforce_limits else None

        if transport is not None:
            self.ser = transport
        else:
            if port is None:
                raise ValueError("Provide either a serial port or a transport")
            import serial  # imported here so sim-only use doesn't need pyserial

            self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            time.sleep(1.0)  # board may reset when the port opens

        self.default_timeout = self.ser.timeout or timeout

    # ------------------------------------------------------------------ #
    # Low level
    # ------------------------------------------------------------------ #
    @classmethod
    def _pack(cls, cmd_id, floats=(), axis=None) -> bytes:
        """
        Build one 48-byte frame.
        `axis` (int) goes in values[0] as a real int32, as the firmware reads it
        with memcpy into an int32_t. `floats` fill the remaining slots.
        """
        if axis is None:
            n_slots, head = 11, (cmd_id,)
            fmt = "<i11f"
        else:
            n_slots, head = 10, (cmd_id, int(axis))
            fmt = "<ii10f"

        floats = list(floats)
        if len(floats) > n_slots:
            raise ValueError(f"Too many values: {len(floats)} > {n_slots}")
        floats += [0.0] * (n_slots - len(floats))

        frame = struct.pack(fmt, *head, *floats)
        if len(frame) != cls.FRAME_SIZE:
            raise ValueError(f"Frame error: expected 48 bytes, got {len(frame)}")
        return frame

    def _transact(self, frame: bytes, reply_len: int, timeout=None) -> bytes:
        """Send one frame and read the reply atomically."""
        with self._lock:
            self.ser.timeout = timeout if timeout is not None else self.default_timeout
            self.ser.reset_input_buffer()  # drop stale bytes from earlier timeouts
            self.ser.write(frame)
            self.ser.flush()
            return self.ser.read(reply_len)

    @staticmethod
    def _ack(resp: bytes) -> bool:
        return len(resp) >= 4 and struct.unpack("<i", resp[:4])[0] == 1

    @staticmethod
    def _check_time(move_time: float):
        if not move_time > 0:
            raise ValueError(f"Trajectory time must be > 0 s, got {move_time}")

    def _move_timeout(self, move_time: float) -> float:
        # The ack may only arrive once the move has finished.
        return max(self.default_timeout, float(move_time) + 5.0)

    # ------------------------------------------------------------------ #
    # Commands
    # ------------------------------------------------------------------ #
    def initialize_robot(self, link1=0.125, link2=0.1, z_min=0.095, z_max=0.15,
                         settling_time=1.0, P0=1.3, I0=0.01, D0=0.001,
                         P1=1.3, I1=0.01, D1=0.001) -> bool:
        """Command 1: SCARA_INITIALISE"""
        values = [link1, link2, z_min, z_max, settling_time, P0, I0, D0, P1, I1, D1]
        ok = self._ack(self._transact(self._pack(self.CMD_INIT, values), 4))
        if ok and self.limits is not None:
            # single source of truth: limits follow the geometry the robot was given
            self.limits = replace(self.limits, l1=link1, l2=link2, z=(z_min, z_max))
        return ok

    def auto_calibrate(self) -> bool:
        """Command 2: SCARA_AUTO_CALIBRATE"""
        frame = self._pack(self.CMD_CALIBRATE)
        return self._ack(self._transact(frame, 4, timeout=self.CALIBRATE_TIMEOUT))

    def move_single_joint(self, axis: int, angle: float, move_time: float) -> bool:
        """Command 3: SCARA_MOVE_JOINT (axis 0-2 = angle, 3 = Z height in metres)"""
        if self.limits is not None:
            self.limits.check_axis(axis, angle)
        self._check_time(move_time)
        frame = self._pack(self.CMD_MOVE_JOINT, [angle, move_time], axis=axis)
        return self._ack(self._transact(frame, 4, self._move_timeout(move_time)))

    def move_joints(self, angle0, angle1, angle2, z, move_time) -> bool:
        """Command 4: SCARA_MOVE_JOINTS  (base deg, elbow deg, wrist deg, Z metres, time s)"""
        if self.limits is not None:
            self.limits.check_joints(angle0, angle1, angle2, z)
        self._check_time(move_time)
        frame = self._pack(self.CMD_MOVE_JOINTS, [angle0, angle1, angle2, z, move_time])
        return self._ack(self._transact(frame, 4, self._move_timeout(move_time)))

    def move_coordinates(self, x, y, z_angle, z, move_time) -> bool:
        """Command 5: SCARA_MOVE_COORD (x m, y m, wrist deg, z m, time s)"""
        if self.limits is not None:
            self.limits.check_coord(x, y, z_angle, z)
        self._check_time(move_time)
        frame = self._pack(self.CMD_MOVE_COORD, [x, y, z_angle, z, move_time])
        return self._ack(self._transact(frame, 4, self._move_timeout(move_time)))

    def read_joint_angle(self, axis: int):
        """Command 6: SCARA_READ_ANGLE -> float, or None on failure."""
        resp = self._transact(self._pack(self.CMD_READ_ANGLE, axis=axis), 8)
        if len(resp) == 8 and self._ack(resp[:4]):
            return struct.unpack("<f", resp[4:])[0]
        return None

    def read_coordinates(self):
        """Command 7: SCARA_READ_COORD -> (x, y, z_angle, z), or None on failure."""
        resp = self._transact(self._pack(self.CMD_READ_COORD), 20)
        if len(resp) == 20 and self._ack(resp[:4]):
            return struct.unpack("<4f", resp[4:])
        return None

    def close(self):
        self.ser.close()


if __name__ == "__main__":
    import sys

    target = sys.argv[1] if len(sys.argv) > 1 else "sim"
    if target == "sim":
        z_lo, z_hi = 0.025, 0.075
        scara = ScaraController(transport=SocketTransport("127.0.0.1", 9000),
                                limits=WorkspaceLimits(z=(z_lo, z_hi)))
    else:
        z_lo, z_hi = 0.095, 0.15
        scara = ScaraController(port=target)  # e.g. COM3

    try:
        print("init:", scara.initialize_robot(z_min=z_lo, z_max=z_hi))
        z_mid = (z_lo + z_hi) / 2
        print("move_joints:", scara.move_joints(45.0, 30.0, 0.0, z_mid, 1.5))
        x, y = scara.limits.fk(20.0, 40.0)  # a point known to be inside the workspace
        print(f"move_coords to ({x:.3f}, {y:.3f}):",
              scara.move_coordinates(x, y, 0.0, z_mid, 1.0))
        print("position:", scara.read_coordinates())
    finally:
        scara.close()