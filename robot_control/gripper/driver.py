"""The gripper's board over USB serial (gripper_firmware/gripper_v4).

The board keeps the jaws between its saved ends, so nothing here can send them past open or closed. It
boots limp and stays limp until the first move; after a restart that first move starts from the middle
of the jaws' travel, at full speed. Disconnecting leaves the jaws as they are, still holding if they were:
release() is what lets go.
"""

import time

import serial
from serial.tools import list_ports

ESPRESSIF_VID = 0x303A
BAUD = 115200
REPLY_S = 1.0               # a reply takes milliseconds
MOVE_S = 3.0                # the whole stroke takes under a second
POLL_S = 0.05


class GripperError(Exception):
    pass


def find_port():
    """The gripper board's serial port. Raises GripperError unless there is exactly one."""
    found = [p.device for p in list_ports.comports() if p.vid == ESPRESSIF_VID]
    if len(found) != 1:
        raise GripperError(f"expected one gripper board on USB, found {found or 'none'}")
    return found[0]


def open_link(port):
    """The serial link to port, without touching DTR and RTS, which reset the board."""
    link = serial.Serial()
    link.port, link.baudrate, link.timeout = port, BAUD, 0.1
    link.dtr = link.rts = False
    link.open()
    return link


class Gripper:
    """link: an open serial link, or None to find the board. Raises GripperError if it isn't calibrated."""

    def __init__(self, link=None):
        self.link = link if link is not None else open_link(find_port())
        if not self.status()["cal"]:
            raise GripperError("the board isn't calibrated: its jaws have no saved ends")

    def _ask(self, command, wanted="OK"):
        """Sends a command and returns the reply starting with wanted. An ERR reply raises GripperError."""
        self.link.reset_input_buffer()
        self.link.write((command + "\n").encode())
        self.link.flush()
        end = time.monotonic() + REPLY_S
        while time.monotonic() < end:
            reply = self.link.readline().decode(errors="replace").strip()
            if reply.startswith("ERR"):
                raise GripperError(f"{command}: {reply}")
            if reply.startswith(wanted):
                return reply
        raise GripperError(f"{command}: no reply from the board")

    def status(self):
        """The board's status line as a dict of ints: us, tgt, lo, hi, cal, closed, open, att and so on."""
        pairs = (field.split("=") for field in self._ask("?", "S ").split()[1:])
        return {name: int(float(value)) for name, value in pairs}

    def _go(self, command):
        if not self.status()["att"]:
            self._ask("A")
        self._ask(command)
        end = time.monotonic() + MOVE_S
        while time.monotonic() < end:
            state = self.status()
            if state["us"] == state["tgt"]:
                return
            time.sleep(POLL_S)
        raise GripperError(f"{command}: the jaws didn't finish moving")

    def open(self):
        """Opens the jaws fully and returns once there."""
        self._go("O")

    def close(self):
        """Closes the jaws to their saved closed end and returns once there. On a part, they press on it for
        as long as they are held."""
        self._go("C")

    def release(self):
        """Stops the pulses, so the servo goes limp."""
        self._ask("R")

    def disconnect(self):
        """Closes the serial link. The board carries on as it is."""
        self.link.close()
