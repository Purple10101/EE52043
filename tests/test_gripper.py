import pytest

from robot_control.gripper import driver
from robot_control.gripper.driver import Gripper, GripperError, find_port


class FakeBoard:
    """Answers like the firmware. A move takes `ticks` status reads to finish."""

    def __init__(self, cal=1, att=0, ticks=0):
        self.cal, self.att, self.ticks = cal, att, ticks
        self.us = self.tgt = 1570
        self.left = 0
        self.sent = []
        self.replies = []
        self.silent = False

    def reset_input_buffer(self):
        self.replies.clear()

    def flush(self):
        pass

    def close(self):
        self.closed_link = True

    def write(self, data):
        command = data.decode().strip()
        self.sent.append(command)
        if self.silent:
            return
        if command == "?":
            if self.left:
                self.left -= 1
            else:
                self.us = self.tgt
            self.replies.append(f"S us={self.us} tgt={self.tgt} lo=1480 hi=1660 cal={self.cal} closed=1660 open=1480 "
                                f"pct=50.0 att={self.att} spd=3 sweep=0 unl=0 pin=44")
        elif command == "A":
            self.att = 1
            self.replies.append("OK attached on GPIO 44 at 1570")
        elif command == "R":
            self.att = 0
            self.replies.append("OK R")
        elif command[0] in "OCP" and not self.att:
            self.replies.append("ERR not attached")     # the driver must attach first
        elif command[0] in "OCP":
            self.tgt = {"O": 1480, "C": 1660}.get(command[0], 1570)
            self.left = self.ticks
            self.replies.append(f"OK {command}")
        else:
            self.replies.append("ERR unknown command")

    def readline(self):
        return (self.replies.pop(0) + "\r\n").encode() if self.replies else b""


def test_an_uncalibrated_board_is_refused():
    with pytest.raises(GripperError, match="calibrated"):
        Gripper(FakeBoard(cal=0))


def test_the_status_line_is_read():
    state = Gripper(FakeBoard()).status()
    assert (state["closed"], state["open"], state["cal"], state["att"]) == (1660, 1480, 1, 0)


def test_the_first_move_starts_the_pulses_then_later_ones_dont():
    board = FakeBoard()
    jaws = Gripper(board)
    jaws.open()
    jaws.close()
    assert [c for c in board.sent if c != "?"] == ["A", "O", "C"]


def test_a_move_returns_only_once_the_jaws_are_there():
    board = FakeBoard(ticks=3)
    Gripper(board).close()
    assert board.us == board.tgt == 1660
    assert board.left == 0


def test_release_stops_the_pulses():
    board = FakeBoard(att=1)
    Gripper(board).release()
    assert board.sent[-1] == "R" and board.att == 0


def test_a_refused_command_raises():
    jaws = Gripper(FakeBoard())
    with pytest.raises(GripperError, match="unknown"):
        jaws._ask("Z")


def test_a_silent_board_raises(monkeypatch):
    monkeypatch.setattr(driver, "REPLY_S", 0.01)
    board = FakeBoard()
    jaws = Gripper(board)
    board.silent = True
    with pytest.raises(GripperError, match="no reply"):
        jaws.status()


def test_jaws_that_never_arrive_raise(monkeypatch):
    monkeypatch.setattr(driver, "MOVE_S", 0.01)
    monkeypatch.setattr(driver, "POLL_S", 0.001)
    with pytest.raises(GripperError, match="finish"):
        Gripper(FakeBoard(ticks=10 ** 6)).close()


def test_the_board_is_found_by_its_usb_id(monkeypatch):
    class Port:
        def __init__(self, device, vid):
            self.device, self.vid = device, vid

    monkeypatch.setattr(driver.list_ports, "comports", lambda: [Port("COM8", 0x0483), Port("COM9", 0x303A)])
    assert find_port() == "COM9"
    monkeypatch.setattr(driver.list_ports, "comports", lambda: [Port("COM8", 0x0483)])
    with pytest.raises(GripperError, match="none"):
        find_port()
