"""Gripper v5: jog the jaws from the keyboard and save the open and closed ends.

    python gripper_jog.py [--port COM9] [--small 10] [--big 50] [--flip]

Keys (no Enter needed):
    c / C   nudge closed, small / big step
    o / O   nudge open,   small / big step
    f       flip which pulse direction counts as "open" (if c opens the jaws, press f)
    1       save the current position as CLOSED     2   save it as OPEN
    w       write both to the board's flash (survives a reset)
    [ / ]   go to the saved closed / open end
    r       release (servo limp; the next c/o attaches again)    a   attach without jogging
    ?       status                  q   quit (releases the servo)

It starts by unlocking the calibration envelope (1500 +/- 550 us, 1 us per 10 ms) and leaves the servo
RELEASED: nothing moves until the first c/C/o/O, which attaches at the board's remembered pulse (shown at
start, the last one it held) and jogs from there. If the shaft hasn't been turned since that release, only
the step moves. After a board reset the remembered pulse is 1500, not where the shaft is: the first press
may then jump. --at <us> attaches there instead. Nothing stops a jog at the jaws, so near closed use small steps and stop at
first contact: the jaw bodies are a hard stop.
"""
import argparse
import msvcrt
import re
import threading
import time

import serial

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--port', default='COM9')
ap.add_argument('--small', type=int, default=10, help='small step, us (default 10)')
ap.add_argument('--big', type=int, default=50, help='big step, us (default 50)')
ap.add_argument('--at', type=int, help='attach at this pulse on the first jog instead of the remembered one')
ap.add_argument('--flip', action='store_true', help='start with higher us = closed (default: lower us closes, as found on v5)')
args = ap.parse_args()

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = args.port, 115200, 0.1
ser.dtr = ser.rts = False  # don't reset the board on open
ser.open()

last_status = {}
attached = False  # the script never attaches on its own
close_sign = +1 if args.flip else -1  # +1: a higher pulse closes. On v5 (2026-11-17) a lower pulse closes.


def reader():
    buf = b''
    while ser.is_open:
        try:
            buf += ser.read(256)
        except serial.SerialException:
            return
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            text = line.decode(errors='replace').strip()
            if not text:
                continue
            if text.startswith('S '):
                last_status.update(dict(re.findall(r'(\w+)=(-?[\d.]+)', text)))
            print('   ' + text)


threading.Thread(target=reader, daemon=True).start()


def send(cmd):
    ser.write((cmd + '\n').encode())
    time.sleep(0.05)


def wait_at_rest(timeout=10.0):
    """Poll status until the servo reaches its target (SC/SO refuse while moving)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        last_status.clear()
        send('?')
        time.sleep(0.15)
        if last_status and last_status.get('us') == last_status.get('tgt'):
            return True
    print('   (still moving, try again)')
    return False


def direction():
    return 'higher us = CLOSED' if close_sign > 0 else 'lower us = CLOSED'


print(__doc__.split('Keys')[0].strip().splitlines()[0])
print(f'port {args.port}, small {args.small} us, big {args.big} us, {direction()}')
print('keys: c/C close  o/O open  f flip  1 save closed  2 save open  w write  [ ] go ends  r release  a attach  ? status  q quit')
time.sleep(0.3)
send('U')
send('?')
time.sleep(0.2)
attached = last_status.get('att') == '1'
if attached:
    print('   servo already attached (another session left it on)')
else:
    at = args.at if args.at is not None else last_status.get('us', '?')
    print(f'   servo RELEASED, nothing moves until c/C/o/O (attaches at {at} us, then jogs).')
    if args.at is None and last_status.get('us') == '1500' and last_status.get('unl') == '0':
        print('   (board looks freshly reset: 1500 may not be where the shaft is, so the first press may jump)')


def attach():
    global attached
    send(f'A {args.at}' if args.at is not None else 'A')
    attached = True

try:
    while True:
        k = msvcrt.getwch()
        if k in ('\x00', '\xe0'):  # arrow/function key prefix: swallow the second byte
            msvcrt.getwch()
            continue
        if k in 'cCoO' and not attached:
            attach()
        if k in '[]12' and not attached:
            print('   released: jog with c/o first')
            continue
        if k == 'c':
            send(f'J {close_sign * args.small}')
        elif k == 'C':
            send(f'J {close_sign * args.big}')
        elif k == 'o':
            send(f'J {-close_sign * args.small}')
        elif k == 'O':
            send(f'J {-close_sign * args.big}')
        elif k in 'fF':
            close_sign = -close_sign
            print(f'   flipped: {direction()}')
        elif k == '1':
            if wait_at_rest():
                send('SC')
        elif k == '2':
            if wait_at_rest():
                send('SO')
        elif k in 'wW':
            send('W')
            print('   written to flash')
        elif k == '[':
            send('C')
        elif k == ']':
            send('O')
        elif k in 'rR':
            send('R')
            attached = False
        elif k in 'aA':
            if not attached:
                attach()
        elif k == '?':
            send('?')
        elif k in 'qQ\x03\x1b':
            break
except KeyboardInterrupt:
    pass
finally:
    send('R')
    time.sleep(0.2)
    ser.close()
    print('released, bye')
