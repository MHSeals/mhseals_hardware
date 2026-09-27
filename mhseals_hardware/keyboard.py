"""Small Unix terminal key reader used by the interactive ROS TUIs."""

import os
import re
import select
import sys
import termios
import time


ARROW_SEQUENCES = {
    b'\x1b[A': 'up', b'\x1b[B': 'down',
    b'\x1b[C': 'right', b'\x1b[D': 'left',
    b'\x1bOA': 'up', b'\x1bOB': 'down',
    b'\x1bOC': 'right', b'\x1bOD': 'left',
}


def decode_key(value, report_events=False):
    """Normalize a complete terminal byte sequence into a key name."""
    if re.fullmatch(rb'\x1b\[(?:99|67);[56](?::[12])?u', value):
        raise KeyboardInterrupt
    event = re.fullmatch(rb'\x1b\[(\d+)(?::[\d:]*)?(?:;\d+(?::([123]))?)?(?:;[\d:]*)?u', value)
    arrow = re.fullmatch(rb'\x1b\[1;\d+:([123])([ABCD])', value)
    if event:
        code = int(event[1])
        if code > 0x10ffff:
            return None
        key = {13: 'enter', 27: 'escape', 32: 'space',
               57352: 'up', 57353: 'down',
               57350: 'left', 57351: 'right'}.get(code, chr(code).lower())
        if key == '\x03':
            raise KeyboardInterrupt
        if report_events:
            return {b'2': 'repeat:', b'3': 'release:'}.get(event[2], 'press:') + key
        return f'release:{key}' if event[2] == b'3' else key
    if arrow:
        key = {b'A': 'up', b'B': 'down', b'C': 'right', b'D': 'left'}[arrow[2]]
        if report_events:
            return {b'1': 'press:', b'2': 'repeat:', b'3': 'release:'}[arrow[1]] + key
        return f'release:{key}' if arrow[1] == b'3' else key
    if value in (b'\x1b[I', b'\x1b[O'):
        return 'focus-in' if value == b'\x1b[I' else 'focus-out'
    if value in (b'\r', b'\n'):
        return 'enter'
    if value == b' ':
        return 'space'
    if value == b'\x03':
        raise KeyboardInterrupt
    if value in ARROW_SEQUENCES:
        return ARROW_SEQUENCES[value]
    if value == b'\x1b':
        return 'escape'
    return value.decode('utf-8', errors='ignore').lower()


class KeyReader:
    """Read individual keys while restoring the terminal on every exit."""

    def __init__(self, report_events=False):
        self.fd = None
        self.settings = None
        self.report_events = report_events
        self.buffer = bytearray()
        self.escape_started = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError('keyboard control requires an interactive TTY')
        self.fd = sys.stdin.fileno()
        self.settings = termios.tcgetattr(self.fd)
        # Be explicit instead of relying on the host Python's setcbreak
        # behavior. Nested SSH/docker PTYs otherwise sometimes retain ECHO.
        attributes = termios.tcgetattr(self.fd)
        attributes[3] &= ~(
            termios.ICANON | termios.ECHO | termios.ECHONL)
        attributes[6][termios.VMIN] = 1
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
        if self.report_events:
            # Kitty keyboard protocol: disambiguate, event types, all keys.
            # Unsupported terminals ignore this request and retain the fallback.
            sys.stdout.write('\x1b[>11u\x1b[?1004h')
            sys.stdout.flush()
        return self

    def __exit__(self, *_):
        try:
            if self.report_events:
                sys.stdout.write('\x1b[<u\x1b[?1004l')
                sys.stdout.flush()
        finally:
            if self.settings is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.settings)

    def read(self, timeout=None):
        """Return a normalized key name, or None when the timeout expires."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            now = time.monotonic()
            if self.buffer:
                complete = self.buffer[0] != 27
                if self.buffer[0] == 27:
                    if len(self.buffer) == 1:
                        complete = now - self.escape_started >= 0.05
                    elif self.buffer[1] not in (ord('['), ord('O')):
                        complete = True
                    else:
                        complete = len(self.buffer) > 2 and 0x40 <= self.buffer[-1] <= 0x7e
                if complete:
                    value = bytes(self.buffer)
                    self.buffer.clear()
                    return decode_key(value, self.report_events)
                if len(self.buffer) > 64 or now - self.escape_started > 1.0:
                    self.buffer.clear()
                    raise ValueError('incomplete or oversized terminal key sequence')
            wait = None if deadline is None else max(0.0, deadline - now)
            if self.buffer == b'\x1b':
                escape_wait = max(0.0, .05 - (now - self.escape_started))
                wait = escape_wait if wait is None else min(wait, escape_wait)
            ready, _, _ = select.select([self.fd], [], [], wait)
            if not ready:
                if deadline is not None and time.monotonic() >= deadline:
                    return None
                continue
            part = os.read(self.fd, 1)
            if not part:
                raise EOFError('terminal input disconnected')
            if not self.buffer:
                self.escape_started = time.monotonic()
            self.buffer.extend(part)
