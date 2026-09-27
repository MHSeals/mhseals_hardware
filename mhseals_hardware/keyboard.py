"""Small Unix terminal key reader used by the interactive ROS TUIs."""

import os
import re
import select
import sys
import termios


ARROW_SEQUENCES = {
    b'\x1b[A': 'up', b'\x1b[B': 'down',
    b'\x1b[C': 'right', b'\x1b[D': 'left',
    b'\x1bOA': 'up', b'\x1bOB': 'down',
    b'\x1bOC': 'right', b'\x1bOD': 'left',
}


def decode_key(value):
    """Normalize a complete terminal byte sequence into a key name."""
    event = re.fullmatch(rb'\x1b\[(\d+)(?:;\d+(?::([123]))?)?u', value)
    arrow = re.fullmatch(rb'\x1b\[1;\d+:([123])([ABCD])', value)
    if event:
        code = int(event[1])
        if code > 0x10ffff:
            return None
        key = {13: 'enter', 27: 'escape', 32: 'space',
               57352: 'up', 57353: 'down',
               57350: 'left', 57351: 'right'}.get(code, chr(code).lower())
        return f'release:{key}' if event[2] == b'3' else key
    if arrow:
        key = {b'A': 'up', b'B': 'down', b'C': 'right', b'D': 'left'}[arrow[2]]
        return f'release:{key}' if arrow[1] == b'3' else key
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
            sys.stdout.write('\x1b[>11u')
            sys.stdout.flush()
        return self

    def __exit__(self, *_):
        if self.report_events:
            sys.stdout.write('\x1b[<u')
            sys.stdout.flush()
        if self.settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)

    def read(self, timeout=None):
        """Return a normalized key name, or None when the timeout expires."""
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        value = os.read(self.fd, 1)
        if value != b'\x1b':
            return decode_key(value)

        # SSH and Docker can split one escape sequence across several reads.
        # Collect the short burst rather than assuming '[A' arrives together.
        sequence = bytearray(value)
        while len(sequence) < 64:
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if not ready:
                break
            sequence.extend(os.read(self.fd, 1))
            if bytes(sequence) in ARROW_SEQUENCES:
                break
            if len(sequence) > 2 and 0x40 <= sequence[-1] <= 0x7e:
                break
        return decode_key(bytes(sequence))
