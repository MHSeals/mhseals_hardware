"""Small Unix terminal key reader used by the interactive ROS TUIs."""

import os
import select
import sys
import termios
import tty


class KeyReader:
    """Read individual keys while restoring the terminal on every exit."""

    def __init__(self):
        self.fd = None
        self.settings = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError('keyboard control requires an interactive TTY')
        self.fd = sys.stdin.fileno()
        self.settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        if self.settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)

    def read(self, timeout=None):
        """Return a normalized key name, or None when the timeout expires."""
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        value = os.read(self.fd, 1)
        if value in (b'\r', b'\n'):
            return 'enter'
        if value == b' ':
            return 'space'
        if value == b'\x03':
            raise KeyboardInterrupt
        if value != b'\x1b':
            return value.decode('utf-8', errors='ignore').lower()
        # Arrow keys arrive as ESC [ A/B/C/D.
        if not select.select([self.fd], [], [], 0.03)[0]:
            return 'escape'
        sequence = os.read(self.fd, 2)
        return {
            b'[A': 'up', b'[B': 'down',
            b'[C': 'right', b'[D': 'left',
        }.get(sequence, 'escape')
