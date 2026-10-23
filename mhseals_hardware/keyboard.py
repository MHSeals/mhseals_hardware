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
    # Ctrl+arrows (modifier 5).
    b'\x1b[1;5A': 'ctrl-up', b'\x1b[1;5B': 'ctrl-down',
    b'\x1b[1;5C': 'ctrl-right', b'\x1b[1;5D': 'ctrl-left',
    # Shift+arrows (modifier 2).
    b'\x1b[1;2A': 'shift-up', b'\x1b[1;2B': 'shift-down',
    b'\x1b[1;2C': 'shift-right', b'\x1b[1;2D': 'shift-left',
}
# Ctrl+H/J/K/L arrive as the control bytes BS, LF, VT, FF. Enter is CR, because
# KeyReader turns off the terminal's CR-to-LF translation to keep the two apart.
CONTROL_KEYS = {b'\x08': 'ctrl-h', b'\n': 'ctrl-j', b'\x0b': 'ctrl-k', b'\x0c': 'ctrl-l'}


def decode_key(value, report_events=False):
    """Normalize a complete terminal byte sequence into a key name.

    Key case is kept (``P`` is not ``p``). Plain terminals send one byte per
    key, or a short escape sequence for arrows. Terminals that speak the Kitty
    keyboard protocol (enabled by KeyReader when ``report_events`` is set) send
    ``ESC [ code ; mods : event u`` instead, which also reports key release
    (event 3) and auto-repeat (event 2). That is what lets manual control hold
    two keys at once. With
    ``report_events`` the result is prefixed ``press:``, ``repeat:`` or
    ``release:``.
    """
    # Ctrl+C as a Kitty sequence (code 99 'c' or 67 'C' with a Ctrl modifier).
    if re.fullmatch(rb'\x1b\[(?:99|67);[56](?::[12])?u', value):
        raise KeyboardInterrupt
    # Kitty key event: unicode code point, optional modifiers and event type.
    event = re.fullmatch(rb'\x1b\[(\d+)(?::[\d:]*)?(?:;(\d+)(?::([123]))?)?(?:;[\d:]*)?u', value)
    # Kitty arrow keys keep the legacy letters but add the event type.
    arrow = re.fullmatch(rb'\x1b\[1;\d+:([123])([ABCD])', value)
    if event:
        code = int(event[1])
        if code > 0x10ffff:
            return None
        # 573xx are Kitty's private-use code points for the arrow keys.
        key = {13: 'enter', 27: 'escape', 32: 'space',
               57352: 'up', 57353: 'down',
               57350: 'left', 57351: 'right'}.get(code, chr(code))
        # Kitty reports the unshifted code point; the shift modifier (bit 0 of
        # modifiers - 1) makes it the upper-case key. Caps lock alone does not.
        if len(key) == 1 and event[2] and (int(event[2]) - 1) & 1:
            key = key.upper()
        if key == '\x03':
            raise KeyboardInterrupt
        if report_events:
            return {b'2': 'repeat:', b'3': 'release:'}.get(event[3], 'press:') + key
        return f'release:{key}' if event[3] == b'3' else key
    if arrow:
        key = {b'A': 'up', b'B': 'down', b'C': 'right', b'D': 'left'}[arrow[2]]
        if report_events:
            return {b'1': 'press:', b'2': 'repeat:', b'3': 'release:'}[arrow[1]] + key
        return f'release:{key}' if arrow[1] == b'3' else key
    if value in (b'\x1b[I', b'\x1b[O'):
        return 'focus-in' if value == b'\x1b[I' else 'focus-out'
    if value == b'\r':
        return 'enter'
    if value in CONTROL_KEYS:
        return CONTROL_KEYS[value]
    if value == b' ':
        return 'space'
    if value == b'\x03':
        raise KeyboardInterrupt
    if value in ARROW_SEQUENCES:
        # With event reporting on, a terminal sends an arrow press as the plain
        # sequence (only repeats and releases carry an event type), so it is a press.
        plain = not ARROW_SEQUENCES[value].startswith(('ctrl-', 'shift-'))
        return ('press:' if report_events and plain else '') + ARROW_SEQUENCES[value]
    if value == b'\x1b[Z':  # Shift+Tab
        return 'backtab'
    if value == b'\x1b':
        return 'escape'
    return value.decode('utf-8', errors='ignore')


# Terminal capability probe, sent once by KeyReader: a 1x1 kitty graphics query,
# XTVERSION, the cell size in pixels (CSI 16 t) and DA1 last. Every terminal answers
# DA1, so its reply closes the probe: a graphics "OK" reply that comes later (or
# never, as under tmux) does not count.
GRAPHICS_ID = 31
PROBE = (f'\x1b_Gi={GRAPHICS_ID},s=1,v=1,a=q,t=d,f=24;AAAA\x1b\\'
         '\x1b[>0q\x1b[16t\x1b[c')
# Strings end at ST (ESC \); OSC may also end at BEL.
STRING_INTRODUCERS = (b'_', b'P', b']', b'^', b'X')


def string_complete(buffer):
    """True once an APC/DCS/OSC/PM/SOS string in ``buffer`` has its terminator."""
    return buffer.endswith(b'\x1b\\') or (buffer[1:2] == b']' and buffer.endswith(b'\x07'))


def terminal_write(sequence):
    """Send a control sequence to the real terminal.

    sys.__stdout__, not sys.stdout: a full-screen Rich Live redirects sys.stdout
    to print captured text above the display, which would swallow the request.
    """
    stream = sys.__stdout__ or sys.stdout
    stream.write(sequence)
    stream.flush()


class KeyReader:
    """Read individual keys while restoring the terminal on every exit."""

    def __init__(self, report_events=False):
        self.fd = None
        self.settings = None
        self.report_events = report_events
        self.buffer = bytearray()
        self.escape_started = None
        self.graphics = False  # kitty graphics answered before DA1
        self.cell_px = None  # (width, height) of one cell in pixels, if reported
        self.xtversion = None
        self.probed = False  # DA1 seen: the probe replies are all in
        self.probe = report_events

    def absorb(self, value):
        """Consume a terminal reply to the probe; True when ``value`` was one.

        Replies are never keys: the APC graphics answer, the DCS XTVERSION
        answer, ``CSI 6 ; h ; w t`` (cell size) and the DA1 ``CSI ? ... c``.
        """
        if value[:2] == b'\x1b_':
            if not self.probed and value.startswith(b'\x1b_Gi=%d;OK' % GRAPHICS_ID):
                self.graphics = True
            return True
        if value[:2] == b'\x1bP':
            if value.startswith(b'\x1bP>|'):
                self.xtversion = value[4:-2].decode('utf-8', errors='replace')
            return True
        if value[:2] in (b'\x1b]', b'\x1b^', b'\x1bX'):
            return True
        size = re.fullmatch(rb'\x1b\[6;(\d+);(\d+)t', value)
        if size:
            height, width = int(size[1]), int(size[2])
            if height > 0 and width > 0:
                self.cell_px = (width, height)
            return True
        if re.fullmatch(rb'\x1b\[\?[\d;]*c', value):
            self.probed = True
            return True
        return False

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError('keyboard control requires an interactive TTY')
        self.fd = sys.stdin.fileno()
        self.settings = termios.tcgetattr(self.fd)
        # Be explicit instead of relying on the host Python's setcbreak
        # behavior. Nested SSH/docker PTYs otherwise sometimes retain ECHO.
        attributes = termios.tcgetattr(self.fd)
        attributes[0] &= ~termios.ICRNL  # Enter stays CR, so Ctrl+J (LF) is its own key
        attributes[3] &= ~(
            termios.ICANON | termios.ECHO | termios.ECHONL)
        attributes[6][termios.VMIN] = 1
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
        if self.report_events:
            # Kitty keyboard protocol: disambiguate, event types, all keys.
            # Unsupported terminals ignore this request and retain the fallback.
            terminal_write('\x1b[>11u\x1b[?1004h')
        if self.probe and not (os.environ.get('NO_COLOR') or os.environ.get('MHS_ASCII')):
            terminal_write(PROBE)
        return self

    def __exit__(self, *_):
        try:
            if self.report_events:
                terminal_write('\x1b[<u\x1b[?1004l')
        finally:
            if self.settings is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.settings)

    def read(self, timeout=None):
        """Return a normalized key name, or None when the timeout expires.

        Bytes are buffered until a sequence is complete: a lone ESC counts as
        the Escape key only after 50 ms without follow-up bytes, and a CSI/SS3
        sequence ends at its final byte (0x40-0x7E).
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            now = time.monotonic()
            if self.buffer:
                complete = self.buffer[0] != 27
                if self.buffer[0] == 27:
                    if len(self.buffer) == 1:
                        complete = now - self.escape_started >= 0.05
                    elif self.buffer[1:2] in STRING_INTRODUCERS:
                        complete = string_complete(self.buffer)
                    elif self.buffer[1] not in (ord('['), ord('O')):
                        complete = True
                    else:
                        complete = len(self.buffer) > 2 and 0x40 <= self.buffer[-1] <= 0x7e
                if complete:
                    value = bytes(self.buffer)
                    self.buffer.clear()
                    if value[0] == 27 and self.absorb(value):
                        continue
                    return decode_key(value, self.report_events)
                if len(self.buffer) > (512 if self.buffer[1:2] in STRING_INTRODUCERS else 64) or now - self.escape_started > 1.0:
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
