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


class Mouse(str):
    """An SGR mouse report (``CSI < b ; col ; row M|m``) as a key name with its details.

    The name is ``press:mouse-left``, ``release:mouse-left``, ``drag:mouse-left``, ``move:mouse``
    or ``wheel-up``/``wheel-down``/``wheel-left``/``wheel-right``, so press and release feed
    `Hold` like keys. ``action``, ``button``, ``col``, ``row`` (1-based cells) and ``mods``
    (``shift``, ``alt``, ``ctrl``) are attributes. Reports come only once a TUI turns them on
    (`MOUSE_ON`); nothing does yet.
    """

    def __new__(cls, action, button, col, row, mods=frozenset()):
        if action == 'wheel':
            name = f'wheel-{button}'
        elif button is None:
            name = f'{action}:mouse'
        else:
            name = f'{action}:mouse-{button}'
        self = super().__new__(cls, name)
        self.action, self.button, self.col, self.row, self.mods = action, button, col, row, frozenset(mods)
        return self


MOUSE_ON = '\x1b[?1002h\x1b[?1006h'    # button presses, releases and drags, in SGR form
MOUSE_OFF = '\x1b[?1002l\x1b[?1006l'
_BUTTONS = {0: 'left', 1: 'middle', 2: 'right'}
_WHEEL = {0: 'up', 1: 'down', 2: 'left', 3: 'right'}


def decode_mouse(value):
    """A `Mouse` from one SGR mouse report, or None when ``value`` is not one."""
    match = re.fullmatch(rb'\x1b\[<(\d+);(\d+);(\d+)([Mm])', value)
    if not match:
        return None
    code, col, row = int(match[1]), int(match[2]), int(match[3])
    mods = {name for bit, name in ((4, 'shift'), (8, 'alt'), (16, 'ctrl')) if code & bit}
    low = code & 3
    if code & 64:
        return Mouse('wheel', _WHEEL[low], col, row, mods)
    if code & 32:
        return Mouse('drag', _BUTTONS.get(low), col, row, mods) if low != 3 else Mouse('move', None, col, row, mods)
    return Mouse('release' if match[4] == b'm' else 'press', _BUTTONS.get(low), col, row, mods)


class Terminal:
    """What the terminal can report, for the input layer and its indicators.

    ``kitty``: the kitty keyboard protocol (it answered ``CSI ? u``): press, repeat and release
    events, so held keys chord. ``legacy``: tmux (and cx, which runs on it) forwards plain keys
    only, so a held key is its auto-repeat stream and only the newest key repeats; manual mode
    (`KeyReader.events`) also asks the terminal outside tmux, and held keys chord once its events
    come through (allow-passthrough on, a kitty-protocol terminal, no tmux nested between). ``plain``: a
    terminal that answered the probe without the protocol. ``unknown``: not probed; manual mode
    finds out from the keys themselves (ManualState.chords).
    """

    KINDS = ('kitty', 'legacy', 'plain', 'unknown')

    def __init__(self, kind='unknown'):
        if kind not in self.KINDS:
            raise ValueError(f'unknown terminal kind {kind!r}')
        self.kind = kind

    @property
    def releases(self):
        """Key releases are reported (kitty), so holds end at the release, not a repeat gap."""
        return self.kind == 'kitty'

    @property
    def chords_unavailable(self):
        """Why held keys cannot chord here (``tmux``, ``no kitty keys``), or None when they can or may."""
        return {'legacy': 'tmux', 'plain': 'no kitty keys'}.get(self.kind)

    @classmethod
    def detect(cls, caps=None, env=None):
        """From the session probe's replies (``keyboard``: the ``CSI ? u`` flags, ``done``: DA1 seen,
        ``xtversion``) and the environment (``TMUX``)."""
        caps = caps or {}
        env = os.environ if env is None else env
        if env.get('TMUX') or (caps.get('xtversion') or '').lower().startswith('tmux'):
            return cls('legacy')
        if caps.get('keyboard') is not None:
            return cls('kitty')
        return cls('plain' if caps.get('done') else 'unknown')

    def __repr__(self):
        return f'Terminal({self.kind!r})'


class Hold:
    """Hold-to-confirm: ``feed(key, now)`` is True once ``name`` has been held ``hold_s``.

    With release events (kitty keys, any mouse button) a press starts the hold and only its
    release, or another key, ends it. Without them the key-repeat stream is the hold: the first
    repeat must come within ``first_gap_s`` (the OS's initial delay) and each next within
    ``gap_s``. ``tick(now)`` applies the gap with no key; ``progress(now)`` is 0..1 for a bar.
    """

    def __init__(self, name, hold_s, releases, first_gap_s=0.65, gap_s=0.15):
        self.name, self.hold_s, self.releases = name, hold_s, releases
        self.first_gap_s, self.gap_s = first_gap_s, gap_s
        self.started = self.last = None
        self.repeated = False

    def reset(self):
        self.started = self.last = None
        self.repeated = False

    def tick(self, now):
        if self.started is not None and not self.releases:
            if now - self.last > (self.gap_s if self.repeated else self.first_gap_s):
                self.reset()
        return self.done(now)

    def feed(self, key, now):
        self.tick(now)
        kind, _, name = (key or '').rpartition(':')
        kind = kind or 'plain'
        if name != self.name:
            if key is not None:
                self.reset()       # any other key cancels
            return self.done(now)
        if kind == 'release':
            self.reset()
        elif self.started is None:
            if kind != 'repeat':   # a repeat never starts a hold
                self.started = self.last = now
        else:
            self.repeated, self.last = self.repeated or kind != 'press', now
        return self.done(now)

    def progress(self, now):
        return 0.0 if self.started is None else min(1.0, (now - self.started) / self.hold_s)

    def done(self, now):
        return self.started is not None and now - self.started >= self.hold_s


# Terminal replies are never keys: the TUI's capability probe (mhseals_testing ui/imaging/probe.py,
# the only one) may be answered after it stops listening. Strings end at ST (ESC \); OSC may also end at BEL.
STRING_INTRODUCERS = (b'_', b'P', b']', b'^', b'X')


def string_end(buffer):
    """The length of the APC/DCS/OSC/PM/SOS string starting ``buffer`` with its terminator, or None."""
    st = buffer.find(b'\x1b\\', 2)
    bel = buffer.find(b'\x07', 2) if buffer[1:2] == b']' else -1
    ends = [i + 2 for i in (st,) if i >= 0] + [i + 1 for i in (bel,) if i >= 0]
    return min(ends) if ends else None


def string_complete(buffer):
    """True once an APC/DCS/OSC/PM/SOS string in ``buffer`` has its terminator."""
    return string_end(buffer) == len(buffer)


# Kitty keyboard protocol (disambiguate, event types, all keys) and focus reports; terminals
# without it ignore the request and keep sending legacy keys.
EVENTS_ON = '\x1b[>11u\x1b[?1004h'
EVENTS_OFF = '\x1b[<u\x1b[?1004l'


def tmux_wrap(sequence):
    """``sequence`` for the terminal outside tmux: a passthrough DCS with its ESCs doubled."""
    return '\x1bPtmux;' + sequence.replace('\x1b', '\x1b\x1b') + '\x1b\\'


# Under tmux the kitty request also goes to the terminal outside it (allow-passthrough on): tmux keeps
# no keyboard flags of its own but hands the CSI u sequences it can't parse, key events among them,
# to the pane as they came. The focus reports stay tmux's (focus-events on).
TMUX_EVENTS_ON = tmux_wrap('\x1b[>11u')
TMUX_EVENTS_OFF = tmux_wrap('\x1b[<u')


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
        self.terminal = Terminal()   # what the terminal reports; a TUI's probe tells it (Terminal.detect)

    def absorb(self, value):
        """True when ``value`` is a terminal reply, never a key: an APC (kitty graphics), DCS
        (XTVERSION), OSC, PM or SOS string, ``CSI 6 ; h ; w t`` (cell size), the DA1 ``CSI ? ... c`` or
        the kitty keyboard flags ``CSI ? flags u``."""
        return (value[:2] in (b'\x1b_', b'\x1bP', b'\x1b]', b'\x1b^', b'\x1bX')
                or bool(re.fullmatch(rb'\x1b\[6;\d+;\d+t|\x1b\[\?[\d;]*c|\x1b\[\?\d*u', value)))

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
            terminal_write(EVENTS_ON)
        return self

    def __exit__(self, *_):
        try:
            if self.report_events:
                terminal_write(EVENTS_OFF)
        finally:
            if self.settings is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.settings)

    def events(self):
        """This reader with key events on (kitty press/repeat/release, focus) until the block ends.

        The same reader, buffer and terminal state: manual mode reuses the session's reader, so
        bytes queued before it starts are read in order and nothing is lost when it ends."""
        return _Events(self)

    def _take(self, now):
        """The first complete sequence in the buffer, removed; None while it is incomplete.

        A lone ESC is the Escape key only 50 ms after it arrived; a CSI/SS3 sequence ends at its
        final byte (0x40-0x7E); an APC/DCS/OSC/PM/SOS string at ST (OSC also at BEL); a UTF-8
        character after its last byte."""
        buffer = self.buffer
        if buffer[0] != 27:
            lead = buffer[0]
            size = 1 if lead < 0xc0 else 2 if lead < 0xe0 else 3 if lead < 0xf0 else 4
            end = size if len(buffer) >= size or now - self.escape_started >= 0.05 else None
        elif len(buffer) == 1:
            end = 1 if now - self.escape_started >= 0.05 else None
        elif buffer[1:2] in STRING_INTRODUCERS:
            end = string_end(buffer)
        elif buffer[1] not in (ord('['), ord('O')):
            end = 2
        else:
            end = next((i + 1 for i in range(2, len(buffer)) if 0x40 <= buffer[i] <= 0x7e), None)
        if end is None:
            return None
        value = bytes(buffer[:end])
        del buffer[:end]
        if buffer:
            self.escape_started = now
        return value

    def read(self, timeout=None):
        """Return a normalized key name, or None when the timeout expires.

        The terminal is read in bulk; sequences are cut from the buffer one at a time (`_take`).
        A terminal reply (`absorb`) is never a key, and an SGR mouse report is a `Mouse`.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            now = time.monotonic()
            if self.buffer:
                value = self._take(now)
                if value is not None:
                    if value[0] == 27 and self.absorb(value):
                        continue
                    mouse = decode_mouse(value)
                    return mouse if mouse is not None else decode_key(value, self.report_events)
                string = self.buffer[1:2] in STRING_INTRODUCERS
                max_len = 512 if string else 64
                if len(self.buffer) > max_len or now - self.escape_started > 1.0:
                    self.buffer.clear()
                    if string:
                        # Unterminated APC/DCS/OSC/PM/SOS (e.g. Alt+] on a
                        # plain terminal): drop it rather than fail the TUI.
                        continue
                    raise ValueError('incomplete or oversized terminal key sequence')
            wait = None if deadline is None else max(0.0, deadline - now)
            if self.buffer and (len(self.buffer) == 1 or self.buffer[0] != 27):
                pending = max(0.0, .05 - (now - self.escape_started))   # a lone ESC or a partial UTF-8 character
                wait = pending if wait is None else min(wait, pending)
            ready, _, _ = select.select([self.fd], [], [], wait)
            if not ready:
                if deadline is not None and time.monotonic() >= deadline:
                    return None
                continue
            part = os.read(self.fd, 4096)
            if not part:
                raise EOFError('terminal input disconnected')
            if not self.buffer:
                self.escape_started = time.monotonic()
            self.buffer.extend(part)


class _Events:
    """`KeyReader.events`: report key events for a block, then restore the reader's mode."""

    def __init__(self, reader):
        self.reader, self.before, self.outer = reader, None, False

    def __enter__(self):
        self.before, self.reader.report_events = self.reader.report_events, True
        self.outer = self.reader.terminal.kind == 'legacy'   # tmux: ask its terminal too
        if not self.before:
            terminal_write(EVENTS_ON + (TMUX_EVENTS_ON if self.outer else ''))
        return self.reader

    def __exit__(self, *_):
        if not self.before:
            terminal_write(EVENTS_OFF + (TMUX_EVENTS_OFF if self.outer else ''))
        self.reader.report_events = self.before
