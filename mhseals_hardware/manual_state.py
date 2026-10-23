"""Clock-injected deadman state; independent of ROS and terminal rendering."""

import math

MANUAL_KEYS = {
    'w': ('surge', 1.0), 'up': ('surge', 1.0),
    's': ('surge', -1.0), 'down': ('surge', -1.0),
    'a': ('sway', 1.0), 'd': ('sway', -1.0),
    'left': ('yaw', 1.0), 'right': ('yaw', -1.0),
}
# A bare arrow is a Kitty press or a plain key (tmux forwards no events), so
# only a later repeat or release proves the terminal reports releases.
ARROWS = ('up', 'down', 'left', 'right')
# A terminal repeats only the newest key, so a held chord key goes quiet while
# another is down or after it is released. After a release, the keys still
# held are trusted for this long without a repeat; their release stops them.
CHORD_HOLD = 2.0


class ManualState:
    """Held motion keys -> one (surge, sway, yaw) effort command.

    Once the terminal reports releases (Kitty protocol, also over SSH), held
    keys chord and each stops on its own release. Until then (tmux, plain
    terminals) one key drives: the newest replaces it, and it stops when its
    repeats do. One watchdog covers every key, because a terminal repeats
    only the newest: expiring keys separately would break diagonals.

    ``chords`` and ``command`` are what a display shows.
    """

    def __init__(self, amplitude=0.25, repeat_timeout=0.15, initial_timeout=0.65,
                 yaw_scale=0.5):
        if not math.isfinite(amplitude) or not 0 <= amplitude <= 1:
            raise ValueError('amplitude must be within [0, 1]')
        if not all(math.isfinite(v) and 0 < v <= 2
                   for v in (repeat_timeout, initial_timeout)):
            raise ValueError('deadman timeouts must be within (0, 2] seconds')
        if not math.isfinite(yaw_scale) or not 0 < yaw_scale <= 1:
            raise ValueError('yaw_scale must be within (0, 1]')
        self.amplitude = amplitude
        self.repeat_timeout = repeat_timeout
        self.initial_timeout = initial_timeout
        self.yaw_scale = yaw_scale  # yaw is far more aggressive at the same effort
        self.deadline = 0.0
        self.held = set()   # keys that will report their release
        self.single = None  # the one key that won't: held while it repeats
        self.chords = False
        self.focused = True
        self.command = (0.0, 0.0, 0.0)

    def update(self, key, now):
        if now >= self.deadline:
            self.held.clear()
            self.single = None
        kind, _, name = (key or '').partition(':')
        if kind not in ('press', 'repeat', 'release'):
            kind, name = 'plain', key
        if name and len(name) == 1:
            name = name.lower()  # caps lock or shift must not stop WASD
        if kind in ('repeat', 'release') or (kind == 'press' and name not in ARROWS):
            self.chords = True
            if name == self.single and kind != 'release':
                self.held.add(name)  # a bare arrow that turned out to report events
                self.single = None
        if name == 'focus-out':
            self.focused = False
        elif name == 'focus-in':
            self.focused = True
        if name in ('space', 'focus-out', 'focus-in') and kind != 'release':
            self.held.clear()
            self.single = None
        elif kind == 'release':
            was_held = name in self.held
            self.held.discard(name)
            if name == self.single:
                self.single = None
            # The OS may restart its initial repeat delay for the remaining key.
            # Only a held motion key's release counts: other releases (+/-,
            # never-held keys) must not keep a key whose release was lost alive.
            if was_held and self.held:
                self.deadline = now + CHORD_HOLD
            elif was_held and self.single:
                self.deadline = now + self.initial_timeout
        elif name in ('+', '=', '-', '_'):
            self.amplitude = round(max(0.0, min(1.0, self.amplitude +
                                   (0.05 if name in ('+', '=') else -0.05))), 2)
        elif name in MANUAL_KEYS and self.focused:
            if name in self.held or name == self.single:
                self.deadline = now + self.repeat_timeout
            elif kind != 'repeat':  # never re-arm a stopped key from repeats alone
                if self.chords and kind == 'press':
                    self.held.add(name)
                else:
                    self.single = name
                self.deadline = now + self.initial_timeout
        # Aliases do not double an axis; opposites cancel.
        axes = {axis: set() for axis in ('surge', 'sway', 'yaw')}
        for active in self.held | ({self.single} if self.single else set()):
            axis, direction = MANUAL_KEYS[active]
            axes[axis].add(direction)
        x, y, yaw = (sum(axes[axis]) for axis in ('surge', 'sway', 'yaw'))
        scale = max(1.0, math.hypot(x, y))
        self.command = (x / scale * self.amplitude, y / scale * self.amplitude,
                        yaw * self.amplitude * self.yaw_scale)
        return self.command
