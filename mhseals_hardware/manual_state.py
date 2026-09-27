"""Clock-injected deadman state; independent of ROS and terminal rendering."""

import math

MANUAL_KEYS = {
    'w': ('surge', 1.0), 'up': ('surge', 1.0),
    's': ('surge', -1.0), 'down': ('surge', -1.0),
    'a': ('sway', 1.0), 'd': ('sway', -1.0),
    'left': ('yaw', 1.0), 'right': ('yaw', -1.0),
}


class ManualState:
    def __init__(self, amplitude=0.25, repeat_timeout=0.15, initial_timeout=0.65):
        if not math.isfinite(amplitude) or not 0 <= amplitude <= 1:
            raise ValueError('amplitude must be within [0, 1]')
        if not all(math.isfinite(v) and 0 < v <= 2
                   for v in (repeat_timeout, initial_timeout)):
            raise ValueError('deadman timeouts must be within (0, 2] seconds')
        self.amplitude = amplitude
        self.repeat_timeout = repeat_timeout
        self.initial_timeout = initial_timeout
        self.key = None
        self.deadline = 0.0
        self.held = set()
        self.event_mode = False
        self.focused = True

    def update(self, key, now):
        # A shared watchdog is intentional: terminals usually repeat only the
        # newest held key. Expiring each key separately breaks diagonal holds.
        if now >= self.deadline:
            self.key = None
            self.held.clear()
        kind, _, name = (key or '').partition(':')
        if kind not in ('press', 'repeat', 'release'):
            kind, name = 'legacy', key
        elif kind in ('press', 'repeat'):
            self.event_mode = True
        if name == 'focus-out':
            self.focused = False
        elif name == 'focus-in':
            self.focused = True
        if name in ('space', 'focus-out', 'focus-in') and kind != 'release':
            self.key = None
            self.held.clear()
        elif kind == 'release':
            self.held.discard(name)
            if name == self.key:
                self.key = None
            # The OS may restart its initial repeat delay for the remaining key.
            if self.held:
                self.deadline = now + self.initial_timeout
        elif name in ('+', '=', '-', '_'):
            self.amplitude = round(max(0.0, min(1.0, self.amplitude +
                                   (0.05 if name in ('+', '=') else -0.05))), 2)
        elif name in MANUAL_KEYS and self.focused and kind in ('press', 'repeat'):
            # Never re-arm a timed-out/stopped chord from repeat events alone.
            if kind == 'press':
                self.held.add(name)
                self.deadline = now + self.initial_timeout
            elif name in self.held:
                self.deadline = now + self.repeat_timeout
            self.key = None
        elif name in MANUAL_KEYS and self.focused and not self.event_mode:
            timeout = (self.repeat_timeout if name == self.key
                       else self.initial_timeout)
            self.key = name
            self.deadline = now + timeout
        keys = self.held if self.event_mode else ({self.key} if self.key else set())
        # Aliases do not double an axis, opposing directions cancel.
        axes = {axis: set() for axis in ('surge', 'sway', 'yaw')}
        for active in keys:
            axis, direction = MANUAL_KEYS[active]
            axes[axis].add(direction)
        x, y, yaw = (sum(axes[axis]) for axis in ('surge', 'sway', 'yaw'))
        scale = max(1.0, math.hypot(x, y))
        return (x / scale * self.amplitude, y / scale * self.amplitude,
                yaw * self.amplitude)
