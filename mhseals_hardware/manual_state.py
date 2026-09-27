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

    def update(self, key, now):
        # Check expiry even when unrelated keys arrive (including speed keys).
        if now >= self.deadline:
            self.key = None
        if key == 'space' or key == f'release:{self.key}':
            self.key = None
        elif key in ('+', '=', '-', '_'):
            self.amplitude = round(max(0.0, min(1.0, self.amplitude +
                                   (0.05 if key in ('+', '=') else -0.05))), 2)
        elif key in MANUAL_KEYS:
            timeout = (self.repeat_timeout if key == self.key
                       else self.initial_timeout)
            self.key = key
            self.deadline = now + timeout
        if self.key is None:
            return None, 0.0
        axis, direction = MANUAL_KEYS[self.key]
        return axis, direction * self.amplitude
