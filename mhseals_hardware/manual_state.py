"""Clock-injected deadman state; independent of ROS and terminal rendering."""

import math
import time

# The manual bindings: action -> keys (a TUI passes its keymap's manual: section, the same action names).
BINDINGS = {
    'surge_fwd': ('w', 'up'), 'surge_back': ('s', 'down'),
    'sway_left': ('a',), 'sway_right': ('d',),
    'yaw_left': ('left',), 'yaw_right': ('right',),
    'speed_up': ('+', '='), 'speed_down': ('-', '_'),
}
AXES = {'surge_fwd': ('surge', 1.0), 'surge_back': ('surge', -1.0), 'sway_left': ('sway', 1.0),
        'sway_right': ('sway', -1.0), 'yaw_left': ('yaw', 1.0), 'yaw_right': ('yaw', -1.0)}
SPEED_STEP = {'speed_up': 0.05, 'speed_down': -0.05}


def fold(key):
    """Caps lock or shift must not stop WASD: a one-character key is its lower case."""
    return key.lower() if key and len(key) == 1 else key


def keys_of(bindings=None):
    """``(motion, speed)`` from ``bindings`` (action -> keys, BINDINGS by default): key -> (axis,
    direction) and key -> speed step."""
    bindings = BINDINGS if bindings is None else bindings
    motion = {fold(k): AXES[a] for a in AXES for k in bindings.get(a, ())}
    speed = {fold(k): SPEED_STEP[a] for a in SPEED_STEP for k in bindings.get(a, ())}
    return motion, speed


MANUAL_KEYS = keys_of()[0]
# A bare arrow is a Kitty press or a plain key (tmux forwards no events), so
# only a later repeat or release proves the terminal reports releases.
ARROWS = ('up', 'down', 'left', 'right')
# A terminal repeats only the newest key, so a held chord key goes quiet while
# another is down or after it is released. After a release, the keys still
# held are trusted for this long without a repeat; their release stops them.
CHORD_HOLD = 2.0
# A key reported held (Kitty) that only went quiet, with no release seen, is re-armed by its next
# repeat within this long: one link stall must not end the hold until the key is pressed again.
REARM_S = 2.0


class ManualState:
    """Held motion keys -> one (surge, sway, yaw) effort command.

    Once the terminal reports releases (Kitty protocol, also over SSH and through
    tmux's passthrough), held keys chord and each stops on its own release. One
    watchdog covers every held key, because a terminal repeats only the newest:
    expiring them separately would break diagonals.

    Until then (a tmux whose terminal can't pass the events, plain terminals) no
    release ever comes, so each key keeps its own deadline: the newest key lives
    while it repeats, and a key another key took over (its repeats stop) stays
    ``chord_hold`` after its last event, so recent presses on different axes
    combine and expire separately. On one axis the newest key replaces the older.

    ``bindings`` is action -> keys (BINDINGS). ``chords`` and ``command`` are what a display shows.
    """

    def __init__(self, amplitude=0.25, repeat_timeout=0.15, initial_timeout=0.65,
                 yaw_scale=0.5, chord_hold=0.5, bindings=None):
        if not math.isfinite(amplitude) or not 0 <= amplitude <= 1:
            raise ValueError('amplitude must be within [0, 1]')
        if not all(math.isfinite(v) and 0 < v <= 2
                   for v in (repeat_timeout, initial_timeout, chord_hold)):
            raise ValueError('deadman timeouts must be within (0, 2] seconds')
        if not math.isfinite(yaw_scale) or not 0 < yaw_scale <= 1:
            raise ValueError('yaw_scale must be within (0, 1]')
        self.amplitude = amplitude
        self.repeat_timeout = repeat_timeout
        self.initial_timeout = initial_timeout
        self.chord_hold = chord_hold
        self.yaw_scale = yaw_scale  # yaw is far more aggressive at the same effort
        self.keys, self.speed = keys_of(bindings)
        self.deadline = 0.0
        self.held = set()   # keys that will report their release
        self.recent = {}    # keys that won't: name -> (last event, deadline)
        self.newest = None  # the recent key whose repeats keep it alive
        self.lapsed = {}    # held keys the watchdog stopped without a release: name -> when
        self.chords = False
        self.focused = True
        self.command = (0.0, 0.0, 0.0)

    def press(self, name, now):
        """A key with no release to come: the newest. Older keys on its axis go, the others keep
        ``chord_hold`` from their last event (their repeats stopped when this one began)."""
        axis = self.keys[name][0]
        for other, (seen, deadline) in list(self.recent.items()):
            if self.keys[other][0] == axis:
                del self.recent[other]
            else:
                self.recent[other] = (seen, max(deadline, seen + self.chord_hold))
        self.recent[name] = (now, now + self.initial_timeout)
        self.newest = name

    def update(self, key, now):
        if now >= self.deadline:
            self.lapsed.update(dict.fromkeys(self.held, self.deadline))
            self.held.clear()
        self.recent = {k: v for k, v in self.recent.items() if now < v[1]}
        kind, _, name = (key or '').partition(':')
        if kind not in ('press', 'repeat', 'release'):
            kind, name = 'plain', key
        name = fold(name)
        if kind in ('repeat', 'release') or (kind == 'press' and name not in ARROWS):
            self.chords = True
            if name in self.recent and kind != 'release':
                self.held.add(name)  # a bare arrow that turned out to report events
                del self.recent[name]
        if name == 'focus-out':
            self.focused = False
        elif name == 'focus-in':
            self.focused = True
        if name in ('space', 'focus-out', 'focus-in') and kind != 'release':
            self.held.clear()
            self.lapsed.clear()   # a stop is never undone by the repeats that follow it
            self.recent.clear()
        elif kind == 'release':
            self.lapsed.pop(name, None)
            was_held = name in self.held
            self.held.discard(name)
            self.recent.pop(name, None)
            # The OS may restart its initial repeat delay for the remaining key.
            # Only a held motion key's release counts: other releases (+/-,
            # never-held keys) must not keep a key whose release was lost alive.
            if was_held and self.held:
                self.deadline = now + CHORD_HOLD
            elif was_held and self.recent:
                self.recent = {k: (seen, max(deadline, now + self.initial_timeout))
                               for k, (seen, deadline) in self.recent.items()}
        elif name in self.speed:
            self.amplitude = round(max(0.0, min(1.0, self.amplitude + self.speed[name])), 2)
        elif name in self.keys and self.focused:
            if name in self.held:
                self.deadline = now + self.repeat_timeout
            elif name in self.recent and name == self.newest:
                self.recent[name] = (now, now + self.repeat_timeout)   # its repeat
            elif kind == 'repeat' and now - self.lapsed.pop(name, -math.inf) <= REARM_S:
                self.held.add(name)   # still held: a repeat, and no release since it went quiet
                self.deadline = now + self.repeat_timeout
            elif kind != 'repeat':  # never re-arm a stopped key from repeats alone
                if self.chords and kind == 'press':
                    self.held.add(name)
                    self.deadline = now + self.initial_timeout
                else:
                    self.press(name, now)
        # Aliases do not double an axis; opposites cancel.
        axes = {axis: set() for axis in ('surge', 'sway', 'yaw')}
        for active in self.held | set(self.recent):
            axis, direction = self.keys[active]
            axes[axis].add(direction)
        x, y, yaw = (sum(axes[axis]) for axis in ('surge', 'sway', 'yaw'))
        scale = max(1.0, math.hypot(x, y))
        self.command = (x / scale * self.amplitude, y / scale * self.amplitude,
                        yaw * self.amplitude * self.yaw_scale)
        return self.command


DRAIN_S = 0.1   # s after the last publish: the queued batch is read before the next, bounded by time, not count
MIN_DRAIN_S = 0.02   # s; after a long stall the budget is spent already, yet a stale burst must still be read whole
EXIT_KEYS = ('x', 'm', 'escape', 'press:x', 'press:m', 'press:escape')
STOP_KEYS = ('space', 'press:space', 'focus-out')


def drain(keys, state, key, clock=time.monotonic, since=None):
    """Feed ``key`` and everything already queued to ``state``: the command to publish, None to exit.

    One coherent command per batch: no intermediate command goes out between a press and its
    release, so a burst of stale repeats after a link stall meets its release first. Stop keys end
    the batch at once; past the deadline the rest stays queued for the next batch. ``since`` is
    the last publish: the deadline counts DRAIN_S from it, so the wait for ``key`` is inside the
    bound and the gap between commands stays under the deadman; MIN_DRAIN_S keeps a stalled
    link's burst whole. Without ``since`` it counts from now.
    """
    now = clock()
    deadline = now + DRAIN_S if since is None else max(since + DRAIN_S, now + MIN_DRAIN_S)
    while True:
        if key and key.lower() in EXIT_KEYS:
            return None
        command = state.update(key, clock())
        if key is None or key in STOP_KEYS or clock() >= deadline:
            return command
        key = keys.read(timeout=0)
        if key is None:
            return command
