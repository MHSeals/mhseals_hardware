"""Pick which command drives the boat: duplicate > rc > manual > stop > auto > idle.

ROS-free and clock-injected: every call takes ``now`` in seconds, so a node
feeds it monotonic time and a test feeds it a fake clock.  The mux node only
forwards ``select(now)``; all the rules live here.

The rule: a lost input goes neutral, and nothing stays locked afterwards. A lane that goes
stale (older than ``command_timeout``) or invalid just falls through to the next one, so
whoever next means to move the boat moves it at once. Only stops someone chooses hold:

* duplicate: another publisher drives the same output, so this one sends only zeros
  (``set_duplicate``) until it goes.
* rc: a valid RC frame and the mode switch at MANUAL, or a stick takeover in GUIDED.
* manual: ``topics.manual`` (keyboard_control) newer than ``command_timeout``.
* stop (input ``hold``): the RC mode switch at HOLD (or an unknown mode) holds while it stays
  there; the RC stop channel's falling edge holds until that switch goes back up.
* auto: ``topics.effort`` newer than ``command_timeout``, ramped in from zero over ``ramp``
  after a start, a stop, rc or manual.
"""

import json
import math
from collections import deque

MANUAL, HOLD, GUIDED, OTHER = 'MANUAL', 'HOLD', 'GUIDED', 'OTHER'
# ArduRover mode numbers: the FCU's MODE1..6 values.
MODE_NAMES = {0: MANUAL, 4: HOLD, 15: GUIDED}
# Upper edges (microseconds) of MODE1..5; MODE6 is everything above.
MODE_BANDS = (1231, 1361, 1491, 1621, 1750)

RC_CENTER = 1500                  # stick centre and stop-switch threshold, µs
RC_WINDOW = 50                    # frame intervals kept for rate and p95

MUX_SCHEMA = 'mhseals.mux/1'


def mux_input(text):
    """The ``input`` of a mux state message (the contract in command_mux), or None: unknown.

    A schema other than MUX_SCHEMA, a missing input or unreadable JSON is unknown, and each
    consumer fails safe on it.
    """
    try:
        state = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(state, dict) or state.get('schema') != MUX_SCHEMA:
        return None
    value = state.get('input')
    return value if isinstance(value, str) else None


def mode_from_pwm(pwm, modes):
    """Decode the mode switch exactly as the FCU does, from raw PWM."""
    slot = sum(pwm >= edge for edge in MODE_BANDS)
    return MODE_NAMES.get(modes[slot], OTHER)


def stick(pwm, center=RC_CENTER, deadband=100, half_range=500):
    """One channel as [-1, 1] with a center deadband."""
    offset = pwm - center
    if abs(offset) <= deadband:
        return 0.0
    return max(-1.0, min(1.0, offset / half_range))


# The tunables a ``command_mux`` section may set (Arbitration.__init__ arguments).
# ``command_timeout`` is the one top-level watchdog the driver and the controller also read.
INIT_KEYS = ('mode_ch', 'modes', 'stop_ch', 'rc_scale', 'rc_max_age', 'command_timeout',
             'stick_takeover', 'stick_time', 'stick_release', 'ramp')


def mux_section(config):
    """The ``command_mux`` section of a hardware config plus its top-level ``command_timeout``."""
    if 'command_timeout' in config['command_mux']:
        raise ValueError('command_timeout is top-level, not a command_mux key')
    return {**config['command_mux'], 'command_timeout': config['command_timeout']}


class Arbitration:
    """The state machine behind ``command_mux``."""

    def __init__(self, mode_ch, modes, stop_ch, rc_scale, rc_max_age, command_timeout,
                 stick_takeover, stick_time, stick_release, ramp):
        # Every value comes from the command_mux section of default.yaml.
        self.mode_ch, self.modes, self.stop_ch = mode_ch, tuple(modes), stop_ch
        self._check_layout()
        self.rc_scale = tuple(rc_scale)
        self.rc_max_age, self.command_timeout = rc_max_age, command_timeout
        if type(stick_takeover) is not int or stick_takeover < 0:
            raise ValueError('stick_takeover must be a nonnegative integer PWM offset')
        self.stick_takeover, self.stick_time = stick_takeover, stick_time
        self.stick_release, self.ramp = stick_release, ramp
        from mhseals_hardware.control_validation import nonnegative
        if len(self.rc_scale) != 3:
            raise ValueError('rc_scale needs three numbers')
        for value in self.rc_scale:
            nonnegative(value, 'rc_scale')
        for name in ('rc_max_age', 'command_timeout', 'stick_takeover', 'stick_time',
                     'stick_release', 'ramp'):
            nonnegative(getattr(self, name), name,
                        positive=name in ('rc_max_age', 'command_timeout'))

        self.channels, self.rc_time = (), None
        self.manual, self.manual_time = (0.0, 0.0, 0.0), None
        self.auto, self.auto_time = (0.0, 0.0, 0.0), None
        self.rejected = 0             # non-finite commands dropped
        self.stop_since = None        # the stop channel's falling edge, until it goes back up
        self.duplicate = False        # a second mux publishes the same output
        self.latched = False          # stick takeover while in GUIDED
        self._deflected = None        # since when a stick is past the takeover
        self._centered = None         # since when all sticks are centered
        self._prev_mode = None
        self._switch_since = None     # when the mode switch last changed
        # Start pending: the first auto after a (re)start always ramps in.
        self._ramp_pending = True
        self._ramp_start = None
        self._since = None            # when the duplicate was first seen
        self._intervals = deque(maxlen=RC_WINDOW)   # seconds between rc frames
        self._input = None            # last selected source
        self._input_since = None      # when it became the source
        self._cause = 'idle'          # why it became the source

    @classmethod
    def from_config(cls, config):
        """Build from ``mux_section``: the ``command_mux`` section plus ``command_timeout``."""
        return cls(**(config or {}))

    def _check_layout(self):
        """Reject a transmitter layout that cannot be decoded."""
        channels = (self.mode_ch, self.stop_ch)
        if not all(type(ch) is int and ch >= 1 for ch in channels):
            raise ValueError('mode_ch and stop_ch must be channel numbers >= 1')
        if len(self.modes) != len(MODE_BANDS) + 1 or not all(
                type(mode) is int and mode >= 0 for mode in self.modes):
            raise ValueError('modes must be %d FCU mode numbers' % (len(MODE_BANDS) + 1))

    # Inputs

    def update_rc(self, channels, now):
        """Store an RC frame (empty channels mean no receiver); True on a stop edge.

        The thruster driver reuses this rule to cut the MOSFET on the same edge.
        """
        was = self._channel(self.channels, self.stop_ch)
        if self.rc_time is not None and now > self.rc_time:
            self._intervals.append(now - self.rc_time)
        self.channels, self.rc_time = tuple(channels), now
        stop = self._channel(self.channels, self.stop_ch)
        valid = self.valid(now)
        edge = valid and was is not None and stop is not None and was >= RC_CENTER > stop
        if edge and self.stop_since is None:
            self.stop_since = now                  # stop switch falling edge
        elif valid and stop is not None and stop >= RC_CENTER:
            self.stop_since = None                 # the switch back up clears it
        self._advance(now)
        return edge

    def update_manual(self, cmd, now):
        """Store a manual command; a NaN or inf one is counted and dropped."""
        if not self._finite(cmd):
            return
        self.manual, self.manual_time = tuple(cmd), now

    def update_auto(self, cmd, now):
        """Store an auto command; a NaN or inf one is counted and dropped."""
        if not self._finite(cmd):
            return
        self.auto, self.auto_time = tuple(cmd), now

    def set_duplicate(self, duplicate, now):
        """Mark another publisher on the output; output stays zero while set."""
        if duplicate and not self.duplicate:
            self._ramp_pending = True
        self.duplicate = duplicate
        self._since = now if duplicate else None

    def _finite(self, cmd):
        """True for a finite command; otherwise count it (the stamp stays old)."""
        if all(math.isfinite(v) for v in cmd):
            return True
        self.rejected += 1
        return False

    # Decoding

    def _channel(self, channels, number):
        return channels[number - 1] if len(channels) >= number else None

    def valid(self, now):
        """At least 4 channels, a young frame, sane sticks and mode channel.

        An out-of-range value (0, 65535 for an unused channel, or a wrapped
        signed-uint16) invalidates the frame instead of reading as a deflection.
        """
        mode_pwm = self._channel(self.channels, self.mode_ch)
        return (len(self.channels) >= 4 and self.rc_time is not None
                and now - self.rc_time <= self.rc_max_age
                and mode_pwm is not None and 900 <= mode_pwm <= 2100
                and all(900 <= self.channels[i] <= 2100 for i in (0, 1, 3)))

    @property
    def mode(self):
        pwm = self._channel(self.channels, self.mode_ch)
        return None if pwm is None else mode_from_pwm(pwm, self.modes)

    def _sticks(self):
        ch = self.channels
        return (stick(ch[1]), -stick(ch[0]), -stick(ch[3]))

    def _past_takeover(self):
        return any(abs(self.channels[i] - RC_CENTER) > self.stick_takeover
                   for i in (0, 1, 3))

    # State machine

    def stops(self, now):
        """The chosen stops in force now: [(cause, since)], oldest first."""
        held = []
        if self.duplicate:
            held.append(('duplicate', self._since))
        if self.stop_since is not None:
            held.append(('stop_channel', self.stop_since))
        if self.valid(now) and self.mode not in (MANUAL, GUIDED):
            held.append(('rc_switch', self._switch_since))
        return held

    def _advance(self, now):
        """Fold elapsed time and the newest RC frame into the takeover latch; True when rc drives."""
        valid = self.valid(now)
        mode = self.mode if valid else None
        if mode != self._prev_mode:
            self._switch_since, self._prev_mode = now, mode
        if not valid or mode != GUIDED:
            self.latched = False
            self._deflected = self._centered = None
        if valid and mode == GUIDED:
            self._track_sticks(now)
        return valid and (mode == MANUAL or self.latched)

    def _track_sticks(self, now):
        if self._past_takeover():
            self._centered = None
            self._deflected = now if self._deflected is None else self._deflected
            if now - self._deflected >= self.stick_time:
                self.latched = True
            return
        self._deflected = None
        if self.latched and any(self._sticks()):
            self._centered = None     # still steering inside the takeover band
        elif self.latched:
            self._centered = now if self._centered is None else self._centered
            if now - self._centered >= self.stick_release:
                # Sticks let go: the takeover ends and the next lane drives.
                self.latched, self._centered = False, None

    def _fresh(self, stamp, now):
        return stamp is not None and now - stamp <= self.command_timeout

    def select(self, now):
        """Return ``(source, (surge, sway, yaw))`` for this instant."""
        rc = self._advance(now)
        zero = (0.0, 0.0, 0.0)
        stops = self.stops(now)
        if self.duplicate:
            cmd, source = zero, 'hold'
        elif rc:
            cmd = tuple(v * s for v, s in zip(self._sticks(), self.rc_scale))
            source = 'rc'
        elif self._fresh(self.manual_time, now):
            cmd, source = self.manual, 'manual'
        elif stops:
            cmd, source = zero, 'hold'
        elif self._fresh(self.auto_time, now):
            cmd, source = self.auto, 'auto'
        else:
            cmd, source = zero, 'idle'

        self._track_input(source, stops, now)
        if source in ('hold', 'rc', 'manual'):
            self._ramp_pending = True
        if source == 'auto':
            if self._ramp_pending:
                self._ramp_start, self._ramp_pending = now, False
            if self._ramp_start is not None and self.ramp > 0:   # ramp <= 0: no ramp
                scale = min(1.0, (now - self._ramp_start) / self.ramp)
                cmd = tuple(v * scale for v in cmd)
        return source, cmd

    def _track_input(self, source, stops, now):
        """Remember when and why the source changed (for ``~/state``)."""
        if source == self._input:
            return
        self._input, self._input_since = source, now
        if source == 'rc':
            self._cause = 'rc_stick' if self.latched else 'rc_switch'
        elif source == 'hold':
            self._cause = stops[0][0]
        else:
            self._cause = source if source in ('manual', 'auto') else 'idle'

    def _rc_rate(self):
        """``(rate_hz, interval_p95_s)`` over the last frames, or Nones."""
        if not self._intervals:
            return None, None
        ordered = sorted(self._intervals)
        p95 = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]
        return len(ordered) / sum(ordered), p95

    @staticmethod
    def _age(stamp, now):
        return None if stamp is None else round(now - stamp, 3)

    def state(self, now, wall_ns=None, pipeline_ms=None):
        """What ``~/state`` publishes (docs/authority.md 2.3), minus schema/seq/t_ns.

        Times here are the node's monotonic seconds, which no other host can
        read; given ``wall_ns`` (wall clock at ``now``) each ``since`` is also
        published as ``since_ns`` on the wall clock.
        """
        source, _ = self.select(now)

        def wall(stamp):
            return None if wall_ns is None or stamp is None else int(
                wall_ns - (now - stamp) * 1e9)

        stops = self.stops(now)
        valid = self.valid(now)
        rate, p95 = self._rc_rate()
        pwm = self._channel(self.channels, self.mode_ch)
        hold = None
        if stops:
            hold = {'by': 'mux' if stops[0][0] == 'duplicate' else 'rc', 'cause': stops[0][0],
                    'since_ns': wall(stops[0][1])}
        return {'input': source, 'since_ns': wall(self._input_since),
                'cause': self._cause, 'hold': hold,
                'rc': {'valid': valid, 'mode': self.mode if valid else None,
                       'latched': self.latched, 'stick': self.latched,
                       'pwm_mode': pwm, 'rate_hz': rate and round(rate, 1),
                       'interval_p95_s': p95 and round(p95, 3),
                       'age_s': self._age(self.rc_time, now),
                       'stop_edge': self.stop_since is not None},
                'manual': {'age_s': self._age(self.manual_time, now), 'node': None},
                'auto': {'age_s': self._age(self.auto_time, now),
                         'active': self._fresh(self.auto_time, now)},
                'rejected': self.rejected, 'pipeline_ms': pipeline_ms,
                'holds': [{'cause': cause, 'since_ns': wall(since)} for cause, since in stops]}
