"""Pick which command drives the boat: rc > manual > hold > auto.

ROS-free and clock-injected: every call takes ``now`` in seconds, so a node
feeds it monotonic time and a test feeds it a fake clock.  The mux node only
forwards ``select(now)``; all the rules live here.

* rc: a valid RC frame and the mode switch at MANUAL, or a stick takeover.
* manual: ``/cmd_vel/manual`` (TUI keys) newer than ``cmd_timeout``.
* hold: any hold cause is set.  Output is zero until it is released.
* duplicate: another publisher drives the same output, so this one sends
  only zeros (``set_duplicate``) and cannot be released.
* auto: ``/cmd_vel`` newer than ``cmd_timeout``, ramped in over ``ramp``.
"""

import hashlib
import math
from collections import deque

MANUAL, HOLD, GUIDED, OTHER = 'MANUAL', 'HOLD', 'GUIDED', 'OTHER'
# ArduRover mode numbers: the FCU's MODE1..6 values.
MODE_NAMES = {0: MANUAL, 4: HOLD, 15: GUIDED}
# Upper edges (microseconds) of MODE1..5; MODE6 is everything above.
MODE_BANDS = (1231, 1361, 1491, 1621, 1750)

# Hold causes renamed for the ``~/state`` ``cause`` field (authority.md 2.3).
STATE_CAUSES = {'rc_stop': 'stop_channel', 'keys': 'manual_end'}
RC_WINDOW = 50                    # frame intervals kept for rate and p95

RC_CAUSES = ('rc_switch', 'rc_lost', 'rc_stop', 'rc_stick')


def token_hash(token):
    """The stored form of a hold token: its hash, or None for no token."""
    if token is None:
        return None
    return hashlib.sha256(str(token).encode()).hexdigest()


def mode_from_pwm(pwm, modes):
    """Decode the mode switch exactly as the FCU does, from raw PWM."""
    slot = sum(pwm >= edge for edge in MODE_BANDS)
    return MODE_NAMES.get(modes[slot], OTHER)


def stick(pwm, center=1500, deadband=100, half_range=500):
    """One channel as [-1, 1] with a center deadband."""
    offset = pwm - center
    if abs(offset) <= deadband:
        return 0.0
    return max(-1.0, min(1.0, offset / half_range))


# The tunables a ``command_mux`` section may set (Arbitration.__init__ arguments).
INIT_KEYS = ('mode_ch', 'modes', 'stop_ch', 'rc_scale', 'rc_max_age', 'cmd_timeout',
             'stick_takeover', 'stick_time', 'stick_release', 'ramp')


class Arbitration:
    """The state machine behind ``command_mux``."""

    def __init__(self, mode_ch, modes, stop_ch, rc_scale, rc_max_age, cmd_timeout,
                 stick_takeover, stick_time, stick_release, ramp):
        # Every value comes from the command_mux section of default.yaml.
        self.mode_ch, self.modes, self.stop_ch = mode_ch, tuple(modes), stop_ch
        self._check_layout()
        self.rc_scale = tuple(rc_scale)
        self.rc_max_age, self.cmd_timeout = rc_max_age, cmd_timeout
        if type(stick_takeover) is not int or stick_takeover < 0:
            raise ValueError('stick_takeover must be a nonnegative integer PWM offset')
        self.stick_takeover, self.stick_time = stick_takeover, stick_time
        self.stick_release, self.ramp = stick_release, ramp
        from mhseals_hardware.control_validation import nonnegative
        if len(self.rc_scale) != 3:
            raise ValueError('rc_scale needs three numbers')
        for value in self.rc_scale:
            nonnegative(value, 'rc_scale')
        for name in ('rc_max_age', 'cmd_timeout', 'stick_takeover', 'stick_time',
                     'stick_release', 'ramp'):
            nonnegative(getattr(self, name), name,
                        positive=name in ('rc_max_age', 'cmd_timeout'))

        self.channels, self.rc_time = (), None
        self.manual, self.manual_time = (0.0, 0.0, 0.0), None
        self.auto, self.auto_time = (0.0, 0.0, 0.0), None
        self.rejected = 0             # non-finite commands dropped
        self.holds = {}               # cause -> {cause, by, since, token}
        self.last_release = None      # {by, cause, tokenless, cleared} of the newest
        self.duplicate = False        # a second mux publishes the same output
        self.latched = False          # stick takeover while in GUIDED
        self._deflected = None        # since when a stick is past the takeover
        self._centered = None         # since when all sticks are centered
        self._prev_mode = None
        self._rc_active = False       # was rc driving at the last advance
        self._manual_fresh = False
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
        """Build from the ``command_mux`` section of hardware.yaml."""
        if config and 'state_file' in config and (
                not isinstance(config['state_file'], str) or not config['state_file']):
            raise ValueError('state_file must be a nonempty path string')
        # ``state_file`` belongs to the node (hold persistence), not the state machine.
        return cls(**{k: v for k, v in (config or {}).items() if k != 'state_file'})

    def adopt_holds(self, holds, now):
        """Re-adopt holds saved by a previous mux process (see ``hold_store``)."""
        for cause, held in holds.items():
            self.holds[cause] = {**held, 'since': min(held['since'], now)}
        if self.holds:
            self._ramp_pending = True

    def _check_layout(self):
        """Reject a transmitter layout that cannot be decoded."""
        channels = (self.mode_ch, self.stop_ch)
        if not all(type(ch) is int and ch >= 1 for ch in channels):
            raise ValueError('mode_ch and stop_ch must be channel numbers >= 1')
        if len(self.modes) != len(MODE_BANDS) + 1 or not all(
                type(mode) is int and mode >= 0 for mode in self.modes):
            raise ValueError('modes must be %d FCU mode numbers' % (len(MODE_BANDS) + 1))

    def apply_config(self, section, now):
        """Apply a ``command_mux`` section only while nothing is being driven.

        Returns False (and changes nothing) in rc, manual or auto, so a remap
        never lands mid-drive; the caller keeps the section pending and retries.
        """
        fresh = self.from_config(section)
        if self.select(now)[0] not in ('idle', 'hold'):
            return False
        for key in INIT_KEYS:
            setattr(self, key, getattr(fresh, key))
        return True

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
        edge = (self.valid(now) and was is not None and stop is not None
                and was >= 1500 > stop)
        if edge:
            self._hold('rc_stop', 'rc', now)       # stop switch falling edge
        self._advance(now)
        return edge

    def update_manual(self, cmd, now):
        """Store a manual command; a NaN or inf one is counted and dropped."""
        if not self._finite(cmd):
            return
        self.manual, self.manual_time = tuple(cmd), now
        self._advance(now)

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

    def request(self, op, cause, by, now, token=None):
        """Apply a hold or release request; return ``(ok, reason)``."""
        self._advance(now)
        if op == 'hold':
            if not isinstance(cause, str) or not cause:
                return False, 'cause must be a non-empty string'
            self._hold(cause, by, now, token)
            return True, ''
        if op != 'release':
            return False, 'unknown op'
        if token is not None:
            held = self.holds.get(cause)
            if held is None or held['token'] != token_hash(token):
                return False, 'no hold with that token'
            del self.holds[cause]
            self._released(by, cause, False, 1)
            return True, ''
        if self.valid(now) and self.mode in (MANUAL, HOLD, OTHER):
            return False, 'mode switch is at %s' % self.mode
        self._released(by, cause, True, len(self.holds))
        self.holds.clear()
        return True, ''

    def _released(self, by, cause, tokenless, cleared):
        """Record who released, so ``~/state`` and the journal can name them."""
        self.last_release = {'by': by, 'cause': cause, 'tokenless': tokenless,
                             'cleared': cleared}

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
        return any(abs(self.channels[i] - 1500) > self.stick_takeover
                   for i in (0, 1, 3))

    # State machine

    def _hold(self, cause, by, now, token=None):
        # Placing a cause again restarts its since, so the controller sees a new hold,
        # except a repeat with the same token (a watchdog refreshing its own hold).
        held = self.holds.get(cause)
        stored = token_hash(token)
        same = held is not None and stored is not None and held['token'] == stored
        since = held['since'] if same else now
        self.holds[cause] = {'cause': cause, 'by': by, 'since': since,
                             'token': token_hash(token)}
        self._ramp_pending = True     # auto resumes through the ramp

    def _advance(self, now):
        """Fold elapsed time and the newest inputs into the hold set."""
        valid = self.valid(now)
        mode = self.mode if valid else None
        if valid and mode != self._prev_mode:
            if mode == GUIDED and self._prev_mode is not None:
                # Moving the switch to GUIDED releases only the RC causes; the
                # first frame after a restart is not a move, so adopted holds stay.
                for cause in RC_CAUSES:
                    self.holds.pop(cause, None)
            self._prev_mode = mode
        if valid and mode not in (MANUAL, GUIDED) and 'rc_switch' not in self.holds:
            # The switch is a level: a held position holds again after a
            # release (U during an RC dropout) without needing a new edge.
            self._hold('rc_switch', 'rc', now)
        if not valid or mode != GUIDED:
            self.latched = False
            self._deflected = self._centered = None
        if valid and mode == GUIDED:
            self._track_sticks(now)

        rc = valid and (mode == MANUAL or self.latched)
        if self._rc_active and not valid:
            self._hold('rc_lost', 'rc', now)
        self._rc_active = rc

        fresh = self._fresh(self.manual_time, now)
        if self._manual_fresh and not fresh:
            self._hold('keys', 'keys', now)
        self._manual_fresh = fresh
        return rc, fresh

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
                # Sticks let go: stay put until the human releases the hold.
                self.latched, self._centered = False, None
                self._hold('rc_stick', 'rc', now)

    def _fresh(self, stamp, now):
        return stamp is not None and now - stamp <= self.cmd_timeout

    def select(self, now):
        """Return ``(source, (surge, sway, yaw))`` for this instant."""
        rc, manual = self._advance(now)
        zero = (0.0, 0.0, 0.0)
        if self.duplicate:
            self._ramp_pending = True
            self._track_input('hold', now)
            return 'hold', zero
        if rc:
            cmd = tuple(v * s for v, s in zip(self._sticks(), self.rc_scale))
            source = 'rc'
        elif manual:
            cmd, source = self.manual, 'manual'
        elif self.holds:
            cmd, source = zero, 'hold'
        elif self._fresh(self.auto_time, now):
            cmd, source = self.auto, 'auto'
        else:
            cmd, source = zero, 'idle'

        self._track_input(source, now)
        if source in ('rc', 'manual'):
            self._ramp_pending = True
        if source == 'auto':
            if self._ramp_pending:
                self._ramp_start, self._ramp_pending = now, False
            if self._ramp_start is not None and self.ramp > 0:   # ramp <= 0: no ramp
                scale = min(1.0, (now - self._ramp_start) / self.ramp)
                cmd = tuple(v * scale for v in cmd)
        return source, cmd

    def _track_input(self, source, now):
        """Remember when and why the source changed (for ``~/state``)."""
        if source == self._input:
            return
        previous, self._input, self._input_since = self._input, source, now
        if source == 'rc':
            self._cause = 'rc_stick' if self.latched else 'rc_switch'
        elif source == 'hold':
            first = min(self.holds.values(), key=lambda h: h['since'], default=None)
            cause = first['cause'] if first else 'duplicate'
            self._cause = STATE_CAUSES.get(cause, cause)
        elif source == 'auto' and previous == 'hold':
            self._cause = 'release'
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
        """What ``~/state`` publishes (authority.md 2.3), minus schema/seq/t_ns.

        Times here are the node's monotonic seconds, which no other host can
        read; given ``wall_ns`` (wall clock at ``now``) each ``since`` is also
        published as ``since_ns`` on the wall clock.
        """
        source, _ = self.select(now)

        def wall(stamp):
            return None if wall_ns is None or stamp is None else int(
                wall_ns - (now - stamp) * 1e9)

        holds = sorted(self.holds.values(), key=lambda h: h['since'])
        if self.duplicate:
            holds.append({'cause': 'duplicate', 'by': 'mux', 'since': self._since,
                          'token': None})
        valid = self.valid(now)
        rate, p95 = self._rc_rate()
        pwm = self._channel(self.channels, self.mode_ch)
        hold = None
        if holds:
            hold = {k: holds[0][k] for k in ('by', 'cause')}
            hold['since_ns'] = wall(holds[0]['since'])
        return {'input': source, 'since_ns': wall(self._input_since),
                'cause': self._cause, 'hold': hold,
                'rc': {'valid': valid, 'mode': self.mode if valid else None,
                       'latched': self.latched, 'stick': self.latched,
                       'pwm_mode': pwm, 'rate_hz': rate and round(rate, 1),
                       'interval_p95_s': p95 and round(p95, 3),
                       'age_s': self._age(self.rc_time, now),
                       'stop_edge': 'rc_stop' in self.holds},
                'manual': {'age_s': self._age(self.manual_time, now), 'node': None},
                'auto': {'age_s': self._age(self.auto_time, now),
                         'active': self._fresh(self.auto_time, now)},
                'rejected': self.rejected, 'pipeline_ms': pipeline_ms,
                # The token is a secret of the holder: only its presence shows.
                'holds': [{**{k: v for k, v in h.items() if k != 'token'},
                           'since_ns': wall(h['since']),
                           'has_token': h['token'] is not None}
                          for h in holds],
                'last_release': self.last_release}
