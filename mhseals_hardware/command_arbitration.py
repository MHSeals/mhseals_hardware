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

import math

MANUAL, HOLD, GUIDED, OTHER = 'MANUAL', 'HOLD', 'GUIDED', 'OTHER'
# ArduRover mode numbers: the FCU's MODE1..6 values.
MODE_NAMES = {0: MANUAL, 4: HOLD, 15: GUIDED}
# Upper edges (microseconds) of MODE1..5; MODE6 is everything above.
MODE_BANDS = (1231, 1361, 1491, 1621, 1750)

RC_CAUSES = ('rc_switch', 'rc_lost', 'rc_stop', 'rc_stick')


def mode_from_pwm(pwm, modes=(0, 0, 0, 4, 0, 15)):
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

    def __init__(self, mode_ch=8, modes=(0, 0, 0, 4, 0, 15), stop_ch=6,
                 rc_scale=(0.5, 0.5, 0.3), rc_max_age=0.3, cmd_timeout=0.5,
                 stick_takeover=200, stick_time=0.2, stick_release=0.5,
                 ramp=1.0):
        self.mode_ch, self.modes, self.stop_ch = mode_ch, tuple(modes), stop_ch
        self._check_layout()
        self.rc_scale = tuple(rc_scale)
        self.rc_max_age, self.cmd_timeout = rc_max_age, cmd_timeout
        self.stick_takeover, self.stick_time = stick_takeover, stick_time
        self.stick_release, self.ramp = stick_release, ramp

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
        self._ramp_pending = False    # a human or hold held the output
        self._ramp_start = None
        self._since = None            # when the duplicate was first seen

    @classmethod
    def from_config(cls, config):
        """Build from the ``command_mux`` section of hardware.yaml."""
        return cls(**(config or {}))

    def _check_layout(self):
        """Reject a transmitter layout that cannot be decoded."""
        channels = (self.mode_ch, self.stop_ch)
        if not all(type(ch) is int and ch >= 1 for ch in channels):
            raise ValueError('mode_ch and stop_ch must be channel numbers >= 1')
        if len(self.modes) != len(MODE_BANDS) + 1 or not all(
                type(mode) is int for mode in self.modes):
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
        """Store an RC frame (empty channels mean no receiver)."""
        was = self._channel(self.channels, self.stop_ch)
        self.channels, self.rc_time = tuple(channels), now
        stop = self._channel(self.channels, self.stop_ch)
        if (self.valid(now) and was is not None and stop is not None
                and was >= 1500 > stop):
            self._hold('rc_stop', 'rc', now)       # stop switch falling edge
        self._advance(now)

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
            if held is None or held['token'] != token:
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
        held = self.holds.get(cause)
        since = held['since'] if held else now
        self.holds[cause] = {'cause': cause, 'by': by, 'since': since,
                             'token': token}
        self._ramp_pending = True     # auto resumes through the ramp

    def _advance(self, now):
        """Fold elapsed time and the newest inputs into the hold set."""
        valid = self.valid(now)
        mode = self.mode if valid else None
        if valid and mode != self._prev_mode:
            if mode == GUIDED:
                # Moving the switch to GUIDED releases only the RC causes.
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

        if source in ('rc', 'manual'):
            self._ramp_pending = True
        if source == 'auto':
            if self._ramp_pending:
                self._ramp_start, self._ramp_pending = now, False
            if self._ramp_start is not None and self.ramp > 0:   # ramp <= 0: no ramp
                scale = min(1.0, (now - self._ramp_start) / self.ramp)
                cmd = tuple(v * scale for v in cmd)
        return source, cmd

    def state(self, now):
        """What ``~/state`` publishes: input, rc decode and the hold set."""
        source, _ = self.select(now)
        holds = sorted(self.holds.values(), key=lambda h: h['since'])
        if self.duplicate:
            holds.append({'cause': 'duplicate', 'by': 'mux', 'since': self._since,
                          'token': None})
        return {'input': source,
                'rc': {'valid': self.valid(now),
                       'mode': self.mode if self.valid(now) else None,
                       'latched': self.latched},
                # The token is a secret of the holder: only its presence shows.
                'holds': [{**{k: v for k, v in h.items() if k != 'token'},
                           'has_token': h['token'] is not None}
                          for h in holds],
                'last_release': self.last_release}
