"""Pick which command drives the boat: rc > manual > hold > auto.

ROS-free and clock-injected: every call takes ``now`` in seconds, so a node
feeds it monotonic time and a test feeds it a fake clock.  The mux node only
forwards ``select(now)``; all the rules live here.

* rc: a valid RC frame and the mode switch at MANUAL.
* manual: ``/cmd_vel/manual`` (TUI keys) newer than ``cmd_timeout``.
* hold: any hold cause is set.  Output is zero until it is released.
* auto: ``/cmd_vel`` newer than ``cmd_timeout``.
"""

MANUAL, HOLD, GUIDED, OTHER = 'MANUAL', 'HOLD', 'GUIDED', 'OTHER'
# ArduRover mode numbers: the FCU's MODE1..6 values.
MODE_NAMES = {0: MANUAL, 4: HOLD, 15: GUIDED}
# Upper edges (microseconds) of MODE1..5; MODE6 is everything above.
MODE_BANDS = (1231, 1361, 1491, 1621, 1750)

RC_CAUSES = ('rc_switch', 'rc_lost', 'rc_stop')


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


class Arbitration:
    """The state machine behind ``command_mux``."""

    def __init__(self, mode_ch=8, modes=(0, 0, 0, 4, 0, 15), stop_ch=6,
                 rc_scale=(0.5, 0.5, 0.3), rc_max_age=0.3, cmd_timeout=0.5):
        self.mode_ch, self.modes, self.stop_ch = mode_ch, tuple(modes), stop_ch
        self.rc_scale = tuple(rc_scale)
        self.rc_max_age, self.cmd_timeout = rc_max_age, cmd_timeout

        self.channels, self.rc_time = (), None
        self.manual, self.manual_time = (0.0, 0.0, 0.0), None
        self.auto, self.auto_time = (0.0, 0.0, 0.0), None
        self.holds = {}               # cause -> {cause, by, since, token}
        self._prev_mode = None
        self._rc_active = False       # was rc driving at the last advance
        self._manual_fresh = False

    @classmethod
    def from_config(cls, config):
        """Build from the ``command_mux`` section of hardware.yaml."""
        return cls(**(config or {}))

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
        self.manual, self.manual_time = tuple(cmd), now
        self._advance(now)

    def update_auto(self, cmd, now):
        self.auto, self.auto_time = tuple(cmd), now

    def request(self, op, cause, by, now, token=None):
        """Apply a hold or release request; return ``(ok, reason)``."""
        self._advance(now)
        if op == 'hold':
            self._hold(cause, by, now, token)
            return True, ''
        if op != 'release':
            return False, 'unknown op'
        if token is not None:
            held = self.holds.get(cause)
            if held is None or held['token'] != token:
                return False, 'no hold with that token'
            del self.holds[cause]
            return True, ''
        if self.valid(now) and self.mode in (MANUAL, HOLD, OTHER):
            return False, 'mode switch is at %s' % self.mode
        self.holds.clear()
        return True, ''

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

    # State machine

    def _hold(self, cause, by, now, token=None):
        held = self.holds.get(cause)
        since = held['since'] if held else now
        self.holds[cause] = {'cause': cause, 'by': by, 'since': since,
                             'token': token}

    def _advance(self, now):
        """Fold elapsed time and the newest inputs into the hold set."""
        valid = self.valid(now)
        mode = self.mode if valid else None
        if valid and mode != self._prev_mode:
            if mode == GUIDED:
                # Moving the switch to GUIDED releases only the RC causes.
                for cause in RC_CAUSES:
                    self.holds.pop(cause, None)
            elif mode != MANUAL:
                self._hold('rc_switch', 'rc', now)
            self._prev_mode = mode

        rc = valid and mode == MANUAL
        if self._rc_active and not valid:
            self._hold('rc_lost', 'rc', now)
        self._rc_active = rc

        fresh = self._fresh(self.manual_time, now)
        if self._manual_fresh and not fresh:
            self._hold('keys', 'keys', now)
        self._manual_fresh = fresh
        return rc, fresh

    def _fresh(self, stamp, now):
        return stamp is not None and now - stamp <= self.cmd_timeout

    def select(self, now):
        """Return ``(source, (surge, sway, yaw))`` for this instant."""
        rc, manual = self._advance(now)
        zero = (0.0, 0.0, 0.0)
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
        return source, cmd

    def state(self, now):
        """What ``~/state`` publishes: input, rc decode and the hold set."""
        source, _ = self.select(now)
        return {'input': source,
                'rc': {'valid': self.valid(now),
                       'mode': self.mode if self.valid(now) else None},
                'holds': sorted(self.holds.values(), key=lambda h: h['since'])}
