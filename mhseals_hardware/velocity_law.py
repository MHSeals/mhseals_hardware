"""The calibrated velocity control law, free of ROS so it can be tested directly.

The characterization (mhseals_testing) fits the body velocity as

    v = b_m + R(psi)^T d_w + G e

with ``b_m`` a body-fixed mechanical bias (ESC trim), ``d_w`` the world-fixed
drift of current and wind, ``G`` the gain and ``e`` the effort. Mechanical
cross-coupling is cancelled ahead of the mixer by the ``decoupling`` matrix C,
so the fitted ``G`` is already the plant this law sees. Only its diagonal is kept,
as the linear fallback for the per-direction speed curves. The drift changes from day
to day, so it is not stored: it is estimated here, in the world frame, and
rotated into the body every cycle so the estimate survives turns.
"""
import math

import numpy

from mhseals_hardware.thruster_mixer import AXES


CURVE_KEYS = tuple(axis + sign for axis in AXES for sign in '+-')


def gain_matrix(mapping):
    """Turn a saved {axis: {axis: gain}} mapping into a 3x3 array."""
    return numpy.array([[float(mapping[row][name]) for name in AXES] for row in AXES])


def rotate(vector, yaw):
    """Rotate a planar (x, y) vector by ``yaw`` (body to world for +yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vector[0] - s * vector[1], s * vector[0] + c * vector[1])


class Plant:
    """What the controller knows about the boat, read from the hardware config.

    Each axis and sign has a speed curve in wrench units: the effort ``e`` that
    holds speed ``v`` satisfies ``e |e| = d1 v + d2 v |v|`` (thrust goes with
    e|e|, so the drag fits it without a negative term). ``characterization.speed_curves``
    holds them as ``{'surge+': {'d1': .., 'd2': ..}, ..., 'yaw-': ..}``; a
    direction without one falls back to the straight line of the
    ``velocity_gain`` diagonal, so a boat characterized before the curves
    existed keeps working.
    """

    def __init__(self, config):
        characterization = config.get('characterization', {})
        curves = characterization.get('speed_curves') or {}
        missing = [key for key in CURVE_KEYS if key not in curves]
        if missing and not characterization.get('velocity_gain'):
            raise ValueError('characterization.speed_curves or velocity_gain is required')
        diagonal = self._diagonal(characterization) if missing else ()
        bias = characterization.get('mechanical_bias', {})
        self.bias = numpy.array([float(bias.get(axis, 0.0)) for axis in AXES])
        limits = characterization.get('motion_limits', {}).get('velocity', {})
        self.limits = numpy.array([float(limits.get(axis, 0.0)) for axis in AXES])
        if any(limit <= 0 for limit in self.limits):
            raise ValueError('characterization motion velocity limits are required')
        self.curves = {}
        for index, axis in enumerate(AXES):
            for key in (axis + '+', axis + '-'):
                if key in curves:
                    d1, d2 = float(curves[key]['d1']), float(curves[key]['d2'])
                elif diagonal[index] != 0:
                    d1, d2 = 0.0, 1.0 / diagonal[index] ** 2  # e = v / gain
                else:
                    raise ValueError('velocity_gain is singular')
                # Effort must rise with speed up to the axis limit, else it can't be inverted.
                # d1 = 0 with d2 > 0 (pure quadratic) is monotone too; the fitter keeps d1 >= 0.
                if not (math.isfinite(d1) and math.isfinite(d2)) or (
                        d1 < 0 or d1 + 2 * d2 * self.limits[index] <= 0):
                    raise ValueError(f'speed curve {key} is not monotone up to the velocity limit')
                self.curves[key] = (d1, d2)

    @staticmethod
    def _diagonal(characterization):
        """The per-axis gains of ``velocity_gain``, the linear fallback for missing curves."""
        gain = gain_matrix(characterization['velocity_gain'])
        return numpy.diag(gain)

    def effort(self, axis, speed):
        """The effort that holds ``speed`` (m/s or rad/s) on ``axis`` (0 surge, 1 sway, 2 yaw)."""
        d1, d2 = self.curves[CURVE_KEYS[2 * axis + (speed < 0)]]
        wrench = d1 * speed + d2 * speed * abs(speed)
        return math.copysign(math.sqrt(abs(wrench)), wrench)

    def speed(self, axis, sign, effort):
        """The steady speed ``effort`` gives on ``axis``, signed.

        ``sign`` (+1 or -1) picks the curve when the effort is zero; otherwise
        the effort's own sign does.
        """
        sign = 1 if effort > 0 else -1 if effort < 0 else sign
        d1, d2 = self.curves[CURVE_KEYS[2 * axis + (sign < 0)]]
        magnitude = effort * effort
        if magnitude == 0:
            return 0.0
        # The positive root of d2 v^2 + d1 v = e^2, in the form that survives d2 -> 0.
        root = math.sqrt(max(0.0, d1 * d1 + 4 * d2 * magnitude))
        return sign * 2 * magnitude / (d1 + root)


class VelocityLaw:
    """Feed-forward through the model, a world-frame drift observer, and PI feedback.

    The observer learns only while asked to hold still (station keeping or idle);
    under way it keeps its estimate and the PI loop covers the rest.

    ``observe`` takes odometry (body velocity, heading); ``step`` turns a wanted
    body velocity into effort. Times are seconds on any monotonic clock.
    """

    def __init__(self, plant, kp=(0.8, 0.8, 0.6), ki=(0.6, 0.6, 0.4),
                 integral_limit=(0.3, 0.3, 0.3), drift_time_constant=5.0, odom_timeout=0.3,
                 drift_command_limit=0.1, drift_limit=0.5, drift_carry=0.02):
        self.plant = plant
        self.kp, self.ki = numpy.array(kp, float), numpy.array(ki, float)
        self.integral_limit = numpy.array(integral_limit, float)
        self.drift_time_constant = float(drift_time_constant)
        self.drift_command_limit = float(drift_command_limit)
        self.drift_limit = float(drift_limit)
        self.drift_carry = float(drift_carry)
        self.carried = numpy.zeros(2)  # world frame, m, since the boat last got under way
        self.odom_timeout = float(odom_timeout)
        self.drift = numpy.zeros(2)  # world frame (x east, y north in odom), m/s
        self.velocity = None
        self.yaw = 0.0
        self.odom_time = None
        self.reset()

    def reset(self):
        self.integral = numpy.zeros(3)
        self.effort = numpy.zeros(3)
        self.target = numpy.zeros(3)
        self.last_step = None

    def observe(self, velocity, yaw, now):
        """Odometry: update the drift estimate from what the model can't explain."""
        # A non-finite sample would poison the integral and drift for good: drop it,
        # so the last good odometry ages out through the staleness check instead.
        if not (numpy.all(numpy.isfinite(velocity)) and math.isfinite(yaw)):
            return
        dt = 0.0 if self.odom_time is None else min(0.2, max(0.0, now - self.odom_time))
        self.velocity, self.yaw, self.odom_time = numpy.array(velocity, float), float(yaw), now
        # Learn while asked to hold still (or idle): the effort holding the boat
        # then measures the current. Under way the residual is mostly the model's own
        # error (a straight-line gain on a squared thrust curve), which would be taken
        # for current and turned with the boat.
        under_way = numpy.any(numpy.abs(self.target) > self.drift_command_limit)
        if under_way:
            self.carried = numpy.zeros(2)
        saturated = numpy.any(numpy.abs(self.effort) >= 0.999)
        if self.drift_time_constant <= 0 or dt == 0 or under_way or saturated:
            return
        self.carried += dt * numpy.array(rotate(self.velocity[:2], self.yaw))
        expected = self.plant.bias + numpy.array([
            self.plant.speed(axis, 1, effort) for axis, effort in enumerate(self.effort)])
        residual = numpy.array(rotate((self.velocity - expected)[:2], self.yaw))
        before = math.hypot(*self.drift)
        alpha = dt / (self.drift_time_constant + dt)
        self.drift += alpha * (residual - self.drift)
        # Against a dock or a buoy the boat stops while the effort pushes on, and the
        # residual reads as a current pushing it off the contact; learning that would hold
        # the push and wind it up. A real current carries the boat along it before the
        # loop catches it, while the boat reached the contact moving the other way. So the
        # estimate grows only once the boat has been carried drift_carry along the residual
        # since it last got under way (displacement, not speed: the PI soon stops the boat);
        # it may always shrink or turn. drift_limit is the backstop.
        size = numpy.linalg.norm(residual)
        carried = size > 0 and self.carried @ residual / size > self.drift_carry
        limit = self.drift_limit if carried else min(before, self.drift_limit)
        speed = math.hypot(*self.drift)
        if speed > limit:
            self.drift *= limit / speed

    def stale(self, now):
        return self.odom_time is None or now - self.odom_time > self.odom_timeout

    def step(self, desired, now, feedback=True):
        """Wanted body velocity (m/s, rad/s) -> effort in [-1, 1] per axis."""
        if self.stale(now):
            self.reset()
            return (0.0, 0.0, 0.0)
        dt = 0.0 if self.last_step is None else min(0.2, max(0.0, now - self.last_step))
        self.last_step = now
        wanted = numpy.array(desired, float)
        wanted[~numpy.isfinite(wanted)] = 0.0  # a bad axis asks for rest; it must not reach the integral
        target = numpy.clip(wanted, -self.plant.limits, self.plant.limits)
        self.target = target
        drift_body = numpy.array([*rotate(self.drift, -self.yaw), 0.0])
        feedforward = numpy.array([self.plant.effort(axis, speed) for axis, speed in
                                   enumerate(target - self.plant.bias - drift_body)])
        error = target - self.velocity
        if feedback:
            unclipped = feedforward + self.kp * error + self.ki * self.integral
            # Conditional integration: an axis pinned at its limit in the
            # direction of its error stops integrating, so it can't wind up.
            pinned = ((unclipped >= 1.0) & (error > 0)) | ((unclipped <= -1.0) & (error < 0))
            self.integral = numpy.where(pinned, self.integral, numpy.clip(
                self.integral + error * dt, -self.integral_limit, self.integral_limit))
            effort = feedforward + self.kp * error + self.ki * self.integral
        else:
            effort = feedforward
        effort = numpy.where(numpy.isfinite(effort), numpy.clip(effort, -1.0, 1.0), 0.0)
        self.effort = effort
        return tuple(float(value) for value in effort)
