"""The calibrated velocity control law, free of ROS so it can be tested directly.

The characterization (mhseals_testing) fits the body velocity as

    v = b_m + R(psi)^T d_w + G e

with ``b_m`` a body-fixed mechanical bias (ESC trim), ``d_w`` the world-fixed
drift of current and wind, ``G`` the gain and ``e`` the effort. Mechanical
cross-coupling is cancelled ahead of the mixer by the ``decoupling`` matrix C,
so the plant this law sees is ``G_eff = G_raw C``. The drift changes from day
to day, so it is not stored: it is estimated here, in the world frame, and
rotated into the body every cycle so the estimate survives turns.
"""
import math

import numpy

from mhseals_hardware.thruster_mixer import AXES, IDENTITY, validate_decoupling


def gain_matrix(mapping):
    """Turn a saved {axis: {axis: gain}} mapping into a 3x3 array."""
    return numpy.array([[float(mapping[row][name]) for name in AXES] for row in AXES])


def raw_gain(characterization):
    """The boat's own gain, with the decoupling active during the fit divided out.

    A fit measures G_fit = G_raw C_fit, so G_raw = G_fit C_fit^-1.
    """
    fitted = gain_matrix(characterization['velocity_gain'])
    used = numpy.array(validate_decoupling(
        characterization.get('fitted_with_decoupling', sum(IDENTITY, ()))))
    return fitted @ numpy.linalg.inv(used)


def decoupling_for(raw):
    """C = G_raw^-1 diag(G_raw): each axis keeps its strength, cross terms cancel."""
    return numpy.linalg.inv(raw) @ numpy.diag(numpy.diag(raw))


def rotate(vector, yaw):
    """Rotate a planar (x, y) vector by ``yaw`` (body to world for +yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vector[0] - s * vector[1], s * vector[0] + c * vector[1])


class Plant:
    """What the controller knows about the boat, read from the hardware config."""

    def __init__(self, config):
        characterization = config.get('characterization', {})
        if not characterization.get('velocity_gain'):
            raise ValueError('characterization.velocity_gain is required')
        decoupling = numpy.array(validate_decoupling(config.get('decoupling', sum(IDENTITY, ()))))
        try:
            self.gain = raw_gain(characterization) @ decoupling
            self.gain_inv = numpy.linalg.inv(self.gain)
        except numpy.linalg.LinAlgError as error:
            raise ValueError('velocity_gain is singular') from error
        bias = characterization.get('mechanical_bias', {})
        self.bias = numpy.array([float(bias.get(axis, 0.0)) for axis in AXES])
        limits = characterization.get('motion_limits', {}).get('velocity', {})
        self.limits = numpy.array([float(limits.get(axis, 0.0)) for axis in AXES])
        if any(limit <= 0 for limit in self.limits):
            raise ValueError('characterization motion velocity limits are required')


class VelocityLaw:
    """Feed-forward through the model, a world-frame drift observer, and PI feedback.

    ``observe`` takes odometry (body velocity, heading); ``step`` turns a wanted
    body velocity into effort. Times are seconds on any monotonic clock.
    """

    def __init__(self, plant, kp=(0.1, 0.1, 0.05), ki=(0.1, 0.1, 0.05),
                 integral_limit=(0.1, 0.1, 0.1), drift_time_constant=5.0, odom_timeout=0.3):
        self.plant = plant
        self.kp, self.ki = numpy.array(kp, float), numpy.array(ki, float)
        self.integral_limit = numpy.array(integral_limit, float)
        self.drift_time_constant = float(drift_time_constant)
        self.odom_timeout = float(odom_timeout)
        self.drift = numpy.zeros(2)  # world frame (x east, y north in odom), m/s
        self.velocity = None
        self.yaw = 0.0
        self.odom_time = None
        self.reset()

    def reset(self):
        self.integral = numpy.zeros(3)
        self.effort = numpy.zeros(3)
        self.last_step = None

    def observe(self, velocity, yaw, now):
        """Odometry: update the drift estimate from what the model can't explain."""
        dt = 0.0 if self.odom_time is None else min(0.2, max(0.0, now - self.odom_time))
        self.velocity, self.yaw, self.odom_time = numpy.array(velocity, float), float(yaw), now
        saturated = numpy.any(numpy.abs(self.effort) >= 0.999)
        if self.drift_time_constant <= 0 or dt == 0 or saturated:
            return
        expected = self.plant.bias + self.plant.gain @ self.effort
        residual = rotate((self.velocity - expected)[:2], self.yaw)
        alpha = dt / (self.drift_time_constant + dt)
        self.drift += alpha * (numpy.array(residual) - self.drift)

    def stale(self, now):
        return self.odom_time is None or now - self.odom_time > self.odom_timeout

    def step(self, desired, now, feedback=True):
        """Wanted body velocity (m/s, rad/s) -> effort in [-1, 1] per axis."""
        if self.stale(now):
            self.reset()
            return (0.0, 0.0, 0.0)
        dt = 0.0 if self.last_step is None else min(0.2, max(0.0, now - self.last_step))
        self.last_step = now
        target = numpy.clip(numpy.array(desired, float), -self.plant.limits, self.plant.limits)
        drift_body = numpy.array([*rotate(self.drift, -self.yaw), 0.0])
        feedforward = self.plant.gain_inv @ (target - self.plant.bias - drift_body)
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
