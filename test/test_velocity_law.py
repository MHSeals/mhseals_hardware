"""The calibrated velocity law: model inversion, drift observer, anti-windup."""
import math

import numpy
import pytest

from mhseals_hardware.configuration import load_defaults
from mhseals_hardware.velocity_law import (
    Plant, VelocityLaw, decoupling_for, raw_gain, rotate,
)

GAIN = ((0.6, 0.0, 0.0), (0.0, 0.4, 0.0), (0.05, 0.0, 0.3))


def mapping(matrix):
    axes = ('surge', 'sway', 'yaw')
    return {row: dict(zip(axes, values)) for row, values in zip(axes, matrix)}


def config(gain=GAIN, decoupling=None, fitted_with=None):
    characterization = dict(velocity_gain=mapping(gain),
                            motion_limits={'velocity': dict(surge=0.6, sway=0.4, yaw=0.3)})
    if fitted_with is not None:
        characterization['fitted_with_decoupling'] = fitted_with
    result = dict(load_defaults(), characterization=characterization)
    if decoupling is not None:
        result['decoupling'] = decoupling
    return result


class Boat:
    """A first-order boat in a world-fixed current: v = R^T d + G e, lagging."""

    def __init__(self, current=(0.0, 0.0), gain=GAIN):
        self.current, self.gain = numpy.array(current), numpy.array(gain)
        self.velocity, self.yaw = numpy.zeros(3), 0.0

    def advance(self, effort, dt=0.05, lag=0.5):
        steady = self.gain @ numpy.array(effort)
        steady[:2] += rotate(self.current, -self.yaw)
        self.velocity += (steady - self.velocity) * dt / lag
        self.yaw += self.velocity[2] * dt


def run(law, boat, desired, seconds, now=0.0, dt=0.05):
    for _ in range(int(seconds / dt)):
        law.observe(boat.velocity, boat.yaw, now)
        boat.advance(law.step(desired(now) if callable(desired) else desired, now), dt)
        now += dt
    return now


def test_feedforward_inverts_the_model():
    law = VelocityLaw(Plant(config()), kp=(0, 0, 0), ki=(0, 0, 0), drift_time_constant=0)
    law.observe((0, 0, 0), 0.0, 0.0)
    effort = law.step((0.3, 0.0, 0.0), 0.0)
    assert effort == pytest.approx((0.5, 0.0, -0.05 * 0.5 / 0.3))


def test_drift_observer_learns_a_world_current_and_keeps_it_through_a_turn():
    boat = Boat(current=(0.0, 0.15))
    law = VelocityLaw(Plant(config()))
    now = run(law, boat, (0.0, 0.0, 0.0), 30)
    assert law.drift == pytest.approx((0.0, 0.15), abs=0.01)
    # Turn half way round: the estimate is in the world frame, so it still holds.
    now = run(law, boat, (0.0, 0.0, 0.2), math.pi / 0.2, now)
    run(law, boat, (0.0, 0.0, 0.0), 5, now)
    assert law.drift == pytest.approx((0.0, 0.15), abs=0.02)
    assert abs(boat.velocity[0]) < 0.03 and abs(boat.velocity[1]) < 0.03  # held still


def test_integral_does_not_wind_up_against_saturation():
    law = VelocityLaw(Plant(config()), kp=(0, 0, 0), ki=(1, 1, 1), integral_limit=(5, 5, 5),
                      drift_time_constant=0)
    now = 0.0
    for _ in range(100):  # the boat never moves: effort pins at +1 on surge
        law.observe((0, 0, 0), 0.0, now)
        assert law.step((0.6, 0.0, 0.0), now)[0] == 1.0
        now += 0.05
    assert law.integral[0] < 0.1


def test_stale_odometry_gives_zero_effort():
    law = VelocityLaw(Plant(config()))
    assert law.step((0.3, 0, 0), 0.0) == (0.0, 0.0, 0.0)  # never observed
    law.observe((0, 0, 0), 0.0, 0.0)
    assert law.step((0.3, 0, 0), 0.1)[0] > 0
    assert law.step((0.3, 0, 0), 1.0) == (0.0, 0.0, 0.0)


def test_decoupling_cancels_coupling_and_composes_across_refits():
    raw = numpy.array(GAIN)
    c = decoupling_for(raw)
    assert raw @ c == pytest.approx(numpy.diag(numpy.diag(raw)))
    # A refit with that decoupling active measures G_raw C; dividing C out recovers G_raw.
    refit = dict(velocity_gain=mapping(raw @ c), fitted_with_decoupling=c.flatten().tolist())
    assert raw_gain(refit) == pytest.approx(raw)
    # The controller's plant through the decoupling is diagonal: no double correction.
    plant = Plant(config(decoupling=c.flatten().tolist()))
    assert plant.gain == pytest.approx(numpy.diag(numpy.diag(raw)))
