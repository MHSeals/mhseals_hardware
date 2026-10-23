"""The calibrated velocity law: model inversion, drift observer, anti-windup."""
import math

import numpy
import pytest

from mhseals_hardware.configuration import load_defaults
from mhseals_hardware.velocity_law import (
    CURVE_KEYS, Plant, VelocityLaw, rotate,
)

GAIN = ((0.6, 0.0, 0.0), (0.0, 0.4, 0.0), (0.05, 0.0, 0.3))


def mapping(matrix):
    axes = ('surge', 'sway', 'yaw')
    return {row: dict(zip(axes, values)) for row, values in zip(axes, matrix)}


def config(gain=GAIN, decoupling=None):
    characterization = dict(velocity_gain=mapping(gain),
                            motion_limits={'velocity': dict(surge=0.6, sway=0.4, yaw=0.3)})
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
    assert effort == pytest.approx((0.5, 0.0, 0.0))  # the diagonal only; cross terms are the mixer's


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
        assert law.step((0.6, 0.0, 0.0), now)[0] == pytest.approx(1.0)
        now += 0.05
    assert law.integral[0] < 0.1


def test_stale_odometry_gives_zero_effort():
    law = VelocityLaw(Plant(config()))
    assert law.step((0.3, 0, 0), 0.0) == (0.0, 0.0, 0.0)  # never observed
    law.observe((0, 0, 0), 0.0, 0.0)
    assert law.step((0.3, 0, 0), 0.1)[0] > 0
    assert law.step((0.3, 0, 0), 1.0) == (0.0, 0.0, 0.0)


def test_non_finite_odometry_is_dropped_and_does_not_latch_the_law():
    law = VelocityLaw(Plant(config()))
    law.observe((0, 0, 0), 0.0, 0.0)
    law.observe((math.nan, 0, 0), 0.0, 0.05)
    law.observe((0, 0, 0), math.nan, 0.1)
    law.step((0.3, 0, 0), 0.1)
    law.observe((0, 0, 0), 0.0, 0.15)
    assert numpy.all(numpy.isfinite(law.drift)) and numpy.all(numpy.isfinite(law.integral))
    assert law.step((0.3, 0, 0), 0.15)[0] > 0
    assert law.step((0.3, 0, 0), 2.0) == (0.0, 0.0, 0.0)  # the dropped samples age out
    law.observe((0, 0, 0), 0.0, 2.0)
    assert law.step((0.3, 0, 0), 2.05)[0] > 0


def test_non_finite_command_is_neutralised_and_does_not_latch_the_integral():
    law = VelocityLaw(Plant(config()))
    law.observe((0, 0, 0), 0.0, 0.0)
    law.step((0.3, 0, 0), 0.05)
    law.observe((0, 0, 0), 0.0, 0.1)
    assert law.step((math.nan, math.inf, -math.inf), 0.1) is not None
    assert numpy.all(numpy.isfinite(law.integral))
    law.observe((0, 0, 0), 0.0, 0.15)
    assert law.step((0.3, 0, 0), 0.15)[0] > 0


def test_linear_fallback_uses_the_diagonal_of_the_fitted_gain():
    # Cross terms in the fit are ignored; each axis keeps its own gain.
    plant = Plant(config())
    assert [plant.speed(axis, 1, 1.0) for axis in range(3)] == pytest.approx(
        [GAIN[0][0], GAIN[1][1], GAIN[2][2]])


def test_model_error_under_way_is_not_taken_for_current():
    # The real boat is 15 % weaker than the fit says, and the water is still.
    boat = Boat(gain=numpy.array(GAIN) * 0.85)
    law = VelocityLaw(Plant(config()))
    run(law, boat, (0.4, 0.0, 0.0), 30)
    assert numpy.hypot(*law.drift) < 0.01
    assert boat.velocity[0] == pytest.approx(0.4, abs=0.03)  # feedback still gets there


def curved(**overrides):
    """A config whose surge+ is a squared-drag curve and the other directions are linear."""
    curves = {key: dict(d1=0.0, d2=1.0 / 0.4 ** 2) for key in CURVE_KEYS}
    curves['surge+'] = dict(d1=1.2, d2=1.5)
    curves['surge-'] = dict(d1=1.0, d2=2.0)
    curves.update(overrides)
    result = config()
    result['characterization']['speed_curves'] = curves
    return result


def test_speed_curves_invert_and_round_trip_in_both_signs():
    plant = Plant(curved())
    for axis, speeds in ((0, (0.0, 0.1, 0.35, 0.6)), (1, (0.1, 0.4)), (2, (0.05, 0.3))):
        for speed in speeds:
            for sign in (1, -1):
                assert plant.speed(axis, sign, plant.effort(axis, sign * speed)) == pytest.approx(
                    sign * speed)
    # The two surge directions differ: 0.3 m/s forward costs less than backward.
    assert plant.effort(0, 0.3) == pytest.approx(math.sqrt(1.2 * 0.3 + 1.5 * 0.09))
    assert plant.effort(0, -0.3) == pytest.approx(-math.sqrt(1.0 * 0.3 + 2.0 * 0.09))
    assert plant.speed(0, -1, 0.0) == 0.0


def test_feedforward_follows_the_curve():
    law = VelocityLaw(Plant(curved()), kp=(0, 0, 0), ki=(0, 0, 0), drift_time_constant=0)
    law.observe((0, 0, 0), 0.0, 0.0)
    assert law.step((0.3, 0.0, 0.0), 0.0)[0] == pytest.approx(math.sqrt(1.2 * 0.3 + 1.5 * 0.09))


def test_missing_curves_fall_back_to_the_velocity_gain_diagonal():
    plant = Plant(config())
    for axis, gain in enumerate((0.6, 0.4, 0.3)):
        for sign in (1, -1):
            assert plant.speed(axis, sign, sign * 0.5) == pytest.approx(sign * 0.5 * gain)
            assert plant.effort(axis, sign * 0.1) == pytest.approx(sign * 0.1 / gain)
    # One curve given, the rest still linear.
    mixed = config()
    mixed['characterization']['speed_curves'] = {'sway-': dict(d1=0.0, d2=9.0)}
    assert Plant(mixed).effort(1, -0.1) == pytest.approx(-0.3)
    assert Plant(mixed).effort(1, 0.1) == pytest.approx(0.1 / 0.4)


def test_non_monotone_curves_are_refused():
    # Effort falls with speed before the 0.6 m/s surge limit: d1 + 2 d2 v < 0.
    with pytest.raises(ValueError, match='surge\\+.*monotone'):
        Plant(curved(**{'surge+': dict(d1=1.0, d2=-1.0)}))
    with pytest.raises(ValueError, match='sway-.*monotone'):
        Plant(curved(**{'sway-': dict(d1=-0.1, d2=1.0)}))
    # A pure-quadratic curve is monotone and inverts as |e| / sqrt(d2).
    plant = Plant(curved(**{'surge+': dict(d1=0.0, d2=2.0)}))
    assert plant.speed(0, 1, 0.5) == pytest.approx(0.5 / math.sqrt(2.0))
    assert plant.speed(0, 1, 0.0) == 0.0
    with pytest.raises(ValueError, match='monotone'):
        Plant(curved(**{'surge+': dict(d1=0.0, d2=0.0)}))
    # A negative d2 that stays rising inside the limit is fine.
    Plant(curved(**{'surge+': dict(d1=1.0, d2=-0.5)}))


def test_curves_need_velocity_gain_or_all_six_directions():
    bare = curved()
    del bare['characterization']['velocity_gain']
    assert Plant(bare).effort(0, 0.3) > 0
    del bare['characterization']['speed_curves']['yaw-']
    with pytest.raises(ValueError, match='required'):
        Plant(bare)
