"""Tests for the planar thruster mixer and canonical output mapping."""

import pytest

from mhseals_hardware.keyboard import decode_key
from mhseals_hardware.thruster_mixer import (
    channel_map_from_observations,
    map_channels,
    mix_thrusters,
    validate_channel_map,
    validate_mixer,
)


def test_surge_order():
    assert mix_thrusters(1.0, 0.0, 0.0) == [-1.0, -1.0, 1.0, 1.0]


def test_sway_order():
    assert mix_thrusters(0.0, 1.0, 0.0) == [1.0, -1.0, -1.0, 1.0]


def test_yaw_order():
    assert mix_thrusters(0.0, 0.0, 1.0) == [1.0, -1.0, 1.0, -1.0]


def test_combined_command_is_desaturated():
    outputs = mix_thrusters(1.0, 1.0, 1.0)
    assert max(abs(output) for output in outputs) == 1.0


def test_custom_matrix_is_used():
    mixer = validate_mixer(range(12))
    assert mix_thrusters(1.0, 0.0, 0.0, mixer) == [
        0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0]


def test_channel_map_reorders_canonical_values():
    channel_map = validate_channel_map([2, 4, 1, 3])
    assert map_channels(['FL', 'FR', 'RR', 'RL'], channel_map) == [
        'RR', 'FL', 'RL', 'FR']


def test_identification_observations_are_inverted():
    observations = {1: 'rr', 2: 'fl', 3: 'rl', 4: 'fr'}
    assert channel_map_from_observations(observations) == (2, 4, 1, 3)


@pytest.mark.parametrize('values', ([1, 2, 3], [1, 1, 3, 4]))
def test_invalid_channel_map_is_rejected(values):
    with pytest.raises(ValueError):
        validate_channel_map(values)


@pytest.mark.parametrize('values', ([0.0] * 11, [0.0] * 11 + [float('inf')]))
def test_invalid_matrix_is_rejected(values):
    with pytest.raises(ValueError):
        validate_mixer(values)


@pytest.mark.parametrize(('sequence', 'expected'), (
    (b'\x1b[A', 'up'),
    (b'\x1bOB', 'down'),
    (b'\x1b[C', 'right'),
    (b'\r', 'enter'),
    (b'1', '1'),
))
def test_terminal_keys_are_decoded(sequence, expected):
    assert decode_key(sequence) == expected


def test_pulses_flip_reversed_thrusters_and_follow_the_channel_map():
    import pytest
    from mhseals_hardware.thruster_mixer import (
        THRUSTER_MIXER, thruster_pulses, validate_reversed,
    )
    surge = thruster_pulses((0.25, 0, 0), THRUSTER_MIXER, (1, 2, 3, 4))
    assert surge == [1400, 1400, 1600, 1600]
    flipped = thruster_pulses((0.25, 0, 0), THRUSTER_MIXER, (2, 1, 3, 4),
                              (True, False, False, False))
    assert flipped == [1400, 1600, 1600, 1600]  # FL now on output 2, flipped to 1600
    with pytest.raises(ValueError):
        validate_reversed([True, False, False])
    with pytest.raises(ValueError):
        validate_reversed([1, 0, 0, 0])


def test_decoupling_is_applied_before_the_mix():
    import pytest
    from mhseals_hardware.thruster_mixer import (
        IDENTITY, THRUSTER_MIXER, decouple, thruster_pulses, validate_decoupling,
    )
    assert decouple((0.3, -0.2, 0.1)) == (0.3, -0.2, 0.1)
    plain = thruster_pulses((0.25, 0, 0), THRUSTER_MIXER, (1, 2, 3, 4))
    assert thruster_pulses((0.25, 0, 0), THRUSTER_MIXER, (1, 2, 3, 4), decoupling=IDENTITY) == plain
    # Surge also asks for a little counter-yaw: FL and RR lose, FR and RL gain.
    corrected = validate_decoupling([1, 0, 0, 0, 1, 0, -0.2, 0, 1])
    assert decouple((0.25, 0, 0), corrected) == pytest.approx((0.25, 0, -0.05))
    assert thruster_pulses((0.25, 0, 0), THRUSTER_MIXER, (1, 2, 3, 4),
                           decoupling=corrected) == [1380, 1420, 1580, 1620]
    for bad in ([1] * 8, [float('nan')] + [0] * 8):
        with pytest.raises(ValueError):
            validate_decoupling(bad)


def test_negative_decoupling_takes_the_column_of_an_axis_commanded_below_zero():
    from mhseals_hardware.thruster_mixer import decouple
    # Yaw trim only while moving forward or left: positive surge/sway columns carry it.
    positive = ((1, 0, 0), (0, 1, 0), (-.1, .2, 1))
    negative = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    assert decouple((.8, 0, 0), positive, negative) == pytest.approx((.8, 0, -.08))
    assert decouple((-.8, 0, 0), positive, negative) == pytest.approx((-.8, 0, 0))
    assert decouple((0, .5, 0), positive, negative) == pytest.approx((0, .5, .1))
    assert decouple((0, -.5, .3), positive, negative) == pytest.approx((0, -.5, .3))
    assert decouple((.8, -.5, 0), positive, None) == decouple((.8, -.5, 0), positive)


def test_huge_finite_commands_saturate_instead_of_overflowing_to_nan():
    from mhseals_hardware.thruster_mixer import THRUSTER_MIXER, thruster_pulses
    for command in ((1e308, 0, 1e308), (1e308, -1e308, 0), (0, 0, -1e308)):
        pulses = thruster_pulses(command, THRUSTER_MIXER, (1, 2, 3, 4))
        assert all(isinstance(p, int) and 1100 <= p <= 1900 for p in pulses)
    assert thruster_pulses((1e308, 0, 0), THRUSTER_MIXER, (1, 2, 3, 4)) == \
        thruster_pulses((1, 0, 0), THRUSTER_MIXER, (1, 2, 3, 4))


def _wrench(efforts, ratio):
    """Planar wrench of an X layout matching THRUSTER_MIXER, thrust e|e|, reverse weaker."""
    return _wrench_of([e * abs(e) * (ratio if e < 0 else 1.0) for e in efforts])


def _wrench_of(thrusts):
    import math
    # Unit thrust direction (x, y) and position for FL, FR, RR, RL, positive effort.
    layout = (((-1, 1), (0.27, 0.34)), ((-1, -1), (0.27, -0.34)),
              ((1, -1), (-0.27, -0.34)), ((1, 1), (-0.27, 0.34)))
    total = [0.0, 0.0, 0.0]
    for thrust, ((dx, dy), (px, py)) in zip(thrusts, layout):
        force = thrust / math.sqrt(2)
        total[0] += force * dx
        total[1] += force * dy
        total[2] += force * (px * dy - py * dx)
    return total


@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('value', (-1.0, -0.4, 0.25, 1.0))
def test_single_axis_output_is_the_linear_mix(axis, value):
    command = [0.0, 0.0, 0.0]
    command[axis] = value
    expected = [sum(gain * part for gain, part in zip(row, command)) for row in
                __import__('mhseals_hardware.thruster_mixer', fromlist=['x']).THRUSTER_MIXER]
    assert mix_thrusters(*command, reverse_ratio=(0.78,) * 4) == pytest.approx(expected)


@pytest.mark.parametrize('surge,sway', [(0.42, 0.42), (0.52, 0.3), (-0.3, 0.5), (0.2, -0.6)])
def test_combined_command_is_the_sum_of_single_axis_wrenches(surge, sway):
    import math
    ratio = 0.78
    alone = [_wrench(mix_thrusters(surge, 0, 0, reverse_ratio=(ratio,) * 4), ratio),
             _wrench(mix_thrusters(0, sway, 0, reverse_ratio=(ratio,) * 4), ratio)]
    expected = [a + b for a, b in zip(*alone)]
    wrench = _wrench(mix_thrusters(surge, sway, 0, reverse_ratio=(ratio,) * 4), ratio)
    assert wrench == pytest.approx(expected, abs=1e-9)
    assert abs(wrench[2]) < 1e-9  # a diagonal does not turn the boat
    # Summing efforts (the old mixer) turns the boat and bends the direction.
    linear = _wrench([sum(g * c for g, c in zip(row, (surge, sway, 0))) for row in
                      ((-1, 1, 1), (-1, -1, -1), (1, -1, 1), (1, 1, -1))], ratio)
    assert abs(linear[2]) > 0.01 or abs(math.atan2(linear[1], linear[0]) -
                                         math.atan2(expected[1], expected[0])) > 0.05


def test_saturated_combined_command_keeps_the_wrench_direction():
    import math
    ratio = 0.78
    outputs = mix_thrusters(1.0, 1.0, 0.5, reverse_ratio=(ratio,) * 4)
    assert max(abs(output) for output in outputs) == pytest.approx(1.0)
    alone = [_wrench(mix_thrusters(*command, reverse_ratio=(ratio,) * 4), ratio)
             for command in ((1, 0, 0), (0, 1, 0), (0, 0, 0.5))]
    expected = [sum(parts) for parts in zip(*alone)]
    wrench = _wrench(outputs, ratio)
    scale = wrench[0] / expected[0]
    assert 0 < scale < 1 and wrench == pytest.approx([scale * v for v in expected])


def test_reverse_ratio_follows_a_flipped_thruster():
    # Mounted backwards, a thruster's weak side (below 1500 us) is its positive effort.
    from mhseals_hardware.thruster_mixer import THRUSTER_MIXER, thruster_pulses
    ratio = 0.78
    pulses = thruster_pulses((0.4, 0.3, 0), THRUSTER_MIXER, (1, 2, 3, 4), (True,) * 4,
                             reverse_ratio=ratio)
    # Pulse side sets the strength; the flip sets the canonical direction.
    thrust = [-(p - 1500) * abs(p - 1500) / 400 ** 2 * (ratio if p < 1500 else 1.0)
              for p in pulses]
    assert abs(_wrench_of(thrust)[2]) < 0.01


def test_reverse_ratio_is_validated():
    from mhseals_hardware.thruster_mixer import validate_reverse_ratio
    assert validate_reverse_ratio(1) == 1.0
    for bad in (0.0, -0.5, 1.5, True, float('nan'), '0.8'):
        with pytest.raises(ValueError):
            validate_reverse_ratio(bad)
