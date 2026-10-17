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
