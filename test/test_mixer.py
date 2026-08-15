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
