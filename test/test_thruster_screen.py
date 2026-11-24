"""The thrusters screen's pure parts: the strength bar and the profile save (mapping, rewrite, diff)."""

import pytest
import yaml

from mhseals_hardware.configuration import load_defaults
from mhseals_hardware.thruster_screen import diff, effort_bar, profile_mapping, profile_text


def test_the_bar_is_centred_on_neutral_and_fills_toward_the_thrust():
    assert effort_bar(1500, 4) == '    │    '
    assert effort_bar(1700, 4) == '    │██  '
    assert effort_bar(1100, 4) == '████│    '
    assert len({len(effort_bar(pulse)) for pulse in (1000, 1100, 1450, 1500, 1900, 2500)}) == 1


PROFILE = '''# Blastoise
extends: roboboat
# Thruster wiring.
thrusters: {channel_map: [1, 3, 2, 4]}
hull: {shape: catamaran}   # kept
'''


def test_save_rewrites_only_thrusters_and_drops_values_equal_to_the_base():
    base = load_defaults()
    start = dict(base, channel_map=[1, 3, 2, 4])
    current = dict(start, channel_map=[1, 2, 3, 4], reversed=[True, False, False, False],
                   pwm_channels=[0, 1, 0, 0])
    mapping = profile_mapping({'channel_map': [1, 3, 2, 4]}, start, current, base)
    assert mapping == {'reversed': [True, False, False, False], 'pwm_channels': [0, 1, 0, 0]}
    new = profile_text(PROFILE, mapping)
    assert new.splitlines()[:3] == PROFILE.splitlines()[:3]
    assert new.splitlines()[-1] == 'hull: {shape: catamaran}   # kept'
    assert yaml.safe_load(new)['thrusters'] == mapping
    lines = diff(PROFILE, new, 'profile.yaml').splitlines()
    assert [line for line in lines if line[:1] in '+-' and line[:3] not in ('+++', '---')] == [
        '-thrusters: {channel_map: [1, 3, 2, 4]}',
        '+thrusters: {reversed: [true, false, false, false], pwm_channels: [0, 1, 0, 0]}']


def test_a_long_or_missing_thrusters_block_and_an_empty_one():
    chips = [f'/sys/devices/platform/{n}.pwm/pwm/pwmchip*' for n in 'abcd']
    new = profile_text(PROFILE, {'pwm_chips': chips})
    assert yaml.safe_load(new)['thrusters'] == {'pwm_chips': chips} and 'hull' in new
    assert profile_text(new, {}) == PROFILE.replace('thrusters: {channel_map: [1, 3, 2, 4]}\n', '')
    assert yaml.safe_load(profile_text('extends: x', {'reversed': [True] * 4}))['thrusters']


def test_a_rewrite_that_would_touch_other_keys_refuses():
    with pytest.raises(ValueError):
        profile_text('thrusters: {}\n  # odd\nhull: 1\nthrusters: 2\n', {'channel_map': [1, 2, 3, 4]})
