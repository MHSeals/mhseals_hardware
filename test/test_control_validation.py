"""Shared domains, including actual mux construction and effort-only loading."""
from array import array

import pytest

from mhseals_hardware.command_arbitration import Arbitration
from mhseals_hardware.configuration import load_defaults
from mhseals_hardware.control_validation import validate_controller


def values():
    return dict(kp=[.8]*3, ki=[.6]*3, integral_limit=[.3]*3,
                drift_time_constant=5., odom_timeout=.3,
                drift_command_limit=.1, command_timeout=.5, feedback=True)


@pytest.mark.parametrize('key,bad', [
    ('kp', [True, 0, 0]), ('ki', [float('nan')]*3),
    ('integral_limit', [-1, 0, 0]), ('kp', [1]),
    ('odom_timeout', 0), ('command_timeout', True),
    ('drift_time_constant', -1), ('drift_command_limit', float('inf')),
    ('feedback', 1)])
def test_controller_domains(key, bad):
    with pytest.raises(ValueError):
        validate_controller(dict(values(), **{key: bad}))


@pytest.mark.parametrize('key,bad', [
    ('mode_ch', 1.5), ('stop_ch', True), ('modes', [0, 0, 0, 0, 0, -1]),
    ('rc_scale', [float('nan')]*3), ('rc_scale', [1]),
    ('rc_scale', [True, 0, 0]), ('ramp', -1), ('cmd_timeout', 0),
    ('rc_max_age', True), ('stick_time', float('inf'))])
def test_mux_domains(key, bad):
    with pytest.raises(ValueError):
        Arbitration.from_config({key: bad})


def test_valid_domains_keep_effort_only_defaults_valid():
    assert validate_controller(values())['feedback'] is True
    assert load_defaults()['characterization'] == {}
    assert Arbitration(ramp=0).ramp == 0


@pytest.mark.parametrize('bad', [[1.5, 2, 3, 4], [True, 2, 3, 4]])
def test_channel_indexes_are_not_coerced(bad):
    from mhseals_hardware.thruster_mixer import validate_channel_map
    with pytest.raises(ValueError):
        validate_channel_map(bad)


@pytest.mark.parametrize('key', ['command_timeout', 'manual_repeat_timeout',
                                'manual_initial_timeout', 'manual_amplitude', 'manual_yaw_scale'])
def test_effort_timing_bool_rejected(key):
    from mhseals_hardware.configuration import validate_config
    with pytest.raises(ValueError):
        validate_config(dict(load_defaults(), **{key: True}))


@pytest.mark.parametrize('value', [True, 1.5])
def test_stick_pwm_threshold_is_integer(value):
    with pytest.raises(ValueError):
        Arbitration(stick_takeover=value)


@pytest.mark.parametrize('bad', [array('d', [-1., 0., 0.]),
                                array('d', [float('inf'), 0., 0.]),
                                array('d', [1., 2.])])
def test_ros_array_domains_are_not_weakened(bad):
    with pytest.raises(ValueError):
        validate_controller(dict(values(), kp=bad))


def test_ros_startup_array_domain_is_supported():
    assert validate_controller(dict(values(), kp=array('d', [.8]*3)))['kp'][0] == .8
