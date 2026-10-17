"""Shared command and CLI helpers, without ROS or hardware."""

from mhseals_hardware.configuration import parse_csv
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, NEUTRAL_PWM, axis_command, effort_to_pulse,
)


def test_axis_commands_and_pulse_conversion():
    assert axis_command() == (0.0, 0.0, 0.0)
    assert axis_command('sway', -0.5) == (0.0, -0.5, 0.0)
    assert axis_command((0.1, 0.2, 0.3)) == (0.1, 0.2, 0.3)
    assert effort_to_pulse(0) == NEUTRAL_PWM
    assert effort_to_pulse(0.15) == 1560
    assert (effort_to_pulse(5), effort_to_pulse(-5)) == (MAX_PWM, MIN_PWM)


def test_parse_csv():
    assert parse_csv(' a, b ') == ('a', 'b')
    assert parse_csv('1,2,3,4', int) == (1, 2, 3, 4)

