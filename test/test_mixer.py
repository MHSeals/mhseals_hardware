"""Tests for the planar thruster mixer."""

from mhseals_hardware.thruster_serial_node import mix_thrusters


def test_surge_order():
    assert mix_thrusters(1.0, 0.0, 0.0) == [1.0, 1.0, -1.0, -1.0]


def test_sway_order():
    assert mix_thrusters(0.0, 1.0, 0.0) == [1.0, -1.0, -1.0, 1.0]


def test_yaw_order():
    assert mix_thrusters(0.0, 0.0, 1.0) == [0.5, -0.5, 0.5, -0.5]


def test_combined_command_is_desaturated():
    outputs = mix_thrusters(1.0, 1.0, 1.0)
    assert max(abs(output) for output in outputs) == 1.0
