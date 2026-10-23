"""Config reload must agree with telemetry and the manual Twist output path."""

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist

from mhseals_hardware.configuration import load_defaults, save_config
from mhseals_hardware import outputs as outputs_module, thruster_pwm_node


def test_live_correction_updates_parameters_and_manual_pulses(tmp_path, monkeypatch):
    class Outputs:
        def open(self):
            return self

        def close(self):
            pass

        def set_pulse_widths(self, pulses):
            self.pulses = pulses

    # Always fake outputs, regardless of host, configuration or ROS environment.
    outputs = Outputs()
    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda *args, **kwargs: outputs)
    config = dict(load_defaults(), channel_map=[1, 3, 2, 4], reversed=[False] * 4)
    path = tmp_path / 'hardware.yaml'
    save_config(config, path)
    rclpy.init(args=['--ros-args', '-r', '__node:=config_reload_test',
                    '-p', f'config_file:={path}'])
    node = None
    try:
        node = thruster_pwm_node.ThrusterPWMNode()
        correction = [1., 0., 0., 0., 1., 0., -.1, 0., 1.]
        save_config(dict(config, decoupling=correction), path)
        node.watch.mtime = None
        node.reload_callback()
        assert list(node.get_parameter('decoupling').value) == correction
        message = Twist()
        message.linear.x = .5
        node.cmd_vel_callback(message)
        assert outputs.pulses == [1280, 1680, 1320, 1720]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


class Recorder:
    """Fake outputs that log their lifecycle, and can refuse to open."""

    def __init__(self, config, events, fail_on=None):
        self.channels = list(config['pwm_channels'])
        self.events, self.fail_on = events, fail_on

    def open(self):
        if self.channels == self.fail_on:
            raise OSError('busy')
        self.events.append(('open', self.channels))
        return self

    def neutral(self):
        self.events.append(('neutral', self.channels))

    def close(self, unexport=False):
        self.events.append(('close', self.channels, unexport))

    def set_pulse_widths(self, pulses):
        pass


def start(tmp_path, monkeypatch, config, args=(), fail_on=None):
    events = []
    make = lambda cfg, node=None: Recorder(cfg, events, fail_on)
    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', make)
    monkeypatch.setattr(outputs_module, 'make_outputs', make)  # used by the switch
    path = tmp_path / 'hardware.yaml'
    save_config(config, path)
    rclpy.init(args=['--ros-args', '-r', '__node:=config_reload_test',
                    '-p', f'config_file:={path}', *args])
    return thruster_pwm_node.ThrusterPWMNode(), path, events


def reload(node, path, config):
    save_config(config, path)
    node.watch.mtime = None  # reload now, even within one mtime tick
    node.reload_callback()


def test_pin_change_moves_the_outputs(tmp_path, monkeypatch):
    config = load_defaults()
    node, path, events = start(tmp_path, monkeypatch, config)
    try:
        events.clear()
        reload(node, path, dict(config, pwm_channels=[1, 0, 0, 0]))
        assert events == [('neutral', [0, 0, 0, 0]), ('close', [0, 0, 0, 0], True),
                          ('open', [1, 0, 0, 0])]
        assert list(node.get_parameter('pwm_channels').value) == [1, 0, 0, 0]
        assert node.timed_out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_failed_switch_reopens_the_old_pins_and_keeps_the_parameters(tmp_path, monkeypatch):
    config = load_defaults()
    node, path, events = start(tmp_path, monkeypatch, config, fail_on=[1, 0, 0, 0])
    try:
        events.clear()
        reload(node, path, dict(config, pwm_channels=[1, 0, 0, 0], reversed=[True] * 4))
        assert events[-1] == ('open', [0, 0, 0, 0])
        assert list(node.get_parameter('pwm_channels').value) == [0, 0, 0, 0]
        assert list(node.get_parameter('reversed').value) == [True] * 4  # the rest still applies
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_launch_override_survives_an_unrelated_edit(tmp_path, monkeypatch):
    config = dict(load_defaults(), output_backend='odroid')
    node, path, events = start(tmp_path, monkeypatch, config,
                               args=['-p', 'output_backend:=topic'])
    try:
        events.clear()
        reload(node, path, dict(config, reversed=[True] * 4))
        assert events == []
        assert node.get_parameter('output_backend').value == 'topic'
        assert list(node.get_parameter('reversed').value) == [True] * 4
    finally:
        node.destroy_node()
        rclpy.shutdown()
