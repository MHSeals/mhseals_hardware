"""Config reload must agree with telemetry and the manual Twist output path."""

import types

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
        node.watch.seen = node.watch._stat()
        node.reload_callback()
        assert list(node.get_parameter('decoupling').value) == correction
        assert list(node.get_parameter('decoupling_negative').value) == correction
        message = Twist()
        message.linear.x = .5
        node.cmd_vel_callback(message)
        assert outputs.pulses == [1280, 1680, 1320, 1720]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_int_edit_of_a_float_setting_reads_back(tmp_path, monkeypatch):
    class Outputs:
        def open(self):
            return self

        def close(self):
            pass

    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda *args, **kwargs: Outputs())
    config = dict(load_defaults(), command_timeout=0.5)
    path = tmp_path / 'hardware.yaml'
    save_config(config, path)
    rclpy.init(args=['--ros-args', '-r', '__node:=int_edit_test', '-p', f'config_file:={path}'])
    node = None
    try:
        node = thruster_pwm_node.ThrusterPWMNode()
        save_config(dict(config, command_timeout=1), path)
        node.watch.mtime = None
        node.watch.seen = node.watch._stat()
        node.reload_callback()
        assert node.command_timeout == 1.0
        assert node.get_parameter('command_timeout').value == 1.0
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
    node.watch.seen = node.watch._stat()
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


def test_failed_switch_is_not_logged_as_applied_and_is_retried(tmp_path, monkeypatch):
    config = load_defaults()
    node, path, events = start(tmp_path, monkeypatch, config, fail_on=[1, 0, 0, 0])
    try:
        reload(node, path, dict(config, pwm_channels=[1, 0, 0, 0]))
        assert node.file_config['pwm_channels'] == [0, 0, 0, 0]  # the last applied state
        events.clear()
        reload(node, path, dict(config, pwm_channels=[1, 0, 0, 0]))  # unchanged file, retried
        assert [event[0] for event in events][:2] == ['neutral', 'close']  # a second attempt
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_failed_switch_does_not_log_the_rolled_back_value_as_reloaded(tmp_path, monkeypatch):
    config = load_defaults()
    node, path, events = start(tmp_path, monkeypatch, config, fail_on=[1, 0, 0, 0])
    try:
        lines = []
        logger = types.SimpleNamespace(
            **{level: (lambda text, **kw: lines.append(text)) for level in
               ('info', 'warning', 'error')})
        monkeypatch.setattr(node, 'get_logger', lambda: logger)
        reload(node, path, dict(config, pwm_channels=[1, 0, 0, 0]))
        assert any('kept the old ones' in line for line in lines)
        assert not any('pwm_channels=' in line for line in lines)  # nothing applied, no 'reloaded'
        lines.clear()
        reload(node, path, dict(config, pwm_channels=[1, 0, 0, 0], reversed=[True] * 4))
        reloaded = [line for line in lines if line.startswith('config reloaded')]
        assert len(reloaded) == 1 and 'reversed=' in reloaded[0] and 'pwm_channels' not in reloaded[0]
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


@pytest.mark.parametrize('text', ['reversed: [true, false', 'manual_amplitude: "x"\n'])
def test_reload_survives_a_syntax_or_type_error(text, tmp_path, monkeypatch):
    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda *a, **k: Recorder(
        load_defaults(), []))
    path = tmp_path / 'hardware.yaml'
    save_config(load_defaults(), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=bad_reload_test',
                    '-p', f'config_file:={path}'])
    node = None
    try:
        node = thruster_pwm_node.ThrusterPWMNode()
        path.write_text(text)
        node.watch.mtime = None
        node.reload_callback()  # must log and keep the old config, not raise
        assert node.config['manual_amplitude'] == load_defaults()['manual_amplitude']
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_non_finite_cmd_vel_goes_neutral_without_raising(tmp_path, monkeypatch):
    class Outputs:
        neutrals = 0

        def open(self):
            return self

        def close(self):
            pass

        def neutral(self):
            self.neutrals += 1

        def set_pulse_widths(self, pulses):
            self.pulses = pulses

    outputs = Outputs()
    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda *args, **kwargs: outputs)
    path = tmp_path / 'hardware.yaml'
    save_config(load_defaults(), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=nan_test', '-p', f'config_file:={path}'])
    node = None
    try:
        node = thruster_pwm_node.ThrusterPWMNode()
        for bad in (float('nan'), float('inf')):
            message = Twist()
            message.linear.x = bad
            node.cmd_vel_callback(message)
        assert outputs.neutrals == 2 and not hasattr(outputs, 'pulses')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_test_pulses_only_when_autonomy_holds_the_thrusters(tmp_path, monkeypatch):
    from std_msgs.msg import String, UInt16MultiArray
    node, _, _ = start(tmp_path, monkeypatch, load_defaults())
    sent = []
    node.outputs.set_pulse_widths = sent.append
    pulses = UInt16MultiArray(data=[1500] * 4)
    try:
        node.pulses_callback(pulses)
        assert sent == []                      # no state yet
        for mux_input, accepted in (('hold', False), ('rc', False), ('manual', False),
                                    ('auto', True), ('idle', True)):
            sent.clear()
            node.guard_callback(String(data=f'{{"input": "{mux_input}"}}'))
            node.pulses_callback(pulses)
            assert bool(sent) is accepted, mux_input
        node.mux_time -= 10                    # a dead mux is not autonomy
        sent.clear()
        node.pulses_callback(pulses)
        assert sent == []
    finally:
        node.destroy_node()
        rclpy.shutdown()


def latest_status(node):
    """Join late, as the TUI does: a transient-local subscriber gets the last status."""
    import json
    import rclpy
    reader = rclpy.create_node('status_reader')
    got = []
    reader.create_subscription(thruster_pwm_node.String, f'/{node.get_name()}/status',
                               lambda message: got.append(json.loads(message.data)),
                               thruster_pwm_node.STATUS_QOS)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.02)
        rclpy.spin_once(reader, timeout_sec=0.02)
        if got:
            break
    reader.destroy_node()
    return got[-1]


def test_outside_set_edit_is_refused_with_a_reason(tmp_path, monkeypatch):
    node, path, events = start(tmp_path, monkeypatch, load_defaults())
    try:
        for name, value in (('frequency', 60.0), ('command_timeout', 1.0)):
            result = node.set_parameters([thruster_pwm_node.Parameter(name, value=value)])[0]
            assert not result.successful and 'edit hardware.yaml' in result.reason
        assert 'not a driver setting' not in result.reason  # command_timeout is a LIVE key
        assert 'not a driver setting' in node.set_parameters(
            [thruster_pwm_node.Parameter('frequency', value=60.0)])[0].reason
        assert node.get_parameter('command_timeout').value == 0.5
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_status_is_latched_and_follows_a_switch(tmp_path, monkeypatch):
    config = load_defaults()
    node, path, events = start(tmp_path, monkeypatch, config)
    try:
        first = latest_status(node)
        assert first['outputs']['pwm_channels'] == [0, 0, 0, 0] and first['error'] == ''
        assert first['mosfet'] == 'none' and len(first['config_sha']) == 12
        reload(node, path, dict(config, pwm_channels=[1, 0, 0, 0]))
        second = latest_status(node)
        assert second['outputs']['pwm_channels'] == [1, 0, 0, 0]
        assert second['session'] == first['session'] and second['config_sha'] != first['config_sha']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_edit_is_reported_in_the_status(tmp_path, monkeypatch):
    config = load_defaults()
    node, path, events = start(tmp_path, monkeypatch, config)
    try:
        sha = latest_status(node)['config_sha']
        # A syntactically fine edit that validation refuses (save_config would refuse it too).
        path.write_text(path.read_text().replace('command_timeout: 0.5', 'command_timeout: 9'))
        node.watch.mtime = None
        node.watch.seen = node.watch._stat()
        node.reload_callback()
        status = latest_status(node)
        assert status['error'] and status['config_sha'] == sha
        assert node.config['command_timeout'] == config['command_timeout']
    finally:
        node.destroy_node()
        rclpy.shutdown()
