"""The driver reads its arm snapshot once; only C changes live (SetParameters)."""

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist

from mhseals_hardware.configuration import load_defaults, save_config
from mhseals_hardware import outputs as outputs_module, thruster_pwm_node
from mhseals_hardware.thruster_mixer import thruster_pulses, validate_decoupling


def test_a_correction_set_applies_live_to_parameters_and_pulses(tmp_path, monkeypatch):
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
        result = node.set_parameters([thruster_pwm_node.Parameter('decoupling', value=correction)])[0]
        assert result.successful
        assert list(node.get_parameter('decoupling').value) == correction
        assert list(node.get_parameter('decoupling_negative').value) == correction
        message = Twist()
        message.linear.x = .5
        node.cmd_vel_callback(message)
        assert outputs.pulses != [1300, 1700, 1300, 1700]  # the uncorrected surge
        assert outputs.pulses == thruster_pulses(
            (.5, 0., 0.), node.mixer, (1, 3, 2, 4), decoupling=validate_decoupling(correction))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_a_constructor_failure_after_open_closes_the_outputs(tmp_path, monkeypatch):
    closed = []

    class Outputs:
        def open(self):
            return self

        def close(self):
            closed.append(True)

    def broken(*args, **kwargs):
        raise RuntimeError('estop setup failed')
    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda *args, **kwargs: Outputs())
    monkeypatch.setattr(thruster_pwm_node, 'RemoteEstop', broken)
    path = tmp_path / 'hardware.yaml'
    save_config(load_defaults(), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=ctor_failure_test',
                     '-p', f'config_file:={path}'])
    try:
        with pytest.raises(RuntimeError, match='estop setup failed'):
            thruster_pwm_node.ThrusterPWMNode()
        assert closed == [True]  # MOSFET off and neutral: run_node gets no node to destroy
    finally:
        rclpy.shutdown()


def test_the_code_version_is_read_before_the_outputs_open(tmp_path, monkeypatch):
    events = []

    class Outputs:
        def open(self):
            events.append('open')
            return self

        def close(self):
            pass

    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda *args, **kwargs: Outputs())
    monkeypatch.setattr(thruster_pwm_node, 'code_version',
                        lambda: events.append('version') or {'digest': 'x'})
    path = tmp_path / 'hardware.yaml'
    save_config(load_defaults(), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=version_order_test', '-p', f'config_file:={path}'])
    node = None
    try:
        node = thruster_pwm_node.ThrusterPWMNode()
        assert events == ['version', 'open']  # nothing slow between power and the deadman
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


def test_a_p_override_wins_and_a_yaml_edit_waits_for_the_next_arm(tmp_path, monkeypatch):
    config = dict(load_defaults(), output_backend='odroid')
    node, path, events = start(tmp_path, monkeypatch, config,
                               args=['-p', 'output_backend:=topic', '-p', 'command_timeout:=1.0',
                                     '-p', 'channel_map:=[2, 1, 3, 4]'])
    try:
        assert node.get_parameter('output_backend').value == 'topic'
        assert node.config['channel_map'] == [2, 1, 3, 4] and node.map_source == '-p channel_map'
        assert node.config['output_backend'] == 'topic' and node.command_timeout == 1.0
        events.clear()
        save_config(dict(config, reverse_thrust_ratio=0.5, reversed=[True] * 4), path)
        assert not hasattr(node, 'watch') and events == []
        assert node.get_parameter('reverse_thrust_ratio').value == config['reverse_thrust_ratio']
        assert list(node.get_parameter('reversed').value) == [False] * 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_p_board_address_refuses_at_start(tmp_path, monkeypatch):
    """The boot service prepares default.yaml's files only, so -p cannot move the driver off them."""
    line = load_defaults()['mosfet_line'] + 1
    try:
        with pytest.raises(ValueError, match='mosfet_line'):
            start(tmp_path, monkeypatch, load_defaults(), args=['-p', f'mosfet_line:={line}'])
    finally:
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


def test_test_pulses_only_when_the_mux_is_idle(tmp_path, monkeypatch):
    from std_msgs.msg import String, UInt16MultiArray
    node, _, _ = start(tmp_path, monkeypatch, load_defaults())
    sent = []
    node.outputs.set_pulse_widths = sent.append
    pulses = UInt16MultiArray(data=[1500] * 4)
    try:
        node.pulses_callback(pulses)
        assert sent == []                      # no state yet
        for mux_input, accepted in (('hold', False), ('rc', False), ('manual', False),
                                    ('auto', False), ('idle', True)):
            sent.clear()
            node.guard_callback(String(data=f'{{"schema": "mhseals.mux/1", "input": "{mux_input}"}}'))
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
        result = node.set_parameters([thruster_pwm_node.Parameter('frequency', value=60.0)])[0]
        assert not result.successful and 'next arm' in result.reason
        result = node.set_parameters([thruster_pwm_node.Parameter('command_timeout', value=1.0)])[0]
        assert not result.successful and 'next arm' in result.reason
        assert node.command_timeout == node.get_parameter('command_timeout').value != 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_status_is_latched_and_names_one_snapshot_sha(tmp_path, monkeypatch):
    from mhseals_hardware.configuration import config_sha, load_config
    node, path, events = start(tmp_path, monkeypatch, load_defaults())
    try:
        first = latest_status(node)
        assert first['outputs']['pwm_channels'] == [0, 0, 0, 0] and first['mosfet'] == 'none'
        assert first['config_sha'] == config_sha(load_config(path))
        assert not {'pending_restart_keys', 'requested_config_sha', 'effective_config_sha'} & set(first)
    finally:
        node.destroy_node()
        rclpy.shutdown()
