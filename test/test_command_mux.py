"""command_mux forwards a human input in the callback that receives it."""

import json
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn

from mhseals_hardware import command_mux


def test_rc_frame_reaches_applied_in_the_same_callback():
    rclpy.init(args=['--ros-args', '-r', '__node:=command_mux_test'])
    try:
        node = command_mux.CommandMux()
        sent = []
        node.applied.publish = sent.append
        frame = RCIn()
        # MANUAL switch (low), stick 2 fully forward.
        frame.channels = [1500, 2000, 1500, 1500, 1500, 1800, 1500, 1100]
        node._rc(frame)
        assert sent and sent[-1].linear.x == pytest.approx(0.5)
        assert node.arbitration.select(command_mux.time.monotonic())[0] == 'rc'
        assert node._pipeline_ms < 5          # hw04: receipt to applied publish
        states = []
        node.state_pub.publish = states.append
        node.publish_state(force=True)
        state = json.loads(states[-1].data)
        assert state['schema'] == 'mhseals.mux/1' and state['seq'] >= 1
        assert state['pipeline_ms'] < 5 and state['t_ns'] > 0 and state['cause'] == 'rc_switch'
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_remote_controller_is_gone():
    assert not Path(command_mux.__file__).with_name('remote_controller.py').exists()


def test_reload_during_rc_pends_until_idle(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_mux: {mode_ch: 8}\n')
    rclpy.init(args=['--ros-args', '-r', '__node:=command_mux_reload',
                     '-p', f'config_file:={path}'])
    try:
        node = command_mux.CommandMux()
        node.applied.publish = lambda message: None
        frame = RCIn()
        frame.channels = [1500, 2000, 1500, 1500, 1500, 1800, 1500, 1100]
        node._rc(frame)
        # ConfigWatcher.poll returns the full merged section, as load_config does.
        section = dict(node._effective_section, mode_ch=7)
        node.watch.poll = lambda: {'command_mux': section}
        node.reload_callback()
        assert node.arbitration.mode_ch == 8 and node.pending == section
        node.arbitration.rc_time = -10        # the frame goes stale
        node.watch.poll = lambda: None
        node.reload_callback()
        assert node.arbitration.mode_ch == 7 and node.pending is None
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_node_restart_keeps_a_hold(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text(f'command_mux: {{state_file: {tmp_path / "mux.json"}}}\n')
    args = ['--ros-args', '-r', '__node:=command_mux_restart', '-p', f'config_file:={path}']
    rclpy.init(args=args)
    try:
        node = command_mux.CommandMux()
        node.applied.publish = lambda message: None
        node.arbitration.request('hold', 'keys', 'keys', command_mux.time.monotonic())
        node.forward()
        node.destroy_node()
        node = command_mux.CommandMux()
        assert 'keys' in node.arbitration.holds
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_rc_switch_hold_is_saved_before_the_command_goes_out(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text(f'command_mux: {{state_file: {tmp_path / "mux.json"}}}\n')
    rclpy.init(args=['--ros-args', '-r', '__node:=command_mux_rc_hold',
                     '-p', f'config_file:={path}'])
    try:
        node = command_mux.CommandMux()
        node.applied.publish = lambda message: None
        frame = RCIn()
        frame.channels = [1500, 1500, 1500, 1500, 1500, 1800, 1500, 1500]   # HOLD
        node._rc(frame)
        saved = json.loads((tmp_path / 'mux.json').read_text())
        assert [row['cause'] for row in saved] == ['rc_switch']
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_steady_input_publishes_no_new_seq_until_something_changes():
    rclpy.init(args=['--ros-args', '-r', '__node:=command_mux_steady'])
    try:
        node = command_mux.CommandMux()
        node.applied.publish = lambda msg: None
        states = []
        node.state_pub.publish = states.append
        frame = RCIn()
        frame.channels = [1500, 2000, 1500, 1500, 1500, 1800, 1500, 1100]
        node._rc(frame)
        first = len(states)
        for _ in range(10):                      # constant-rate rc, 25 Hz keepalive
            command_mux.time.sleep(0.01)
            node._rc(frame)
            node.forward()
        assert len(states) == first
        node.publish_state(force=True)           # the 1 Hz heartbeat still publishes
        assert len(states) == first + 1
        node.arbitration.request('hold', 'test', 'tester', command_mux.time.monotonic())
        node.forward()                           # a hold publishes at once
        assert len(states) == first + 2
        assert json.loads(states[-1].data)['hold']['cause'] == 'test'
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
