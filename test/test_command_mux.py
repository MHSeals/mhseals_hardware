"""command_mux forwards a human input in the callback that receives it."""

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
        node.watch.poll = lambda: {'command_mux': {'mode_ch': 7}}
        node.reload_callback()
        assert node.arbitration.mode_ch == 8 and node.pending == {'mode_ch': 7}
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
