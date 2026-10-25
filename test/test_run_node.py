"""run_node exits quietly on the spin-versus-shutdown race but not on real errors."""

import os
import signal
import subprocess
import sys
import textwrap
import time

import pytest

pytest.importorskip('rclpy')
from mhseals_hardware import ros_util  # noqa: E402


class FakeNode:
    def __init__(self):
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


def run(monkeypatch, error, context_ok):
    node = FakeNode()

    calls = []

    def spin_once(_node, timeout_sec):
        calls.append(timeout_sec)
        raise error

    def ok():
        return context_ok or not calls

    monkeypatch.setattr(ros_util.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(ros_util.rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(ros_util.rclpy, 'ok', ok)
    monkeypatch.setattr(ros_util.rclpy, 'try_shutdown', lambda: None)
    try:
        ros_util.run_node(lambda: node)
    finally:
        assert node.destroyed


def test_shutdown_race_is_quiet(monkeypatch):
    run(monkeypatch, RuntimeError('Unable to convert call argument 0 to Python object'), False)


def test_runtime_error_with_live_context_propagates(monkeypatch):
    with pytest.raises(RuntimeError):
        run(monkeypatch, RuntimeError('boom'), True)


def test_sigint_exits_on_stalled_sim_clock():
    # use_sim_time with no /clock publisher: ROS-time timers never fire.
    script = textwrap.dedent("""
        import sys
        from rclpy.node import Node
        from mhseals_hardware.ros_util import run_node

        class Idle(Node):
            def __init__(self):
                super().__init__('idle_sim', parameter_overrides=[
                    __import__('rclpy').parameter.Parameter('use_sim_time', value=True)])
                self.create_timer(0.1, lambda: None)
                print('ready', flush=True)

        run_node(Idle)
    """)
    env = dict(os.environ, ROS_DOMAIN_ID='198')
    proc = subprocess.Popen([sys.executable, '-c', script], stdout=subprocess.PIPE, env=env)
    try:
        assert proc.stdout.readline().strip() == b'ready'
        proc.send_signal(signal.SIGINT)
        start = time.monotonic()
        proc.wait(timeout=5)
        assert time.monotonic() - start < 1.0
    finally:
        proc.kill()


def test_driver_sigterm_runs_the_ctrl_c_shutdown(monkeypatch):
    """SIGTERM reaches destroy_node (MOSFET off, neutral), and a repeat cannot cut it short."""
    from mhseals_hardware import thruster_pwm_node
    node = FakeNode()

    def spin_once(_node, timeout_sec):
        os.kill(os.getpid(), signal.SIGTERM)
        time.sleep(0.5)  # the handler interrupts this

    monkeypatch.setattr(ros_util.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(ros_util.rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(ros_util.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(ros_util.rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(thruster_pwm_node, 'ThrusterPWMNode', lambda: node)
    previous = signal.getsignal(signal.SIGTERM)
    try:
        thruster_pwm_node.main()
        assert signal.getsignal(signal.SIGTERM) == signal.SIG_IGN
    finally:
        signal.signal(signal.SIGTERM, previous)
    assert node.destroyed
