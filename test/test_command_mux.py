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
