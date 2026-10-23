"""The velocity controller rebuilds its model when the hardware YAML changes."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist

from mhseals_hardware.configuration import save_config
from mhseals_hardware.velocity_controller import VelocityController
from test_velocity_law import config

STRONGER = ((0.8, 0.0, 0.0), (0.0, 0.4, 0.0), (0.05, 0.0, 0.3))


def test_reload_rebuilds_the_model_and_holds_neutral_while_invalid(tmp_path):
    path = tmp_path / 'hardware.yaml'
    save_config(config(), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=velocity_reload_test',
                     '-p', f'config_file:={path}'])
    node = None
    try:
        node = VelocityController()
        sent = []
        node._pub.publish = sent.append
        curves = dict(node._law.plant.curves)

        def edit(new):
            save_config(new, path)
            node._watch.mtime = None  # reload now, even within one mtime tick
            node._watch.seen = node._watch._stat()
            node._reload()

        edit(config(gain=STRONGER))
        assert node._law.plant.curves != curves
        edit(dict(config(), characterization={}))  # no velocity_gain: Plant refuses it
        assert node._fault
        push = Twist()
        push.linear.x = 0.3

        def command():
            node._law.observe((0., 0., 0.), 0., time.monotonic())  # fresh odometry
            node._command(push)

        command()
        assert sent[-1] == Twist()
        edit(config())
        assert not node._fault and node._law.plant.curves == curves
        command()
        assert sent[-1] != Twist()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
