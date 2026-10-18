"""Config reload must agree with telemetry and the manual Twist output path."""

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist

from mhseals_hardware.configuration import load_defaults, save_config
from mhseals_hardware import thruster_pwm_node


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
        node.config_mtime = None
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
