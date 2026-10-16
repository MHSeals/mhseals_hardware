"""Mix ROS velocity commands into native Odroid PWM outputs."""

import time

from geometry_msgs.msg import Twist
from rclpy.node import Node

from mhseals_hardware.configuration import load_config, validate_config
from mhseals_hardware.odroid_pwm import OdroidPWMOutputs
from mhseals_hardware.ros_util import run_node, twist_values
from mhseals_hardware.thruster_mixer import (
    effort_to_pulse, map_channels, mix_thrusters, validate_channel_map, validate_mixer,
)


class ThrusterPWMNode(Node):
    """Fail-neutral ROS bridge for the ODROID's hardware PWM outputs."""

    def __init__(self):
        super().__init__('thruster_pwm_node')
        # Every hardware setting is a ROS parameter, defaulting to the YAML config.
        self.declare_parameter('config_file', '')
        config = load_config(self.get_parameter('config_file').value or None)
        for key, value in config.items():
            self.declare_parameter(key, value)
        config = validate_config({key: self.get_parameter(key).value for key in config})
        self.command_timeout = float(
            self.get_parameter('command_timeout').value)
        self.channel_map = validate_channel_map(
            self.get_parameter('channel_map').value)
        self.mixer = validate_mixer(
            self.get_parameter('thruster_matrix').value)
        # Opening arms the ESCs at neutral; destroy_node disarms them.
        self.outputs = OdroidPWMOutputs.from_config(config).open()
        self.last_command_time = time.monotonic()
        self.timed_out = False
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.1, self.watchdog_callback)
        self.get_logger().info('Odroid native PWM thruster control ready')

    def cmd_vel_callback(self, message):
        # Twist effort -> per-thruster effort -> pulse widths -> physical outputs.
        mixed = mix_thrusters(*twist_values(message), self.mixer)
        canonical = [effort_to_pulse(value) for value in mixed]
        self.outputs.set_pulse_widths(map_channels(canonical, self.channel_map))
        self.last_command_time = time.monotonic()
        self.timed_out = False

    def watchdog_callback(self):
        # Fail neutral when the command stream stops (crashed teleop, lost link).
        if (not self.timed_out and time.monotonic() - self.last_command_time >
                self.command_timeout):
            self.outputs.neutral()
            self.timed_out = True
            self.get_logger().warning('cmd_vel timed out; thrusters neutral')

    def destroy_node(self):
        try:
            self.outputs.close()
        finally:
            super().destroy_node()


def main(args=None):
    run_node(ThrusterPWMNode, args)


if __name__ == '__main__':
    main()
