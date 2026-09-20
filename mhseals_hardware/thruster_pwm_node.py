"""Mix ROS velocity commands into native Odroid PWM outputs."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from mhseals_hardware.odroid_pwm import (
    DEFAULT_MOSFET_CHIP, DEFAULT_MOSFET_LINE, DEFAULT_PWM_CHIPS,
    OdroidPWMOutputs,
)
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, NEUTRAL_PWM, PWM_SCALE, THRUSTER_MIXER,
    map_channels, mix_thrusters, validate_channel_map, validate_mixer,
)


class ThrusterPWMNode(Node):
    """Fail-neutral ROS bridge with no Pico or serial transport."""

    def __init__(self):
        super().__init__('thruster_pwm_node')
        self.declare_parameter('pwm_chips', list(DEFAULT_PWM_CHIPS))
        self.declare_parameter('pwm_channels', [0, 0, 0, 0])
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('mosfet_chip', DEFAULT_MOSFET_CHIP)
        self.declare_parameter('mosfet_line', DEFAULT_MOSFET_LINE)
        self.declare_parameter('mosfet_active_high', True)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('channel_map', [1, 2, 3, 4])
        self.declare_parameter('thruster_matrix',
                               [v for row in THRUSTER_MIXER for v in row])
        self.command_timeout = float(
            self.get_parameter('command_timeout').value)
        self.channel_map = validate_channel_map(
            self.get_parameter('channel_map').value)
        self.mixer = validate_mixer(
            self.get_parameter('thruster_matrix').value)
        self.outputs = OdroidPWMOutputs(
            self.get_parameter('pwm_chips').value,
            self.get_parameter('pwm_channels').value,
            self.get_parameter('frequency').value,
            mosfet_chip=self.get_parameter('mosfet_chip').value,
            mosfet_line=self.get_parameter('mosfet_line').value,
            mosfet_active_high=self.get_parameter(
                'mosfet_active_high').value).open()
        self.last_command_time = time.monotonic()
        self.timed_out = False
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.1, self.watchdog_callback)
        self.get_logger().info('Odroid native PWM thruster control ready')

    def cmd_vel_callback(self, message):
        mixed = mix_thrusters(message.linear.x, message.linear.y,
                              message.angular.z, self.mixer)
        canonical = [max(MIN_PWM, min(MAX_PWM,
                         round(NEUTRAL_PWM + PWM_SCALE * value)))
                     for value in mixed]
        self.outputs.set_pulse_widths(map_channels(canonical, self.channel_map))
        self.last_command_time = time.monotonic()
        self.timed_out = False

    def watchdog_callback(self):
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
    rclpy.init(args=args)
    node = None
    try:
        node = ThrusterPWMNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
