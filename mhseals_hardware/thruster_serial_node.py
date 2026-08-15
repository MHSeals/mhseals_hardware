"""Mix planar velocity commands and send four PWM values to a Pico."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial

from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, NEUTRAL_PWM, PWM_SCALE, THRUSTER_MIXER,
    map_channels, mix_thrusters, validate_channel_map, validate_mixer,
)


class ThrusterSerialNode(Node):
    """Forward mixed ``cmd_vel`` commands to the Pico over USB serial."""

    def __init__(self):
        super().__init__('thruster_serial_node')
        if not hasattr(serial, 'Serial'):
            raise RuntimeError(
                'PySerial is required, but Python imported the unrelated '
                f'"serial" package from {getattr(serial, "__file__", "?")}. '
                'Uninstall "serial" and install "pyserial==3.5".')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter(
            'thruster_matrix',
            [value for row in THRUSTER_MIXER for value in row],
        )
        self.declare_parameter('channel_map', [1, 2, 3, 4])

        port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.mixer = validate_mixer(
            self.get_parameter('thruster_matrix').value)
        self.channel_map = validate_channel_map(
            self.get_parameter('channel_map').value)
        self.serial = serial.Serial(port, baud_rate, timeout=0.1)
        self.last_command_time = time.monotonic()
        self.timed_out = False

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.1, self.watchdog_callback)
        self.send_pwm([NEUTRAL_PWM] * 4)
        self.get_logger().info(f'Sending thruster commands on {port}')

    def send_pwm(self, values):
        """Send one newline-delimited physical-output command."""
        message = ','.join(str(value) for value in values) + '\n'
        self.serial.write(message.encode('ascii'))

    def cmd_vel_callback(self, message):
        """Mix a planar velocity command and send it to the Pico."""
        outputs = mix_thrusters(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            self.mixer,
        )
        canonical_pwm = [
            max(MIN_PWM, min(MAX_PWM, round(NEUTRAL_PWM + PWM_SCALE * value)))
            for value in outputs
        ]
        self.send_pwm(map_channels(canonical_pwm, self.channel_map))
        self.last_command_time = time.monotonic()
        self.timed_out = False

    def watchdog_callback(self):
        """Stop the thrusters when velocity commands stop arriving."""
        elapsed = time.monotonic() - self.last_command_time
        if not self.timed_out and elapsed > self.command_timeout:
            self.send_pwm([NEUTRAL_PWM] * 4)
            self.timed_out = True
            self.get_logger().warning(
                'cmd_vel timed out; thrusters set to neutral')

    def destroy_node(self):
        """Neutralize the ESCs before closing the serial port."""
        try:
            self.send_pwm([NEUTRAL_PWM] * 4)
            self.serial.close()
        finally:
            super().destroy_node()


def main(args=None):
    """Run the thruster serial node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ThrusterSerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
