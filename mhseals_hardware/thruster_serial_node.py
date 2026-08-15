"""Mix planar velocity commands and send four PWM values to a Pico."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial


NEUTRAL_PWM = 1500
PWM_SCALE = 400
MIN_PWM = 1100
MAX_PWM = 1900

# Rows are FL, FR, RR, and RL. Columns are ROS-frame surge (+forward), sway
# (+left), and yaw (+counterclockwise). Positive local thrust points aft on
# the front pair and forward on the rear pair.
THRUSTER_MIXER = (
    (-1.0, 1.0, 1.0),
    (-1.0, -1.0, -1.0),
    (1.0, -1.0, 1.0),
    (1.0, 1.0, -1.0),
)


def validate_mixer(values):
    """Return a four-row mixer from a flat, finite 12-value sequence."""
    if len(values) != 12:
        raise ValueError('thruster_matrix must contain exactly 12 values')
    values = tuple(float(value) for value in values)
    if not all(value == value and abs(value) != float('inf')
               for value in values):
        raise ValueError('thruster_matrix values must be finite')
    return tuple(tuple(values[index:index + 3])
                 for index in range(0, 12, 3))


def validate_channel_map(values):
    """Validate canonical-to-physical output numbers."""
    channels = tuple(int(value) for value in values)
    if len(channels) != 4 or sorted(channels) != [1, 2, 3, 4]:
        raise ValueError('channel_map must be a permutation of [1, 2, 3, 4]')
    return channels


def map_channels(values, channel_map):
    """Map canonical FL,FR,RR,RL values onto four physical Pico outputs."""
    mapped = [None] * 4
    for value, physical_channel in zip(values, channel_map):
        mapped[physical_channel - 1] = value
    return mapped


def mix_thrusters(surge, sway, yaw, mixer=THRUSTER_MIXER):
    """Return normalized, desaturated outputs in FL, FR, RR, RL order."""
    commands = [
        row[0] * surge + row[1] * sway + row[2] * yaw
        for row in mixer
    ]
    peak = max(1.0, *(abs(command) for command in commands))
    return [command / peak for command in commands]


class ThrusterSerialNode(Node):
    """Forward mixed ``cmd_vel`` commands to the Pico over USB serial."""

    def __init__(self):
        super().__init__('thruster_serial_node')
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
