#!/usr/bin/env python3

"""Convert MAVROS RC input into body-frame velocity commands."""

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class RemoteController(Node):
    """Publish ``cmd_vel`` commands from the translation and yaw RC sticks."""

    def __init__(self):
        super().__init__('remote_controller')

        self.declare_parameter('center_pwm', 1500)
        self.declare_parameter('pwm_range', 500)
        self.declare_parameter('deadband_pwm', 40)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('timeout', 0.5)

        self.center_pwm = self.get_parameter('center_pwm').value
        self.pwm_range = self.get_parameter('pwm_range').value
        self.deadband_pwm = self.get_parameter('deadband_pwm').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.timeout = self.get_parameter('timeout').value
        self.last_rc_time = None

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.1, self.stop_if_stale)

    def normalized_channel(self, pwm):
        """Return a channel value in [-1, 1], with a center deadband."""
        offset = float(pwm - self.center_pwm)
        if abs(offset) <= self.deadband_pwm:
            return 0.0

        value = offset / float(self.pwm_range)
        return max(-1.0, min(1.0, value))

    def rc_callback(self, msg):
        # RC channel numbers are one-based; the message array is zero-based.
        if len(msg.channels) < 4:
            self.get_logger().warning(
                'RCIn has fewer than four channels; stopping the boat',
                throttle_duration_sec=5.0,
            )
            self.publisher.publish(Twist())
            return

        channel_1 = self.normalized_channel(msg.channels[0])
        channel_2 = self.normalized_channel(msg.channels[1])
        channel_4 = self.normalized_channel(msg.channels[3])

        command = Twist()
        command.linear.x = channel_2 * self.max_linear_speed
        command.linear.y = -channel_1 * self.max_linear_speed
        command.angular.z = -channel_4 * self.max_angular_speed
        self.publisher.publish(command)
        self.last_rc_time = self.get_clock().now()

    def stop_if_stale(self):
        if self.last_rc_time is None:
            return

        age = (self.get_clock().now() - self.last_rc_time).nanoseconds / 1e9
        if age > self.timeout:
            self.publisher.publish(Twist())
            self.last_rc_time = None
            self.get_logger().warning('RC input timed out; stopping the boat')


def main(args=None):
    rclpy.init(args=args)
    node = RemoteController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
