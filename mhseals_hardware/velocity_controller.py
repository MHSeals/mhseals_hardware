"""Small calibrated body-velocity to normalized-thrust controller.

The native PWM node consumes normalized effort, despite its historical
``cmd_vel`` topic name.  This node is the explicit boundary that accepts SI
body velocity commands from Nav2 and emits ``cmd_vel_effort`` for that node.
"""
from __future__ import annotations

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from mhseals_hardware.configuration import load_config

AXES = ("surge", "sway", "yaw")


def _matrix(value):
    return tuple(tuple(float(row[name]) for name in AXES) for row in value)


def _inverse(matrix):
    aug = [list(row) + [1.0 if i == j else 0.0 for j in range(3)]
           for i, row in enumerate(matrix)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda i: abs(aug[i][col]))
        if abs(aug[pivot][col]) < 1e-9:
            raise ValueError("velocity_gain is singular")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        scale = aug[col][col]
        aug[col] = [v / scale for v in aug[col]]
        for row in range(3):
            if row == col:
                continue
            scale = aug[row][col]
            aug[row] = [a - scale * b for a, b in zip(aug[row], aug[col])]
    return tuple(tuple(aug[i][j + 3] for j in range(3)) for i in range(3))


class VelocityController(Node):
    def __init__(self):
        super().__init__('calibrated_velocity_controller')
        self.declare_parameter('config_file', '')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_effort')
        self.declare_parameter('odom_topic', '/odom/local')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('feedback', True)
        self.declare_parameter('kp', [0.1, 0.1, 0.05])
        self.declare_parameter('ki', [0.1, 0.1, 0.05])
        self.declare_parameter('integral_limit', [0.2, 0.2, 0.2])
        config = load_config(self.get_parameter('config_file').value or None)
        characterization = config.get('characterization', {})
        gain = characterization.get('velocity_gain')
        if not gain:
            raise ValueError('characterization.velocity_gain is required')
        self._gain_inv = _inverse(_matrix(gain))
        self._bias = tuple(float(characterization.get('bias', {}).get(axis, 0.0))
                           for axis in AXES)
        limits = characterization.get('motion_limits', {}).get('velocity', {})
        self._limits = tuple(float(limits.get(axis, 0.0)) for axis in AXES)
        if any(limit <= 0 for limit in self._limits):
            raise ValueError('characterization motion velocity limits are required')
        self._kp = tuple(float(v) for v in self.get_parameter('kp').value)
        self._ki = tuple(float(v) for v in self.get_parameter('ki').value)
        self._integral_limit = tuple(float(v) for v in self.get_parameter('integral_limit').value)
        self._integral = [0.0, 0.0, 0.0]
        self._velocity = (0.0, 0.0, 0.0)
        self._last = time.monotonic()
        self._last_command = self._last
        self._pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('input_topic').value, self._command, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._odom, 10)
        self.create_timer(0.05, self._watchdog)

    def _odom(self, message):
        twist = message.twist.twist
        self._velocity = (twist.linear.x, twist.linear.y, twist.angular.z)

    def _command(self, message):
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self._last))
        self._last = now
        desired = (message.linear.x, message.linear.y, message.angular.z)
        error = tuple(desired[i] - self._velocity[i] for i in range(3))
        feedback = bool(self.get_parameter('feedback').value)
        for i in range(3):
            if feedback:
                self._integral[i] = max(-self._integral_limit[i], min(
                    self._integral_limit[i], self._integral[i] + error[i] * dt))
        target = tuple(max(-self._limits[i], min(self._limits[i], desired[i])) for i in range(3))
        feedforward = tuple(sum(self._gain_inv[i][j] * (target[j] - self._bias[j]) for j in range(3))
                            for i in range(3))
        effort = tuple(feedforward[i] + (self._kp[i] * error[i] + self._ki[i] * self._integral[i]
                                        if feedback else 0.0) for i in range(3))
        effort = tuple(max(-1.0, min(1.0, value)) if math.isfinite(value) else 0.0
                       for value in effort)
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = effort
        self._pub.publish(out)
        self._last_command = now

    def _watchdog(self):
        if time.monotonic() - self._last_command <= float(self.get_parameter('command_timeout').value):
            return
        self._integral = [0.0, 0.0, 0.0]
        self._pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
