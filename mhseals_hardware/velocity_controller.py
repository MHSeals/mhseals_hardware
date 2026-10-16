"""Small calibrated body-velocity to normalized-thrust controller.

The native PWM node consumes normalized effort, despite its historical
``cmd_vel`` topic name.  This node is the explicit boundary that accepts SI
body velocity commands from Nav2 and emits ``cmd_vel_effort`` for that node.
"""
from __future__ import annotations

import math
import time

import numpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from mhseals_hardware.configuration import load_config
from mhseals_hardware.ros_util import run_node, twist_from, twist_values
from mhseals_hardware.thruster_mixer import AXES


def _matrix(value):
    """Turn the saved {axis: {axis: gain}} mapping into a 3x3 matrix."""
    return tuple(tuple(float(value[row][name]) for name in AXES) for row in AXES)


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
        # velocity = bias + gain @ effort, so effort = gain^-1 @ (velocity - bias).
        try:
            self._gain_inv = numpy.linalg.inv(_matrix(gain)).tolist()
        except numpy.linalg.LinAlgError as error:
            raise ValueError("velocity_gain is singular") from error
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
        self._velocity = twist_values(message.twist.twist)

    def _command(self, message):
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self._last))
        self._last = now
        desired = twist_values(message)
        error = tuple(desired[i] - self._velocity[i] for i in range(3))
        feedback = bool(self.get_parameter('feedback').value)
        for i in range(3):
            if feedback:
                self._integral[i] = max(-self._integral_limit[i], min(
                    self._integral_limit[i], self._integral[i] + error[i] * dt))
        target = tuple(max(-self._limits[i], min(self._limits[i], desired[i])) for i in range(3))
        # Feed-forward from the fitted model, plus PI feedback on odometry error.
        feedforward = tuple(sum(self._gain_inv[i][j] * (target[j] - self._bias[j]) for j in range(3))
                            for i in range(3))
        effort = tuple(feedforward[i] + (self._kp[i] * error[i] + self._ki[i] * self._integral[i]
                                        if feedback else 0.0) for i in range(3))
        effort = tuple(max(-1.0, min(1.0, value)) if math.isfinite(value) else 0.0
                       for value in effort)
        self._pub.publish(twist_from(effort))
        self._last_command = now

    def _watchdog(self):
        if time.monotonic() - self._last_command <= float(self.get_parameter('command_timeout').value):
            return
        self._integral = [0.0, 0.0, 0.0]
        self._pub.publish(Twist())


def main(args=None):
    run_node(VelocityController, args)


if __name__ == '__main__':
    main()
