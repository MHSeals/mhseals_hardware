"""Calibrated body-velocity to normalized-effort controller.

The native PWM node consumes normalized effort, despite its historical
``cmd_vel`` topic name. This node is the explicit boundary that accepts SI
body velocity commands from Nav2 and emits ``cmd_vel_effort`` for that node.
The control law lives in ``velocity_law``; this node only wires it to ROS and
publishes its live drift estimate on ``~/drift`` (odom frame, m/s). With
``config_file`` set it rebuilds the model when the YAML changes, and holds neutral
effort while the new model is invalid.
"""
from __future__ import annotations

import math
import time

from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import numpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from mhseals_hardware.configuration import ConfigWatcher, load_config
from mhseals_hardware.ros_util import run_node, twist_from, twist_values
from mhseals_hardware.velocity_law import Plant, VelocityLaw


# The config keys Plant reads; an edit to anything else keeps the model and the integrator.
PLANT_KEYS = ('thruster_matrix', 'characterization', 'decoupling', 'decoupling_negative')
# Controller gains: read from the parameters on every command, so a change is live.
GAIN_KEYS = ('kp', 'ki', 'integral_limit', 'drift_time_constant', 'odom_timeout',
             'drift_command_limit')


def yaw_of(orientation):
    """Heading of a quaternion about +z."""
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VelocityController(Node):
    def __init__(self):
        super().__init__('calibrated_velocity_controller')
        self.declare_parameter('config_file', '')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_effort')
        self.declare_parameter('odom_topic', '/odom/local')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('feedback', True)
        # Effort per m/s (rad/s): about one over the boat's gain, so feedback is felt;
        # turning while moving needs it, as the steady model has no turning terms.
        self.declare_parameter('kp', [0.8, 0.8, 0.6])
        self.declare_parameter('ki', [0.6, 0.6, 0.4])
        self.declare_parameter('integral_limit', [0.3, 0.3, 0.3])
        # Seconds for the world-frame drift estimate to settle; 0 disables it.
        self.declare_parameter('drift_time_constant', 5.0)
        self.declare_parameter('odom_timeout', 0.3)
        # Largest commanded speed at which the drift estimate still learns (velocity_law).
        self.declare_parameter('drift_command_limit', 0.1)
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        config = load_config(value('config_file') or None)
        self._law = VelocityLaw(Plant(config), value('kp'), value('ki'), value('integral_limit'),
                                value('drift_time_constant'), value('odom_timeout'),
                                value('drift_command_limit'))
        self._plant_inputs = self._inputs(config)
        self._fault = ''  # why the model is invalid; neutral effort until it is fixed
        self._frame = 'odom'
        self._last_command = time.monotonic()
        self._pub = self.create_publisher(Twist, value('output_topic'), 10)
        self._drift = self.create_publisher(Vector3Stamped, '~/drift', 10)
        self.create_subscription(Twist, value('input_topic'), self._command, 10)
        self.create_subscription(Odometry, value('odom_topic'), self._odom, 10)
        self.add_on_set_parameters_callback(self._validate_gains)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(0.2, self._publish_drift)
        if value('config_file'):
            self._watch = ConfigWatcher(value('config_file'))
            self.create_timer(1.0, self._reload)

    def _validate_gains(self, parameters):
        """Accept a gain change only if it is finite, and of the right length; apply it live."""
        values = {}
        for parameter in parameters:
            if parameter.name in GAIN_KEYS:
                try:
                    value = numpy.array(parameter.value, float)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason='not numeric')
                shape = () if parameter.name.endswith(('constant', 'timeout', 'limit')) and \
                    parameter.name != 'integral_limit' else (3,)
                if value.shape != shape or not numpy.all(numpy.isfinite(value)):
                    return SetParametersResult(successful=False, reason=f'need finite {shape}')
                values[parameter.name] = value
        for name, value in values.items():
            attribute = getattr(self._law, name)
            setattr(self._law, name, value if isinstance(attribute, numpy.ndarray) else float(value))
        return SetParametersResult(successful=True)

    @staticmethod
    def _inputs(config):
        return {key: config.get(key) for key in PLANT_KEYS}

    def _reload(self):
        """Rebuild the model after a YAML edit. The driver applies the same file."""
        try:
            config = self._watch.poll()
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().error(f'config reload rejected, keeping the old model: {error}')
            return
        if config is None:
            return
        inputs = self._inputs(config)
        if inputs == self._plant_inputs and not self._fault:
            return  # a pin, timeout or manual_* edit: keep the integral
        self._plant_inputs = inputs
        self._law.reset()
        try:
            self._law.plant = Plant(config)
        except ValueError as error:
            # The driver already applied this file, so the old model is wrong too.
            self._fault = str(error)
            self.get_logger().error(f'velocity model invalid, holding neutral: {error}')
            return
        self._fault = ''
        self.get_logger().info('velocity model rebuilt')

    def _odom(self, message):
        self._frame = message.header.frame_id or self._frame
        self._law.observe(twist_values(message.twist.twist), yaw_of(message.pose.pose.orientation),
                          time.monotonic())

    def _command(self, message):
        now = time.monotonic()
        if self._fault:
            self._pub.publish(Twist())
            self._last_command = now
            return
        effort = self._law.step(twist_values(message), now,
                                feedback=bool(self.get_parameter('feedback').value))
        self._pub.publish(twist_from(effort))
        self._last_command = now

    def _publish_drift(self):
        drift = Vector3Stamped()
        drift.header.stamp = self.get_clock().now().to_msg()
        drift.header.frame_id = self._frame
        drift.vector.x, drift.vector.y = (float(v) for v in self._law.drift)
        self._drift.publish(drift)

    def _watchdog(self):
        if time.monotonic() - self._last_command <= float(self.get_parameter('command_timeout').value):
            return
        # The driver goes neutral on its own; match it so the drift estimate
        # keeps learning from zero effort.
        self._law.reset()
        self._pub.publish(Twist())


def main(args=None):
    run_node(VelocityController, args)


if __name__ == '__main__':
    main()
