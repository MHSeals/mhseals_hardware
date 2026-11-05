"""Calibrated body-velocity to normalized-effort controller.

The one unit boundary on the auto lane, before the command_mux: SI body velocity from Nav2
on ``topics.auto`` becomes normalized effort on ``topics.effort``, the mux's auto input. The
control law lives in ``velocity_law``; this node only wires it to ROS and publishes its live
drift estimate on ``~/drift`` (odom frame, m/s). It drives only while the mux input is ``auto``
(the contract in command_mux); any other input, or an unknown state, resets the law and sends
neutral effort, so a hold, rc or manual leaves no stale integral behind. The node
reads its arm snapshot (``config_file``, ``-p`` overrides win) once and never watches the
YAML; only the gains change live, through SetParameters (``command_timeout``, shared with
the mux and the driver, applies at the next arm, so the three never diverge). Callbacks run on one
single-threaded executor, so a parameter batch never interleaves with a command.
"""
from __future__ import annotations

import copy
import math
import time

from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import numpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from mhseals_hardware.control_validation import (
    GAIN_KEYS, SCALAR_GAINS, TOPIC_KEYS, VECTOR_GAINS, controller_section,
    validate_controller, validate_drift_limit,
)
from mhseals_hardware.command_arbitration import mux_input
from mhseals_hardware.configuration import declared, load_config
from mhseals_hardware.ros_util import run_node, twist_from, twist_values
from mhseals_hardware.velocity_law import Plant, VelocityLaw


# The mux's latched ~/state, so the current hold reaches a controller started late.
STATE_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(orientation):
    """Heading of a quaternion about +z."""
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VelocityController(Node):
    def __init__(self):
        super().__init__('calibrated_velocity_controller')
        self.declare_parameter('config_file', '')
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        config = load_config(value('config_file') or None)
        controller_section(declared(self, config['controller']))
        declared(self, config, ['command_timeout'])  # the one watchdog, shared with mux and driver
        self._law = VelocityLaw(Plant(config), value('kp'), value('ki'), value('integral_limit'),
                                value('drift_time_constant'), value('odom_timeout'),
                                value('drift_command_limit'), value('drift_limit'),
                                value('drift_carry'), value('proximity_distance'),
                                value('obstacle_timeout'))
        validate_drift_limit(self._law.drift_limit, self._law.plant.limits)
        self._held = True  # until the mux reports auto: neutral effort, a reset law
        self._frame = 'odom'
        self._last_command = time.monotonic()
        topics = config['topics']
        self._pub = self.create_publisher(Twist, topics['effort'], 10)
        self._drift = self.create_publisher(Vector3Stamped, '~/drift', 10)
        self.create_subscription(Twist, topics['auto'], self._command, 10)
        self.create_subscription(Odometry, value('odom_topic'), self._odom, 10)
        self.create_subscription(String, topics['mux_state'], self._mux_state, STATE_QOS)
        self.create_subscription(Vector3Stamped, value('obstacle_topic'), self._obstacle, 10)
        self.add_on_set_parameters_callback(self._validate_gains)
        self.add_post_set_parameters_callback(self._commit_gains)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(0.2, self._publish_drift)

    def _validate_gains(self, parameters):
        """Pure preflight; a later validator can still refuse the whole batch."""
        values = {name: self.get_parameter(name).value
                  for name in GAIN_KEYS + ('feedback',)}
        for parameter in parameters:
            if parameter.name in TOPIC_KEYS + ('config_file', 'command_timeout'):
                return SetParametersResult(successful=False,
                                           reason='applies at the next arm: ' + parameter.name)
            if parameter.name in values:
                values[parameter.name] = parameter.value
        try:
            validate_controller(values)
            validate_drift_limit(values['drift_limit'], self._law.plant.limits)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        candidate = copy.copy(self._law)
        for name in GAIN_KEYS:
            value = values[name]
            setattr(candidate, name, numpy.array(value, float) if name in VECTOR_GAINS
                    else float(value))
        candidate.integral = numpy.clip(self._law.integral,
                                        -candidate.integral_limit, candidate.integral_limit)
        speed = math.hypot(*self._law.drift)
        candidate.drift = self._law.drift * min(1.0, candidate.drift_limit / speed) if speed else \
            self._law.drift.copy()
        self._candidate_law = candidate  # unconsumed if any later callback refuses
        return SetParametersResult(successful=True)

    def _commit_gains(self, parameters):
        """Jazzy post-set runs only after all validators and parameter storage.

        Preserve observer/history and reproject accumulated PI effort onto new
        integral bounds. The Plant is fixed for the node's life (one arm).
        """
        if not any(p.name in GAIN_KEYS for p in parameters):
            return
        self._law = self._candidate_law

    def _odom(self, message):
        self._frame = message.header.frame_id or self._frame
        self._law.observe(twist_values(message.twist.twist), yaw_of(message.pose.pose.orientation),
                          time.monotonic())

    def _obstacle(self, message):
        """Nearest obstacle from the hull, base frame; NaN when none is near."""
        self._law.observe_obstacle((message.vector.x, message.vector.y), time.monotonic())

    def _command(self, message):
        now = time.monotonic()
        if self._held:
            self._law.reset()  # no stale integral or effort when driving resumes
            self._pub.publish(Twist())
            self._last_command = now
            return
        effort = self._law.step(twist_values(message), now,
                                feedback=bool(self.get_parameter('feedback').value))
        self._pub.publish(twist_from(effort))
        self._last_command = now

    def _mux_state(self, message):
        """Drive only while the mux input is auto; anything else, or unknown, resets.

        Nothing is published here: the mux forwards the auto lane only while its input is auto,
        and a zero on the transition would refresh a stale lane, so the mux would never idle.
        """
        held = mux_input(message.data) != 'auto'
        if held and not self._held:
            self._law.reset()
        self._held = held

    def _publish_drift(self):
        drift = Vector3Stamped()
        drift.header.stamp = self.get_clock().now().to_msg()
        drift.header.frame_id = self._frame
        drift.vector.x, drift.vector.y = (float(v) for v in self._law.drift)
        self._drift.publish(drift)

    def _watchdog(self):
        if time.monotonic() - self._last_command <= float(self.get_parameter('command_timeout').value):
            return
        # The mux goes idle on its own; match it so the drift estimate keeps learning from
        # zero effort. Publishing here would keep the auto lane fresh, so the mux never idles.
        self._law.reset()


def main(args=None):
    run_node(VelocityController, args)


if __name__ == '__main__':
    main()
