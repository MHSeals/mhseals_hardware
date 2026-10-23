#!/usr/bin/env python3

"""Choose which command reaches the driver: rc > manual > hold > auto.

All rules live in ``command_arbitration.Arbitration``; this node only wires
topics.  A human input is forwarded in the callback that receives it, and a
25 Hz timer keeps the output alive between frames.
"""

import json
import time

from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String

from mhseals_hardware.command_arbitration import Arbitration
from mhseals_hardware.ros_util import run_node, twist_from, twist_values

STATE_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class CommandMux(Node):
    """Publish ``/cmd_vel/applied`` and the latched ``~/state``."""

    def __init__(self):
        super().__init__('command_mux')
        self.declare_parameter('rc_scale', [0.5, 0.5, 0.3])
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('ramp', 1.0)
        self.arbitration = Arbitration(
            rc_scale=self.get_parameter('rc_scale').value,
            cmd_timeout=self.get_parameter('cmd_timeout').value,
            ramp=self.get_parameter('ramp').value)

        self.applied = self.create_publisher(Twist, '/cmd_vel/applied', 10)
        self.state_pub = self.create_publisher(String, '~/state', STATE_QOS)
        self.create_subscription(RCIn, '/mavros/rc/in', self._rc, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel/manual', self._manual, 10)
        self.create_subscription(Twist, '/cmd_vel', self._auto, 10)
        self.create_subscription(String, '~/request', self._request, 10)
        self.create_timer(0.04, self.forward)
        self.create_timer(0.5, self.check_duplicate)
        self.create_timer(1.0, lambda: self.publish_state(force=True))
        self._last_state = None

    def check_duplicate(self):
        """Hold at zero while another node also publishes ``/cmd_vel/applied``.

        Two publishers interleave frames and the driver obeys the newest, so
        a hold on one is defeated by the other.  Our own publisher counts one.
        """
        other = self.count_publishers('/cmd_vel/applied') > 1
        if other != self.arbitration.duplicate:
            self.get_logger().error(
                'another publisher on /cmd_vel/applied: holding at zero' if other
                else 'duplicate publisher gone: resuming')
            self.arbitration.set_duplicate(other, time.monotonic())

    def forward(self):
        """Publish the winning command now; also the 25 Hz keepalive."""
        now = time.monotonic()
        _, cmd = self.arbitration.select(now)
        self.applied.publish(twist_from(cmd))
        self.publish_state()

    def publish_state(self, force=False):
        """Publish ``~/state`` when it changed, or always with ``force``."""
        state = self.arbitration.state(time.monotonic())
        text = json.dumps(state, sort_keys=True)
        if force or text != self._last_state:
            self._last_state = text
            self.state_pub.publish(String(data=text))

    def _rc(self, message):
        self.arbitration.update_rc(list(message.channels), time.monotonic())
        self.forward()

    def _manual(self, message):
        self.arbitration.update_manual(twist_values(message), time.monotonic())
        self.forward()

    def _auto(self, message):
        self.arbitration.update_auto(twist_values(message), time.monotonic())

    def _request(self, message):
        try:
            request = json.loads(message.data)
            ok, reason = self.arbitration.request(
                request['op'], request.get('cause', ''), request.get('by', ''),
                time.monotonic(), request.get('token'))
        except (ValueError, KeyError, TypeError) as error:
            ok, reason = False, str(error)
        if not ok:
            self.get_logger().warning(f'mux request rejected: {reason}')
        elif request.get('op') == 'release':
            # Journal event; a tokenless release is the U semantics (clears all).
            self.get_logger().info(
                f"hold_release by={request.get('by', '')!r} "
                f"cause={request.get('cause', '')!r} "
                f"tokenless={request.get('token') is None}")
        self.forward()


def main(args=None):
    run_node(CommandMux, args)


if __name__ == '__main__':
    main()
