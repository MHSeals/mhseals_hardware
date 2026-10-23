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

from mhseals_hardware import hold_store
from mhseals_hardware.command_arbitration import Arbitration
from mhseals_hardware.configuration import ConfigWatcher, load_config
from mhseals_hardware.ros_util import run_node, twist_from, twist_values

STATE_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


RELOAD_PERIOD = 1.0


# Fields that move on every call (ages, rates, wall conversions, timing).
VOLATILE_RC = ('age_s', 'rate_hz', 'interval_p95_s')


def change_key(state):
    """The state body without volatile fields, as a comparable string.

    authority.md 2.1 publishes on change plus 1 Hz: ages, rates, pipeline_ms
    and the jittering ``since_ns`` conversions must not count as a change.
    """
    body = {k: v for k, v in state.items()
            if k not in ('since_ns', 'pipeline_ms', 'seq', 't_ns')}
    body['rc'] = {k: v for k, v in state['rc'].items() if k not in VOLATILE_RC}
    body['manual'] = {k: v for k, v in state['manual'].items() if k != 'age_s'}
    body['auto'] = {k: v for k, v in state['auto'].items() if k != 'age_s'}
    if state.get('hold'):
        body['hold'] = {k: v for k, v in state['hold'].items() if k != 'since_ns'}
    body['holds'] = [{k: v for k, v in h.items() if k != 'since_ns'}
                     for h in state['holds']]
    return json.dumps(body, sort_keys=True)


class CommandMux(Node):
    """Publish ``/cmd_vel/applied`` and the latched ``~/state``."""

    def __init__(self):
        super().__init__('command_mux')
        self.declare_parameter('rc_scale', [0.5, 0.5, 0.3])
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('ramp', 1.0)
        self.declare_parameter('config_file', '')
        self.arbitration = Arbitration(
            rc_scale=self.get_parameter('rc_scale').value,
            cmd_timeout=self.get_parameter('cmd_timeout').value,
            ramp=self.get_parameter('ramp').value)
        # The hardware YAML's command_mux section overrides the parameters.
        config_file = self.get_parameter('config_file').value or None
        self.pending = None
        section = load_config(config_file).get('command_mux', {})
        self.arbitration.apply_config(section, time.monotonic())
        # Holds survive a restart: re-adopt the saved set (invariant 4).
        self.state_file = section.get('state_file') or hold_store.default_path()
        self.arbitration.adopt_holds(hold_store.load(self.state_file), time.monotonic())
        self._saved = None
        self.watch = ConfigWatcher(config_file)
        self.create_timer(RELOAD_PERIOD, self.reload_callback)

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
        self._seq = 0
        self._pipeline_ms = None

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
        self.persist_holds()
        self.applied.publish(twist_from(cmd))
        self.publish_state()

    def persist_holds(self):
        """Write the hold set when it changed, before the command goes out."""
        holds = self.arbitration.holds
        text = json.dumps(holds, sort_keys=True)
        if text == self._saved:
            return
        try:
            hold_store.save(self.state_file, holds)
            self._saved = text
        except OSError as error:
            self.get_logger().error(f'cannot save holds to {self.state_file}: {error}')

    def reload_callback(self):
        """Pick up an edited ``command_mux`` section; hold it pending while driving."""
        try:
            config = self.watch.poll()
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().error(f'config reload rejected, keeping the old one: {error}')
            return
        if config is not None:
            self.pending = config.get('command_mux', {})
        if self.pending is None:
            return
        if self.arbitration.apply_config(self.pending, time.monotonic()):
            self.get_logger().info('command_mux config applied')
            self.pending = None
        elif config is not None:
            self.get_logger().info('command_mux config pending until idle or hold')

    def publish_state(self, force=False):
        """Publish ``~/state`` when it changed, or always with ``force``."""
        state = self.arbitration.state(time.monotonic(), time.time_ns(),
                                       self._pipeline_ms)
        key = change_key(state)
        if force or key != self._last_state:
            self._last_state = key
            self._seq += 1
            state.update(schema='mhseals.mux/1', seq=self._seq, t_ns=time.time_ns())
            text = json.dumps(state, sort_keys=True)
            self.state_pub.publish(String(data=text))

    def _rc(self, message):
        start = time.perf_counter()
        self.arbitration.update_rc(list(message.channels), time.monotonic())
        _, cmd = self.arbitration.select(time.monotonic())
        self.persist_holds()
        self.applied.publish(twist_from(cmd))
        # Receipt to applied publish, measured here; reported with the next state.
        self._pipeline_ms = round((time.perf_counter() - start) * 1e3, 3)
        self.publish_state()

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
