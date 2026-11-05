#!/usr/bin/env python3

"""Choose which command reaches the driver: rc > manual > hold > auto.

All rules live in ``command_arbitration.Arbitration``; this node only wires
the ``topics`` section of the YAML. Every input is effort: ``topics.rc``, ``topics.manual`` and
``topics.effort`` (the auto lane: the velocity controller's output, or an effort publisher). The
winner goes to ``topics.applied``, which the driver mixes. A human input is forwarded in the
callback that receives it, and a 25 Hz timer keeps the output alive between frames.

The state contract: ``topics.mux_state`` (latched, on a change and at 1 Hz) carries JSON with
``schema`` 'mhseals.mux/1' and ``input``, the lane that drives now: ``rc``, ``manual``,
``hold``, ``auto`` or ``idle`` (nothing fresh). A consumer reads only ``input``
(``command_arbitration.mux_input``): the driver takes test pulses only while it is ``idle``, the
velocity controller drives only while it is ``auto``, and a watchdog is outranked by anything
but ``auto`` and ``idle``. Another schema or a missing input is unknown, and unknown fails safe:
no pulse, a reset controller, outranked. Every other field is for display.

The node reads the ``command_mux`` section of its arm snapshot, plus the top-level
``command_timeout`` the driver and the controller also read, once (``-p`` overrides win) and
never watches the YAML. Only ``rc_scale``, ``command_timeout`` and ``ramp`` change live, through
SetParameters: validated, then applied at once. Every other edit applies at the next arm.
"""

import json
import time

from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String

from mhseals_hardware import hold_store
from mhseals_hardware.command_arbitration import Arbitration, INIT_KEYS, MUX_SCHEMA, mux_section
from mhseals_hardware.configuration import declared, load_config
from mhseals_hardware.ros_util import run_node, twist_from, twist_values

STATE_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


LIVE_PARAMETERS = ('rc_scale', 'command_timeout', 'ramp')


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
    """Publish ``topics.applied`` and the latched ``topics.mux_state``."""

    def __init__(self):
        super().__init__('command_mux')
        self.declare_parameter('config_file', '')
        # The mux takes only the command_mux and topics sections.
        config = load_config(self.get_parameter('config_file').value or None)
        section, self.topics = mux_section(config), config['topics']
        self.section = declared(self, section, LIVE_PARAMETERS)
        self.arbitration = Arbitration.from_config(self.section)
        self.add_on_set_parameters_callback(self._check_parameters)
        self.add_post_set_parameters_callback(self._commit_parameters)
        # Holds survive a restart: re-adopt the saved set (invariant 4).
        self.state_file = section.get('state_file') or hold_store.default_path()
        self.arbitration.adopt_holds(hold_store.load(self.state_file), time.monotonic())
        self._saved = None

        topics = self.topics
        self.applied = self.create_publisher(Twist, topics['applied'], 10)
        self.state_pub = self.create_publisher(String, topics['mux_state'], STATE_QOS)
        self.create_subscription(RCIn, topics['rc'], self._rc, qos_profile_sensor_data)
        self.create_subscription(Twist, topics['manual'], self._manual, 10)
        self.create_subscription(Twist, topics['effort'], self._auto, 10)
        self.create_subscription(String, '~/request', self._request, 10)
        self.create_timer(0.04, self.forward)
        self.create_timer(0.5, self.check_duplicate)
        self.create_timer(1.0, lambda: self.publish_state(force=True))
        self._last_state = None
        self._seq = 0
        self._pipeline_ms = None

    @staticmethod
    def _live_changes(parameters):
        return {p.name: list(p.value) if p.name == 'rc_scale' else p.value
                for p in parameters if p.name in LIVE_PARAMETERS}

    def _check_parameters(self, parameters):
        """Live tunables are validated against the section they would join; config_file is fixed."""
        if any(p.name == 'config_file' for p in parameters):
            return SetParametersResult(successful=False, reason='applies at the next arm: config_file')
        changes = self._live_changes(parameters)
        if changes:
            try:
                Arbitration.from_config({**self.section, **changes})
            except (TypeError, ValueError) as error:
                return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _commit_parameters(self, parameters):
        """Apply a validated set at once: these keys never remap a channel."""
        changes = self._live_changes(parameters)
        if not changes:
            return
        self.section = {**self.section, **changes}
        fresh = Arbitration.from_config(self.section)
        for key in changes:
            setattr(self.arbitration, key, getattr(fresh, key))
        self.get_logger().info('command_mux parameters set: ' + ', '.join(
            f'{key}={value}' for key, value in changes.items()))

    def check_duplicate(self):
        """Hold at zero while another node also publishes ``topics.applied``.

        Two publishers interleave frames and the driver obeys the newest, so
        a hold on one is defeated by the other.  Our own publisher counts one.
        """
        other = self.count_publishers(self.topics['applied']) > 1
        if other != self.arbitration.duplicate:
            self.get_logger().error(
                f"another publisher on {self.topics['applied']}: holding at zero" if other
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

    def publish_state(self, force=False):
        """Publish ``topics.mux_state`` when it changed, or always with ``force``."""
        state = self.arbitration.state(time.monotonic(), time.time_ns(),
                                       self._pipeline_ms)
        state['configuration'] = {
            'effective': {key: getattr(self.arbitration, key) for key in INIT_KEYS},
            'state_file': str(self.state_file)}
        key = change_key(state)
        if force or key != self._last_state:
            self._last_state = key
            self._seq += 1
            state.update(schema=MUX_SCHEMA, seq=self._seq, t_ns=time.time_ns())
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
            # Journal event; the release rule is Arbitration.request's.
            self.get_logger().info(
                f"hold_release by={request.get('by', '')!r} "
                f"cause={request.get('cause', '')!r} "
                f"tokenless={request.get('token') is None}")
        self.forward()


def main(args=None):
    run_node(CommandMux, args)


if __name__ == '__main__':
    main()
