"""Mix ROS effort commands into thruster pulses: ODROID PWM pins or a simulator topic.

Two inputs drive the outputs, newest message wins:
  cmd_vel        Twist effort, through the decoupling, mixer, reversals and channel map
  ~/test_pulses  UInt16MultiArray raw pulses per physical output (bench tests)
Test pulses are accepted only while ``pulse_guard_topic`` (the command_mux ``~/state``)
reports input ``auto`` or ``idle``, so they never outvote a human; no fresh state, no pulse.
An empty ``pulse_guard_topic`` disables the guard (bench use without a mux).
Either goes neutral after ``command_timeout`` without a message. With
``config_file`` set, edits to the YAML apply live; pin or backend edits move the outputs
(neutral, ESC power off, new pins at neutral, power on).
"""

import json
import math
import time
import uuid

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, UInt16MultiArray

from mhseals_hardware.configuration import (
    ConfigWatcher, config_sha, load_config, validate_config,
)
from mhseals_hardware.outputs import OUTPUT_KEYS, make_outputs, switch_outputs
from mhseals_hardware.ros_util import run_node, twist_values
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, thruster_pulses, validate_channel_map, validate_decoupling, validate_mixer,
    validate_reversed,
)

RELOAD_PERIOD = 1.0
PULSE_INPUTS = ('auto', 'idle')  # mux inputs that leave the thrusters to autonomy or nobody
GUARD_MAX_AGE = 3.0  # s; the mux republishes ~/state at 1 Hz
# Settings the driver applies itself: the mixer inputs, and the pins it can move.
LIVE_KEYS = ('channel_map', 'thruster_matrix', 'decoupling', 'decoupling_negative', 'reversed',
             'command_timeout')
STATUS_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
GUARD_QOS = STATUS_QOS  # the mux ~/state is latched the same way


class ThrusterPWMNode(Node):
    """Fail-neutral bridge from cmd_vel effort to the configured thruster outputs."""

    def __init__(self):
        super().__init__('thruster_pwm_node')
        # Every hardware setting is a ROS parameter, defaulting to the YAML config.
        self.declare_parameter('config_file', '')
        self.config_file = self.get_parameter('config_file').value or None
        config = load_config(self.config_file)
        self.file_config = config  # the YAML as loaded, before any parameters
        # Mappings (the characterization) can't be ROS parameters; they come
        # from the YAML only.
        keys = [key for key, value in config.items() if not isinstance(value, dict)]
        for key in keys:
            self.declare_parameter(key, config[key])
        self.config = validate_config(dict(config, **{key: self.get_parameter(key).value
                                                       for key in keys}))
        self.declared = set(keys)
        self.own_write = False  # true only while this node mirrors a reload into parameters
        self.session = uuid.uuid4().hex[:8]
        self.error = ''  # the last rejected reload; empty when the last one applied
        self.status_pub = self.create_publisher(String, '~/status', STATUS_QOS)
        self.apply(self.config)
        # Opening arms the ESCs at neutral; destroy_node disarms them.
        self.outputs = make_outputs(self.config, node=self).open()
        self.last_command_time = time.monotonic()
        self.timed_out = False
        self.add_on_set_parameters_callback(self.refuse_external)
        self.publish_status()
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(UInt16MultiArray, '~/test_pulses', self.pulses_callback, 10)
        self.declare_parameter('pulse_guard_topic', '/command_mux/state')
        self.guard_topic = self.get_parameter('pulse_guard_topic').value
        self.mux_input, self.mux_time = None, 0.0
        if self.guard_topic:
            self.create_subscription(String, self.guard_topic, self.guard_callback, GUARD_QOS)
        self.create_timer(0.1, self.watchdog_callback)
        if self.config_file:
            self.watch = ConfigWatcher(self.config_file)
            self.create_timer(RELOAD_PERIOD, self.reload_callback)
        self.get_logger().info(
            f"thruster control ready ({self.config['output_backend']} outputs)")

    def apply(self, config):
        self.command_timeout = float(config['command_timeout'])
        self.channel_map = validate_channel_map(config['channel_map'])
        self.mixer = validate_mixer(config['thruster_matrix'])
        self.decoupling = validate_decoupling(config['decoupling'])
        self.decoupling_negative = validate_decoupling(config['decoupling_negative'])
        self.reversed = validate_reversed(config['reversed'])

    def refuse_external(self, parameters):
        """The YAML is the one source of truth: settings written over ROS are refused."""
        if self.own_write:
            return SetParametersResult(successful=True)
        for parameter in parameters:
            if parameter.name in self.declared:
                reason = 'edit hardware.yaml'
                if parameter.name not in LIVE_KEYS + OUTPUT_KEYS:
                    reason = 'not a driver setting; ' + reason
                return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

    def publish_status(self):
        """Latched ~/status: what the driver runs, for the TUI's owner and sync checks."""
        mosfet = getattr(self.outputs, 'mosfet', None)
        state = 'none' if mosfet is None else 'on' if mosfet.request is not None else 'off'
        self.status_pub.publish(String(data=json.dumps({
            'session': self.session, 'mosfet': state, 'config_sha': config_sha(self.file_config),
            'outputs': {key: self.config[key] for key in OUTPUT_KEYS}, 'error': self.error})))

    def reload_callback(self):
        """Apply the YAML's edits; keep running on a bad one."""
        try:
            self.reload()
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.error = str(error)
            self.get_logger().error(f'config reload rejected, keeping the old one: {error}')
            self.publish_status()

    def reload(self):
        config = self.watch.poll()
        if config is None:
            return
        # Diff against the previous file, not self.config, so launch overrides stay.
        changed = {key: value for key, value in config.items()
                   if not isinstance(value, dict) and value != self.file_config.get(key)}
        new = validate_config(dict(self.config, **changed))  # may raise: nothing is applied
        self.file_config, self.error = config, ''
        if not changed:
            self.publish_status()
            return
        if any(key in changed for key in OUTPUT_KEYS):
            self.outputs, error = switch_outputs(self.outputs, self.config, new, node=self)
            self.timed_out = True  # hold neutral until the next command
            if error:
                self.error = f'outputs not moved: {error}'
                self.get_logger().error(f'outputs not moved, kept the old ones: {error}')
                new.update({key: self.config[key] for key in OUTPUT_KEYS})
                # Not applied: leave them out of the log and let the next edit retry them.
                for key in OUTPUT_KEYS:
                    if key in changed:
                        del changed[key]
                        self.file_config[key] = self.config[key]
            else:
                self.get_logger().info('outputs moved: ' + ', '.join(
                    f'{key}={changed[key]}' for key in OUTPUT_KEYS if key in changed))
        self.config = new
        self.apply(new)
        self.sync_parameters({key: new[key] for key in changed if self.has_parameter(key)})
        if changed:
            self.get_logger().info('config reloaded: ' + ', '.join(
                f'{key}={value}' for key, value in changed.items()))
        self.publish_status()

    def sync_parameters(self, values):
        """Mirror applied values into the ROS parameters, matching each one's declared type."""
        parameters = []
        for key, value in values.items():
            declared = self.get_parameter(key).value
            # A hand-edited `1` for a float setting must not be refused as a type change.
            if isinstance(declared, float) and isinstance(value, int) and not isinstance(value, bool):
                value = float(value)
            elif isinstance(declared, (list, tuple)) and declared and isinstance(declared[0], float):
                value = [float(item) if isinstance(item, int) and not isinstance(item, bool) else item
                         for item in value]
            parameters.append(Parameter(key, value=value))
        self.own_write = True
        try:
            results = self.set_parameters(parameters)
        finally:
            self.own_write = False
        for result, parameter in zip(results, parameters):
            if not result.successful:
                self.get_logger().warning(
                    f'parameter {parameter.name} not synced: {result.reason}')

    def cmd_vel_callback(self, message):
        # Twist effort -> per-thruster effort -> pulse widths -> physical outputs.
        values = twist_values(message)
        if not all(math.isfinite(value) for value in values):
            # A NaN or inf must not crash the node (and with it RC): go neutral.
            self.outputs.neutral()
            self.get_logger().warning('non-finite cmd_vel; thrusters neutral',
                                      throttle_duration_sec=1.0)
            return
        self.drive(thruster_pulses(values, self.mixer, self.channel_map,
                                   self.reversed, self.decoupling, self.decoupling_negative))

    def pulses_callback(self, message):
        """Raw pulses per physical output, for bench tests; validated by the outputs."""
        pulses = list(message.data)
        refusal = self.pulse_refusal()
        if refusal:
            self.get_logger().warning(f'test pulses rejected: {refusal}',
                                      throttle_duration_sec=1.0)
            return
        try:
            if len(pulses) != 4 or any(not MIN_PWM <= pulse <= MAX_PWM for pulse in pulses):
                raise ValueError(f'need four pulses within {MIN_PWM}-{MAX_PWM} us, got {pulses}')
            self.drive(pulses)
        except ValueError as error:
            self.get_logger().error(f'test pulses rejected: {error}')

    def guard_callback(self, message):
        try:
            self.mux_input = json.loads(message.data).get('input')
        except (ValueError, AttributeError):
            self.mux_input = None
        self.mux_time = time.monotonic()

    def pulse_refusal(self):
        """Why test pulses are refused now, or None when autonomy holds the thrusters."""
        if not self.guard_topic:
            return None
        if time.monotonic() - self.mux_time > GUARD_MAX_AGE or self.mux_input is None:
            return f'no fresh {self.guard_topic}'
        if self.mux_input not in PULSE_INPUTS:
            return f'command_mux input is {self.mux_input}'
        return None

    def drive(self, pulses):
        self.outputs.set_pulse_widths(pulses)
        self.last_command_time = time.monotonic()
        self.timed_out = False

    def watchdog_callback(self):
        # Fail neutral when the command stream stops (crashed teleop, lost link).
        if (not self.timed_out and time.monotonic() - self.last_command_time >
                self.command_timeout):
            self.outputs.neutral()
            self.timed_out = True
            self.get_logger().warning('commands timed out; thrusters neutral')

    def destroy_node(self):
        try:
            self.outputs.close()
        finally:
            super().destroy_node()


def main(args=None):
    run_node(ThrusterPWMNode, args)


if __name__ == '__main__':
    main()
