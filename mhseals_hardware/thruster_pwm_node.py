"""Mix ROS effort commands into thruster pulses: ODROID PWM pins or a simulator topic.

Two inputs drive the outputs, newest message wins:
  cmd_vel        Twist effort, through the decoupling, mixer, reversals and channel map
  ~/test_pulses  UInt16MultiArray raw pulses per physical output (bench tests)
Test pulses are accepted only while ``pulse_guard_topic`` (the command_mux ``~/state``)
reports input ``auto`` or ``idle``, so they never outvote a human; no fresh state, no pulse.
An empty ``pulse_guard_topic`` disables the guard (bench use without a mux).
Either goes neutral after ``command_timeout`` without a message. With
``estop_cuts_power``, the RC stop-channel edge on ``/mavros/rc/in`` cuts the
MOSFET and latches: this process never powers the ESCs again (see estop.py);
``estop_on_rc_loss`` does the same when RC controlling the boat is lost. SIGTERM
shuts down like Ctrl+C. ``mosfet_heartbeat_hz`` toggles the enable for a
heartbeat circuit. The ``~/estop`` service (std_srvs/Trigger) is the operator's
e-stop: the same latched cut. With
``config_file`` set, validated watchdog edits apply live. Structural or coupled model
edits require an explicit owner-controlled restart; this node never reopens outputs.
"""

import json
import math
import signal
import time
import uuid

from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String, UInt16MultiArray
from std_srvs.srv import Trigger

from mhseals_hardware.configuration import (
    ConfigWatcher, config_sha, startup_config, validate_config,
)
from mhseals_hardware.estop import RemoteEstop
from mhseals_hardware.outputs import OUTPUT_KEYS, make_outputs
from mhseals_hardware.ros_util import run_node, twist_values
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, thruster_pulses, validate_channel_map, validate_decoupling, validate_mixer,
    validate_reverse_ratio, validate_reversed,
)

RELOAD_PERIOD = 1.0
PULSE_INPUTS = ('auto', 'idle')  # mux inputs that leave the thrusters to autonomy or nobody
GUARD_MAX_AGE = 3.0  # s; the mux republishes ~/state at 1 Hz
HEARTBEAT_STALL = 0.15  # s between flips; under the heartbeat circuit's ~0.22 s timeout
# No runtime power/ownership proof permits structural changes in this node lifetime.
RESTART_KEYS = OUTPUT_KEYS + ('frequency', 'channel_map', 'reversed', 'thruster_matrix',
                              'decoupling', 'decoupling_negative', 'reverse_thrust_ratio',
                              'characterization', 'hardware_epoch', 'estop_cuts_power',
                              'estop_on_rc_loss', 'mosfet_heartbeat_hz')
LIVE_KEYS = ('command_timeout', 'command_mux')  # command_mux less its channels (see reload)
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
        config, file_layer = startup_config(self, self.config_file)
        self.requested_config = file_layer
        self.pending_restart_keys = []
        self.file_config = file_layer  # the YAML as loaded, before any parameters
        # Mappings (the characterization) can't be ROS parameters; they come
        # from the YAML only.
        keys = [key for key, value in config.items() if not isinstance(value, dict)]
        for key in keys:
            self.declare_parameter(key, config[key])
        self.config = validate_config(dict(config, **{key: self.get_parameter(key).value
                                                       for key in keys}))
        if self.config != config:
            raise ValueError('ROS overrides conflict with the loaded startup config')
        self.declared = set(keys)
        self.own_write = False  # true only while this node mirrors a reload into parameters
        self.session = uuid.uuid4().hex[:8]
        self.error = ''  # the last rejected reload; empty when the last one applied
        self.status_pub = self.create_publisher(String, '~/status', STATUS_QOS)
        self.apply(self.config)
        # Opening arms the ESCs at neutral; destroy_node disarms them.
        self.outputs = make_outputs(self.config, node=self).open()
        self.estop = RemoteEstop(self.outputs, self.config['command_mux'],
                                 self.config['estop_cuts_power'], self.config['estop_on_rc_loss'])
        self.last_beat = None
        if self.config['mosfet_heartbeat_hz'] and getattr(self.outputs, 'mosfet', None):
            # Two flips per cycle; a stopped, killed or hung node stops the heartbeat.
            self.create_timer(0.5 / self.config['mosfet_heartbeat_hz'], self.heartbeat)
        # Operator e-stop (the TUI): the same latched cut, cause 'operator'.
        self.create_service(Trigger, '~/estop', self.estop_service)
        self.last_command_time = time.monotonic()
        self.timed_out = False
        self.publish_status()
        if self.estop.enabled or self.estop.on_rc_loss:
            self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback,
                                     qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(UInt16MultiArray, '~/test_pulses', self.pulses_callback, 10)
        self.declare_parameter('pulse_guard_topic', '/command_mux/state')
        self.guard_topic = self.get_parameter('pulse_guard_topic').value
        self.mux_input, self.mux_time = None, 0.0
        if self.guard_topic:
            self.create_subscription(String, self.guard_topic, self.guard_callback, GUARD_QOS)
        self.add_on_set_parameters_callback(self.refuse_external)
        self.create_timer(0.1, self.watchdog_callback)
        if self.config_file:
            self.watch = ConfigWatcher(self.config_file)
            self.create_timer(RELOAD_PERIOD, self.reload_callback)
        self.get_logger().info(
            f"thruster control ready ({self.config['output_backend']} outputs)")

    def apply(self, config):
        if isinstance(config['command_timeout'], bool):
            raise ValueError('command_timeout must be numeric, not bool')
        self.command_timeout = float(config['command_timeout'])
        self.channel_map = validate_channel_map(config['channel_map'])
        self.mixer = validate_mixer(config['thruster_matrix'])
        self.decoupling = validate_decoupling(config['decoupling'])
        self.decoupling_negative = validate_decoupling(config['decoupling_negative'])
        self.reversed = validate_reversed(config['reversed'])
        self.reverse_ratio = validate_reverse_ratio(config['reverse_thrust_ratio'])
        if getattr(self, 'estop', None) is not None:
            self.estop.configure(config['command_mux'])

    def refuse_external(self, parameters):
        """The YAML is the one source of truth: settings written over ROS are refused."""
        if self.own_write:
            return SetParametersResult(successful=True)
        for parameter in parameters:
            if parameter.name in self.declared | {'config_file', 'pulse_guard_topic'}:
                reason = 'edit hardware.yaml'
                if parameter.name in RESTART_KEYS + ('config_file', 'pulse_guard_topic'):
                    reason = 'restart required; ' + reason
                elif parameter.name not in LIVE_KEYS:
                    reason = 'not a driver setting; ' + reason
                return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

    def publish_status(self):
        """Latched ~/status: what the driver runs, for the TUI's owner and sync checks."""
        mosfet = getattr(self.outputs, 'mosfet', None)
        state = ('none' if mosfet is None else
                 'on' if mosfet.request is not None and mosfet.enabled else 'off')
        self.status_pub.publish(String(data=json.dumps({
            'session': self.session, 'mosfet': state, 'estop': self.estop.state(),
            'config_sha': config_sha(self.file_config),
            'requested_config_sha': config_sha(self.requested_config),
            'effective_config_sha': config_sha(self.config),
            'pending_restart_keys': self.pending_restart_keys,
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
        self.requested_config = config
        # The mux defers a stop or mode channel remap until nothing drives; this node can't
        # tell when that is, so the e-stop takes those two only on a restart.
        mux, old_mux = config.get('command_mux') or {}, self.file_config.get('command_mux') or {}
        self.pending_restart_keys = sorted(
            [key for key in RESTART_KEYS if config.get(key) != self.file_config.get(key)] +
            ['command_mux.' + key for key in ('mode_ch', 'stop_ch')
             if mux.get(key) != old_mux.get(key)])
        if self.pending_restart_keys:
            self.error = 'restart required: ' + ', '.join(self.pending_restart_keys)
            self.get_logger().error(self.error)
            self.publish_status()
            return
        # Diff against the accepted file so unrelated launch overrides survive.
        changed = {key: config[key] for key in LIVE_KEYS
                   if config[key] != self.file_config.get(key)}
        if isinstance(config['command_timeout'], bool):
            raise ValueError('command_timeout must be numeric, not bool')
        new = validate_config(dict(self.config, **changed))
        self.file_config, self.error = config, ''
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
                                   self.reversed, self.decoupling, self.decoupling_negative,
                                   self.reverse_ratio))

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

    def rc_callback(self, message):
        """Remote e-stop: the stop-channel edge cuts the MOSFET and latches until re-armed."""
        self.estop_step(self.estop.rc, message.channels)

    def estop_step(self, step, *args):
        """One e-stop update (a frame, or the stream aging); report a cut."""
        try:
            cut = step(*args, time.monotonic(), time.time_ns())
        except Exception as error:  # a failed cut is latched; report, keep the node (and RC) alive
            self.get_logger().error(f'remote e-stop cut failed: {error}')
            cut = self.estop.latched
        if cut:
            self.get_logger().error(f'remote e-stop ({self.estop.cause}): ESC power cut and '
                                    'latched; re-arm to restore')
            self.publish_status()

    def estop_service(self, _request, response):
        """Cut and latch now; success means ESC power is off (an earlier cut keeps its cause)."""
        self.estop_step(self.estop.trip, 'operator')
        mosfet = getattr(self.outputs, 'mosfet', None)
        powered = mosfet is not None and mosfet.request is not None and mosfet.enabled
        response.success = self.estop.latched and not powered
        response.message = (f'latched ({self.estop.cause})' if response.success else
                            'cut failed: ESC power may still be on')
        return response

    def heartbeat(self):
        """Flip the enable; a gap long enough for the circuit to drop latches a cut,
        so power never comes back by itself when a stalled node resumes. Latched, it
        never flips, even when the cut's write failed and the line still reads on."""
        if self.estop.latched:
            return
        now = time.monotonic()
        if self.last_beat is not None and now - self.last_beat > HEARTBEAT_STALL:
            self.estop_step(self.estop.trip, 'stall')
        self.last_beat = now
        self.outputs.mosfet.beat()

    def drive(self, pulses):
        if self.estop.latched:
            return  # cut and neutral; only a new driver (the arm path) powers on again
        self.outputs.set_pulse_widths(pulses)
        self.last_command_time = time.monotonic()
        self.timed_out = False

    def watchdog_callback(self):
        if self.estop.enabled or self.estop.on_rc_loss:
            self.estop_step(self.estop.tick)  # RC gone stale while it controlled the boat
        self.retry_cut()
        # Fail neutral when the command stream stops (crashed teleop, lost link).
        if (not self.timed_out and time.monotonic() - self.last_command_time >
                self.command_timeout):
            self.outputs.neutral()
            self.timed_out = True
            self.get_logger().warning('commands timed out; thrusters neutral')

    def retry_cut(self):
        """Latched but the MOSFET write failed, so the line still powers the ESCs: retry."""
        mosfet = getattr(self.outputs, 'mosfet', None)
        if not (self.estop.latched and mosfet is not None and mosfet.request is not None
                and mosfet.enabled):
            return
        try:
            mosfet.set_enabled(False)
        except Exception as error:
            self.get_logger().error(f'e-stop cut retry failed: {error}',
                                    throttle_duration_sec=1.0)
            return
        self.get_logger().error('e-stop cut retry succeeded: ESC power off')
        self.publish_status()

    def destroy_node(self):
        try:
            self.outputs.close()
        finally:
            super().destroy_node()


def stop_on_sigterm(_signum, _frame):
    """SIGTERM runs the same shutdown as Ctrl+C (destroy_node: MOSFET off, neutral).

    A repeat is ignored so it cannot interrupt that cleanup; SIGKILL still can.
    """
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None):
    signal.signal(signal.SIGTERM, stop_on_sigterm)
    run_node(ThrusterPWMNode, args)


if __name__ == '__main__':
    main()
