"""Mix ROS effort commands into thruster pulses: ODROID PWM pins or a simulator topic.

Input: ``topics.applied`` (Twist effort, the command_mux output) goes through the decoupling,
mixer, reversals and channel map (thruster_mixer, docs/CONTROL.md) to four pulses. Without a
message for ``command_timeout`` the outputs go neutral.

Output: the pulses on the configured backend, and always on ``pwm_topic`` (UInt16MultiArray,
physical outputs 1-4, us) at 20 Hz: with the topic backend that topic is the output itself,
with the odroid backend it is the pins' readback, so a bag always holds the actuation.

Safety: with ``estop_cuts_power`` the RC stop-channel edge on ``topics.rc`` cuts the MOSFET and
latches (estop.py). The ``~/estop`` service (std_srvs/Trigger) is the operator's e-stop: the
same latched cut. ``~/clear`` (std_srvs/Trigger, ``astro estop clear``) powers the ESCs again at
neutral; it refuses while the stop switch is still down. Losing RC never cuts: the mux falls through and the command timeout brings neutral. SIGTERM shuts down like Ctrl+C. ``mosfet_heartbeat`` toggles the enable for a
heartbeat circuit. ``~/status`` (latched String JSON) reports the MOSFET, the e-stop and the
live correction.

Config: read once at start (``config_file``, ``-p`` overrides win). Only the correction C
(``decoupling``, ``decoupling_negative``) changes live through ``ros2 param set``, validated as
a batch; a ``decoupling`` set alone moves ``decoupling_negative`` with it while the two are equal.
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
from std_msgs.msg import Float32MultiArray, String, UInt16MultiArray
from std_srvs.srv import Trigger

from mhseals_hardware.configuration import (
    board_unchanged, config_sha, declared, load_layers, plain, validate_config,
)
from mhseals_hardware.estop import RemoteEstop
from mhseals_hardware.outputs import OUTPUT_KEYS, make_outputs
from mhseals_hardware.ros_util import run_node, twist_values
from mhseals_hardware.thruster_mixer import (
    single_allowed, single_pulses,
    thruster_pulses, validate_channel_map, validate_decoupling, validate_mixer,
    validate_reverse_ratio, validate_reversed,
)

READBACK_HZ = 20.0  # pwm_topic rate when the outputs are pins (the topic backend repeats itself)
HEARTBEAT_STALL = 0.15  # s between flips; under the heartbeat circuit's ~0.22 s timeout
# The only settings that change in a running driver, through SetParameters.
PARAMETER_KEYS = ('decoupling', 'decoupling_negative')
STATUS_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class ThrusterPWMNode(Node):
    """Fail-neutral bridge from the mux effort to the configured thruster outputs."""

    def __init__(self):
        super().__init__('thruster_pwm_node')
        # Every flat hardware setting is a ROS parameter defaulting to the loaded config; the
        # mappings (command_mux, controller, characterization) come from the YAML only.
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value or None
        loaded, self.map_source, self.profile = load_layers(config_file)  # the ready line names both
        self.config = validate_config(board_unchanged(declared(self, loaded)))
        if self.config['channel_map'] != loaded['channel_map']:
            self.map_source = '-p channel_map'
        self.declared = {key for key, value in self.config.items() if not isinstance(value, dict)}
        self.session = uuid.uuid4().hex[:8]
        self.status_pub = self.create_publisher(String, '~/status', STATUS_QOS)
        self.apply(self.config)
        # Opening arms the ESCs at neutral; destroy_node disarms them.
        self.outputs = make_outputs(self.config, node=self).open()
        try:
            self._start()
        except BaseException:
            self.outputs.close()  # run_node never gets a node to destroy
            raise

    def _start(self):
        """Everything after the outputs open; a failure here closes them again."""
        self.estop = RemoteEstop(self.outputs, self.config, self.config['estop_cuts_power'])
        self.last_beat = None
        topics = self.config['topics']
        if self.config['mosfet_heartbeat'] and getattr(self.outputs, 'mosfet', None):
            # Two flips per cycle; a stopped, killed or hung node stops the heartbeat.
            self.create_timer(0.5 / self.config['mosfet_heartbeat'], self.heartbeat)
        # Operator e-stop: the same latched cut, cause 'operator'.
        self.create_service(Trigger, '~/estop', self.estop_service)
        self.create_service(Trigger, '~/clear', self.clear_service)
        self.last_command_time = time.monotonic()
        self.timed_out = False
        self.publish_status()
        if self.estop.enabled:
            self.create_subscription(RCIn, topics['rc'], self.rc_callback,
                                     qos_profile_sensor_data)
        self.create_subscription(Twist, topics['applied'], self.cmd_vel_callback, 10)
        # The one-thruster test (keyboard 1-4): [index, effort] on topics.single, used only while the
        # mux applies zero from the keyboard lane or none (single_allowed); its own age limits it.
        self.single, self.single_time, self.mux_state = None, None, None
        if topics.get('single'):
            self.create_subscription(Float32MultiArray, topics['single'], self.single_callback, 10)
            self.create_subscription(String, topics['mux_state'], self.mux_callback, STATUS_QOS)
        if self.config['output_backend'] != 'topic':
            # The topic backend publishes pwm_topic itself; pins publish their readback there.
            self.readback_pub = self.create_publisher(UInt16MultiArray, self.config['pwm_topic'], 10)
            self.create_timer(1.0 / READBACK_HZ, self.publish_readback)
        self.add_pre_set_parameters_callback(self.follow_correction)
        self.add_on_set_parameters_callback(self.check_parameters)
        self.add_post_set_parameters_callback(self.commit_parameters)
        self.create_timer(0.1, self.watchdog_callback)
        self.get_logger().info(
            f"thruster control ready ({self.config['output_backend']} outputs, channel_map "
            f"{self.config['channel_map']} from {self.map_source}, "
            f"vessel thrusters: {self.profile or 'none'})")

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

    def follow_correction(self, parameters):
        """A C set alone carries C- along while C- follows it, as an unset YAML C- does."""
        names = {parameter.name for parameter in parameters}
        if ('decoupling' not in names or 'decoupling_negative' in names
                or list(self.config['decoupling_negative']) != list(self.config['decoupling'])):
            return parameters
        source = next(parameter for parameter in parameters if parameter.name == 'decoupling')
        return list(parameters) + [Parameter('decoupling_negative', source.type_, source.value)]

    def check_parameters(self, parameters):
        """Live keys are validated as a batch; every other setting applies at the next start."""
        changes = {}
        for parameter in parameters:
            if parameter.name in PARAMETER_KEYS:
                changes[parameter.name] = plain(parameter.value)
            elif parameter.name in self.declared | {'config_file'}:
                return SetParametersResult(
                    successful=False, reason='applies at the next start; edit hardware.yaml')
        try:
            self.live_config(changes)
        except (ValueError, TypeError, KeyError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def commit_parameters(self, parameters):
        """Runs once the batch is stored: the driver uses what GetParameters now reads."""
        changes = {parameter.name: plain(parameter.value) for parameter in parameters
                   if parameter.name in PARAMETER_KEYS}
        if not changes:
            return
        self.config = self.live_config(changes)
        self.apply(self.config)
        self.get_logger().info('parameters applied live: ' + ', '.join(
            f'{key}={value}' for key, value in changes.items()))
        self.publish_status()

    def live_config(self, changes):
        """The running config with ``changes``, fully validated; raises on a bad one."""
        return validate_config(dict(self.config, **changes))

    def publish_status(self):
        """Latched ~/status: what the driver runs (MOSFET, e-stop, config hash, live C)."""
        mosfet = getattr(self.outputs, 'mosfet', None)
        state = ('none' if mosfet is None else
                 'on' if mosfet.request is not None and mosfet.enabled else 'off')
        self.status_pub.publish(String(data=json.dumps({
            'session': self.session, 'mosfet': state, 'estop': self.estop.state(),
            'config_sha': config_sha(self.config),  # the snapshot, -p and live sets included
            'outputs': {key: self.config[key] for key in OUTPUT_KEYS},
            'live': {key: list(self.config[key]) if isinstance(self.config[key], (list, tuple))
                     else self.config[key] for key in PARAMETER_KEYS}})))

    def cmd_vel_callback(self, message):
        # Twist effort -> per-thruster effort -> pulse widths -> physical outputs.
        values = twist_values(message)
        if not all(math.isfinite(value) for value in values):
            # A NaN or inf must not crash the node (and with it RC): go neutral.
            self.outputs.neutral()
            self.get_logger().warning('non-finite cmd_vel; thrusters neutral',
                                      throttle_duration_sec=1.0)
            return
        age = None if self.single_time is None else time.monotonic() - self.single_time
        if self.single is not None and single_allowed(values, self.mux_state, age):
            self.drive(single_pulses(self.single[0], self.single[1], self.channel_map, self.reversed))
            return
        self.drive(thruster_pulses(values, self.mixer, self.channel_map,
                                   self.reversed, self.decoupling, self.decoupling_negative,
                                   self.reverse_ratio))

    def single_callback(self, message):
        """[index, effort]: run one thruster alone (checked again on every applied command)."""
        data = list(message.data)
        if len(data) == 2 and all(math.isfinite(v) for v in data) and 0 <= data[0] < 4:
            self.single, self.single_time = (int(data[0]), float(data[1])), time.monotonic()
        else:
            self.single = None

    def mux_callback(self, message):
        try:
            state = json.loads(message.data)
        except ValueError:
            state = None
        self.mux_state = state if isinstance(state, dict) else None

    def rc_callback(self, message):
        """Remote e-stop: the stop-channel edge cuts the MOSFET and latches until ~/clear."""
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
                                    'latched; astro estop clear restores it')
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

    def clear_service(self, _request, response):
        """Clear the latch (astro estop clear); refused while the stop switch is down."""
        try:
            response.success, response.message = self.estop.clear()
        except Exception as error:
            response.success, response.message = False, f'clear failed: {error}'
        if response.success:
            self.last_beat = None    # a fresh heartbeat, not a stall since the cut
            self.get_logger().warning(f'e-stop {response.message}')
            self.publish_status()
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
            return  # cut and neutral until ~/clear
        self.outputs.set_pulse_widths(pulses)
        self.last_command_time = time.monotonic()
        self.timed_out = False

    def publish_readback(self):
        """pwm_topic: the pulses the pins hold now, read back from them."""
        try:
            pulses = [int(pulse) for pulse in self.outputs.readback()]
        except Exception as failure:  # an unreadable pin must not stop the driver
            self.get_logger().warning(f'pwm readback failed: {failure}', throttle_duration_sec=5.0)
            return
        self.readback_pub.publish(UInt16MultiArray(data=pulses))

    def watchdog_callback(self):
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
        # A second Ctrl+C or a SIGTERM must not cut the MOSFET-off and neutral writes short.
        previous = {}
        for signum in (signal.SIGINT, signal.SIGTERM):
            try:
                previous[signum] = signal.signal(signum, signal.SIG_IGN)
            except ValueError:  # not the main thread: nothing to ignore there
                pass
        try:
            self.outputs.close()
        finally:
            try:
                super().destroy_node()
            finally:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)


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
