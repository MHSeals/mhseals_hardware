"""Mix ROS effort commands into thruster pulses: ODROID PWM pins or a simulator topic.

Inputs:
  topics.applied       Twist effort (the command_mux output), through the decoupling, mixer,
                       reversals and channel map
  ~/test_pulses        UInt16MultiArray raw pulses per physical output (bench tests), held
                       for ``command_timeout`` after each message
  ~/test_pulses/held   String JSON {"pulses": [4 us], "hold_s": s, "seq": n}: the same, held
                       for ``hold_s`` (at most TEST_HOLD_MAX_S) after each message, so a slow
                       or stalled publisher does not chop the pulse
Test pulses are accepted only while ``topics.mux_state`` reports input ``idle`` (the contract
in command_mux), so they never outvote a human or autonomy; no fresh or known state, no pulse.
A live test pulse owns the outputs: the mux forwards its output (zeros too) at 25 Hz, which
would otherwise overwrite the pulse within 40 ms. It ends, to neutral, when its hold runs
out, on an all-neutral pulse, or at once when the guard reports another input (manual, rc,
auto, a hold) or goes stale or unknown, or a mux frame is not neutral (a takeover whose mux
state has not arrived yet: that frame drives); the mux drives again after. It goes neutral after
``command_timeout`` without a message. ``~/output`` (String JSON, 10 Hz and on each test
change) reports what the outputs hold, read back from them, its source, the test seq, the last
refused pulse and the last ended test with their reasons (e-stop latched, mux not idle, guard
stale, a bad pulse), and the code version this process runs (version.py). With
``estop_cuts_power``, the RC stop-channel edge on ``topics.rc`` cuts the
MOSFET and latches: this process never powers the ESCs again (see estop.py);
``estop_on_rc_loss`` does the same when RC controlling the boat is lost. SIGTERM
shuts down like Ctrl+C. ``mosfet_heartbeat`` toggles the enable for a
heartbeat circuit. The ``~/estop`` service (std_srvs/Trigger) is the operator's
e-stop: the same latched cut. The node reads its arm snapshot (``config_file``, with
``-p`` overrides) once and never watches the YAML: it lives one arm. Only the correction C
(``decoupling``, ``decoupling_negative``) changes live, through SetParameters, validated as a
batch: a bad value is refused and the old one kept. A ``decoupling`` set alone moves ``decoupling_negative`` with it while the two are equal (unset
in the YAML). Every other edit applies at the next arm; this node never reopens outputs.
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

from mhseals_hardware import vessel
from mhseals_hardware.command_arbitration import mux_input
from mhseals_hardware.configuration import (
    board_unchanged, channel_map_source, config_sha, declared, load_config, plain, validate_config,
)
from mhseals_hardware.estop import RemoteEstop
from mhseals_hardware.outputs import OUTPUT_KEYS, make_outputs
from mhseals_hardware.ros_util import run_node, twist_values
from mhseals_hardware.version import code_version
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, NEUTRAL_PWM, thruster_pulses, validate_channel_map, validate_decoupling, validate_mixer,
    validate_reverse_ratio, validate_reversed,
)

PULSE_INPUTS = ('idle',)  # the mux input that leaves the thrusters to nobody
GUARD_MAX_AGE = 3.0  # s; the mux republishes ~/state at 1 Hz
TEST_HOLD_MAX_S = 5.0  # s; the longest a held test pulse lasts without another message
HEARTBEAT_STALL = 0.15  # s between flips; under the heartbeat circuit's ~0.22 s timeout
# The only settings that change in a running driver, through SetParameters.
PARAMETER_KEYS = ('decoupling', 'decoupling_negative')
STATUS_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
GUARD_QOS = STATUS_QOS  # the mux ~/state is latched the same way


class ThrusterPWMNode(Node):
    """Fail-neutral bridge from the mux effort to the configured thruster outputs."""

    def __init__(self):
        super().__init__('thruster_pwm_node')
        # Every flat hardware setting is a ROS parameter defaulting to the arm snapshot; the
        # mappings (command_mux, controller, characterization) come from the YAML only.
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value or None
        self.config = validate_config(board_unchanged(declared(self, load_config(config_file))))
        self.map_source = channel_map_source(config_file)  # the layer the ready line names
        self.declared = {key for key, value in self.config.items() if not isinstance(value, dict)}
        self.session = uuid.uuid4().hex[:8]
        self.status_pub = self.create_publisher(String, '~/status', STATUS_QOS)
        self.apply(self.config)
        # Before power: nothing slow runs between opening the outputs and the deadman.
        self.version = code_version()
        # Opening arms the ESCs at neutral; destroy_node disarms them.
        self.outputs = make_outputs(self.config, node=self).open()
        try:
            self._start()
        except BaseException:
            self.outputs.close()  # run_node never gets a node to destroy
            raise

    def _start(self):
        """Everything after the outputs open; a failure here closes them again."""
        self.estop = RemoteEstop(self.outputs, self.config,
                                 self.config['estop_cuts_power'], self.config['estop_on_rc_loss'])
        self.last_beat = None
        topics = self.config['topics']
        if self.config['mosfet_heartbeat'] and getattr(self.outputs, 'mosfet', None):
            # Two flips per cycle; a stopped, killed or hung node stops the heartbeat.
            self.create_timer(0.5 / self.config['mosfet_heartbeat'], self.heartbeat)
        # Operator e-stop (the TUI): the same latched cut, cause 'operator'.
        self.create_service(Trigger, '~/estop', self.estop_service)
        self.last_command_time = time.monotonic()
        self.timed_out = False
        self.publish_status()
        if self.estop.enabled or self.estop.on_rc_loss:
            self.create_subscription(RCIn, topics['rc'], self.rc_callback,
                                     qos_profile_sensor_data)
        self.create_subscription(Twist, topics['applied'], self.cmd_vel_callback, 10)
        self.test = None  # the live test pulse: {pulses, seq, until}
        self.refused = self.ended = None  # the last refused pulse and ended test: {reason, seq, t_ns}
        self.source = 'neutral'  # what last wrote the outputs: cmd_vel, test, neutral, estop
        self.output_pub = self.create_publisher(String, '~/output', 10)
        self.create_subscription(UInt16MultiArray, '~/test_pulses', self.pulses_callback, 10)
        self.create_subscription(String, '~/test_pulses/held', self.held_pulses_callback, 10)
        self.guard_topic = topics['mux_state']
        self.mux_input, self.mux_time = None, 0.0
        self.create_subscription(String, self.guard_topic, self.guard_callback, GUARD_QOS)
        self.add_pre_set_parameters_callback(self.follow_correction)
        self.add_on_set_parameters_callback(self.check_parameters)
        self.add_post_set_parameters_callback(self.commit_parameters)
        self.create_timer(0.1, self.watchdog_callback)
        profile = vessel.directory() if vessel.section('thrusters') else 'none'
        self.get_logger().info(
            f"thruster control ready ({self.config['output_backend']} outputs, channel_map "
            f"{self.config['channel_map']} from {self.map_source}, "
            f"vessel thrusters: {profile})")

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
        """Live keys are validated as a batch; every other setting applies at the next arm."""
        changes = {}
        for parameter in parameters:
            if parameter.name in PARAMETER_KEYS:
                changes[parameter.name] = plain(parameter.value)
            elif parameter.name in self.declared | {'config_file'}:
                return SetParametersResult(
                    successful=False, reason='applies at the next arm; edit hardware.yaml')
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
        """Latched ~/status: what the driver runs, for the TUI's owner and sync checks."""
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
            self.end_test('non-finite cmd_vel')
            self.source = 'neutral'
            return
        if self.test is not None:
            refusal = self.pulse_refusal()
            if refusal is None and not any(values):
                return  # the test pulse owns the outputs until it ends
            self.end_test(refusal or 'a cmd_vel command took over')
        self.drive(thruster_pulses(values, self.mixer, self.channel_map,
                                   self.reversed, self.decoupling, self.decoupling_negative,
                                   self.reverse_ratio))

    def pulses_callback(self, message):
        """Raw pulses per physical output, for bench tests, held for command_timeout."""
        self.test_pulse(list(message.data), self.command_timeout, None)

    def held_pulses_callback(self, message):
        """Raw pulses per output held for the message's hold_s, tagged with its seq."""
        try:
            request = json.loads(message.data)
            pulses, hold = list(request['pulses']), float(request['hold_s'])
            seq = request.get('seq')
            if not (math.isfinite(hold) and hold > 0):
                raise ValueError(f'hold_s must be positive, got {hold}')
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.refuse(f'bad test pulse message: {error}', None)
            return
        self.test_pulse(pulses, min(hold, TEST_HOLD_MAX_S), seq)

    def refuse(self, reason, seq):
        """Drop a test pulse with its reason: logged (throttled) and kept for ~/output."""
        self.get_logger().warning(f'test pulses rejected: {reason}', throttle_duration_sec=1.0)
        self.refused = dict(reason=reason, seq=seq, t_ns=time.time_ns())
        self.publish_output()

    def test_pulse(self, pulses, hold, seq):
        """Take the outputs for ``hold`` seconds (all neutral ends a test); validated here."""
        if self.estop.latched:
            self.refuse(self.estop_reason(), seq)
            return
        refusal = self.pulse_refusal()
        if refusal:
            self.refuse(refusal, seq)
            return
        if (len(pulses) != 4 or any(isinstance(pulse, bool) or not isinstance(pulse, int)
                                    or not MIN_PWM <= pulse <= MAX_PWM for pulse in pulses)):
            self.refuse(f'need four pulses within {MIN_PWM}-{MAX_PWM} us, got {pulses}', seq)
            return
        if all(pulse == self.outputs_neutral() for pulse in pulses):
            self.end_test(None)
            self.drive(pulses, 'neutral')
            return
        changed = self.test is None or self.test['pulses'] != pulses or self.test['seq'] != seq
        self.test = dict(pulses=pulses, seq=seq, until=time.monotonic() + hold)
        self.drive(pulses, 'test')
        if changed:
            self.publish_output()

    def estop_reason(self):
        return f'e-stop latched ({self.estop.cause}): ESC power is cut until re-armed'

    def outputs_neutral(self):
        return int(getattr(self.outputs, 'neutral_us', NEUTRAL_PWM))

    def end_test(self, reason):
        """End a live test pulse at neutral; ``reason`` None: its sender ended it."""
        if self.test is None:
            return
        seq, self.test = self.test['seq'], None
        self.ended = dict(reason=reason or 'ended by its sender', seq=seq, t_ns=time.time_ns())
        if reason:
            self.get_logger().warning(f'test pulse ended: {reason}; thrusters neutral')
        if not self.estop.latched:
            self.outputs.neutral()
            self.source = 'neutral'
            # Neutral now; cmd_vel takes over at its next frame, the deadman runs from here.
            self.last_command_time = time.monotonic()
            self.timed_out = False
        self.publish_output()

    def guard_callback(self, message):
        self.mux_input = mux_input(message.data)
        self.mux_time = time.monotonic()
        if self.test is not None:
            refusal = self.pulse_refusal()
            if refusal:
                self.end_test(refusal)  # a human or a hold takes over at once

    def pulse_refusal(self):
        """Why test pulses are refused now, or None while the mux is idle."""
        if time.monotonic() - self.mux_time > GUARD_MAX_AGE:
            return f'guard stale: no fresh {self.guard_topic} (is command_mux running?)'
        if self.mux_input is None:
            return f'guard unknown: {self.guard_topic} is not a mhseals.mux/1 state with an input'
        if self.mux_input not in PULSE_INPUTS:
            return f'command_mux input is {self.mux_input}, not idle'
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

    def drive(self, pulses, source='cmd_vel'):
        if self.estop.latched:
            return  # cut and neutral; only a new driver (the arm path) powers on again
        self.outputs.set_pulse_widths(pulses)
        self.source = source
        self.last_command_time = time.monotonic()
        self.timed_out = False

    def publish_output(self):
        """~/output: what the outputs hold now (read back from them), its source, the test."""
        try:
            pulses, error = [int(pulse) for pulse in self.outputs.readback()], ''
        except Exception as failure:  # an unreadable pin must not stop the driver
            pulses, error = None, str(failure)
        test = self.test
        self.output_pub.publish(String(data=json.dumps({
            'session': self.session, 't_ns': time.time_ns(), 'pulses': pulses, 'error': error,
            'source': 'estop' if self.estop.latched else 'test' if test else self.source,
            'test': None if test is None else {
                'pulses': test['pulses'], 'seq': test['seq'],
                'remaining_s': round(max(0.0, test['until'] - time.monotonic()), 3)},
            'refused': self.refused, 'ended': self.ended, 'version': self.version})))

    def watchdog_callback(self):
        if self.estop.enabled or self.estop.on_rc_loss:
            self.estop_step(self.estop.tick)  # RC gone stale while it controlled the boat
        self.retry_cut()
        if self.estop.latched and self.test is not None:
            self.end_test(self.estop_reason())
        if self.test is not None:
            # The test pulse has its own deadman: its hold, and the guard staying fresh.
            refusal = self.pulse_refusal()
            if refusal or time.monotonic() > self.test['until']:
                self.end_test(refusal or 'hold ran out (no test pulse message)')
        # Fail neutral when the command stream stops (crashed teleop, lost link).
        elif (not self.timed_out and time.monotonic() - self.last_command_time >
                self.command_timeout):
            self.outputs.neutral()
            self.source = 'neutral'
            self.timed_out = True
            self.get_logger().warning('commands timed out; thrusters neutral')
        self.publish_output()

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
