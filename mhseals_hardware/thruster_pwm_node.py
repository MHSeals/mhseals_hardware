"""Mix ROS effort commands into thruster pulses: ODROID PWM pins or a simulator topic.

Two inputs drive the outputs, newest message wins:
  cmd_vel        Twist effort, through the decoupling, mixer, reversals and channel map
  ~/test_pulses  UInt16MultiArray raw pulses per physical output (bench tests)
Either goes neutral after ``command_timeout`` without a message. With
``config_file`` set, edits to the YAML apply live; pin or backend edits move the outputs
(neutral, ESC power off, new pins at neutral, power on).
"""

import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import UInt16MultiArray

from mhseals_hardware.configuration import ConfigWatcher, load_config, validate_config
from mhseals_hardware.outputs import OUTPUT_KEYS, make_outputs, switch_outputs
from mhseals_hardware.ros_util import run_node, twist_values
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, thruster_pulses, validate_channel_map, validate_decoupling, validate_mixer,
    validate_reversed,
)

RELOAD_PERIOD = 1.0


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
        self.apply(self.config)
        # Opening arms the ESCs at neutral; destroy_node disarms them.
        self.outputs = make_outputs(self.config, node=self).open()
        self.last_command_time = time.monotonic()
        self.timed_out = False
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(UInt16MultiArray, '~/test_pulses', self.pulses_callback, 10)
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

    def reload_callback(self):
        """Apply the YAML's edits; keep running on a bad one."""
        try:
            config = self.watch.poll()
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().error(f'config reload rejected, keeping the old one: {error}')
            return
        if config is None:
            return
        # Diff against the previous file, not self.config, so launch overrides stay.
        changed = {key: value for key, value in config.items()
                   if not isinstance(value, dict) and value != self.file_config.get(key)}
        self.file_config = config
        if not changed:
            return
        new = validate_config(dict(self.config, **changed))
        if any(key in changed for key in OUTPUT_KEYS):
            self.outputs, error = switch_outputs(self.outputs, self.config, new, node=self)
            self.timed_out = True  # hold neutral until the next command
            if error:
                self.get_logger().error(f'outputs not moved, kept the old ones: {error}')
                new.update({key: self.config[key] for key in OUTPUT_KEYS})
            else:
                self.get_logger().info('outputs moved: ' + ', '.join(
                    f'{key}={changed[key]}' for key in OUTPUT_KEYS if key in changed))
        self.config = new
        self.apply(new)
        self.sync_parameters({key: new[key] for key in changed if self.has_parameter(key)})
        self.get_logger().info('config reloaded: ' + ', '.join(
            f'{key}={value}' for key, value in changed.items()))

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
        for result, parameter in zip(self.set_parameters(parameters), parameters):
            if not result.successful:
                self.get_logger().warning(
                    f'parameter {parameter.name} not synced: {result.reason}')

    def cmd_vel_callback(self, message):
        # Twist effort -> per-thruster effort -> pulse widths -> physical outputs.
        self.drive(thruster_pulses(twist_values(message), self.mixer, self.channel_map,
                                   self.reversed, self.decoupling, self.decoupling_negative))

    def pulses_callback(self, message):
        """Raw pulses per physical output, for bench tests; validated by the outputs."""
        pulses = list(message.data)
        try:
            if len(pulses) != 4 or any(not MIN_PWM <= pulse <= MAX_PWM for pulse in pulses):
                raise ValueError(f'need four pulses within {MIN_PWM}-{MAX_PWM} us, got {pulses}')
            self.drive(pulses)
        except ValueError as error:
            self.get_logger().error(f'test pulses rejected: {error}')

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
