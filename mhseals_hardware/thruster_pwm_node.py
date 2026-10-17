"""Mix ROS effort commands into thruster pulses: ODROID PWM pins or a simulator topic.

Two inputs drive the outputs, newest message wins:
  cmd_vel        Twist effort, through the decoupling, mixer, reversals and channel map
  ~/test_pulses  UInt16MultiArray raw pulses per physical output (bench tests)
Either goes neutral after ``command_timeout`` without a message. With
``config_file`` set, edits to the YAML's mapping, mixer, decoupling, reversals and timeout
apply live; pin and backend changes need a restart.
"""

import os
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

from mhseals_hardware.configuration import load_config, validate_config
from mhseals_hardware.outputs import make_outputs
from mhseals_hardware.ros_util import run_node, twist_values
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, thruster_pulses, validate_channel_map, validate_decoupling, validate_mixer,
    validate_reversed,
)

# Settings that can change while the outputs run; anything else needs a restart.
LIVE_KEYS = ('channel_map', 'thruster_matrix', 'decoupling', 'reversed', 'command_timeout')
RELOAD_PERIOD = 1.0


class ThrusterPWMNode(Node):
    """Fail-neutral bridge from cmd_vel effort to the configured thruster outputs."""

    def __init__(self):
        super().__init__('thruster_pwm_node')
        # Every hardware setting is a ROS parameter, defaulting to the YAML config.
        self.declare_parameter('config_file', '')
        self.config_file = self.get_parameter('config_file').value or None
        config = load_config(self.config_file)
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
        self.config_mtime = self.mtime()
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(UInt16MultiArray, '~/test_pulses', self.pulses_callback, 10)
        self.create_timer(0.1, self.watchdog_callback)
        if self.config_file:
            self.create_timer(RELOAD_PERIOD, self.reload_callback)
        self.get_logger().info(
            f"thruster control ready ({self.config['output_backend']} outputs)")

    def apply(self, config):
        self.command_timeout = float(config['command_timeout'])
        self.channel_map = validate_channel_map(config['channel_map'])
        self.mixer = validate_mixer(config['thruster_matrix'])
        self.decoupling = validate_decoupling(config['decoupling'])
        self.reversed = validate_reversed(config['reversed'])

    def mtime(self):
        try:
            return os.stat(self.config_file).st_mtime_ns if self.config_file else None
        except OSError:
            return None

    def reload_callback(self):
        """Hot-reload the live settings when the YAML changes; keep running on a bad edit."""
        mtime = self.mtime()
        if mtime is None or mtime == self.config_mtime:
            return
        self.config_mtime = mtime
        try:
            config = load_config(self.config_file)
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().error(f'config reload rejected, keeping the old one: {error}')
            return
        restart = [key for key in config if key not in LIVE_KEYS
                   and not isinstance(config[key], dict) and config[key] != self.config[key]]
        live = {key: config[key] for key in LIVE_KEYS}
        self.config.update(live)
        self.apply(self.config)
        self.get_logger().info('config reloaded: ' + ', '.join(
            f'{key}={value}' for key, value in live.items()))
        if restart:
            self.get_logger().warning(f"{', '.join(restart)} changed; restart to apply")

    def cmd_vel_callback(self, message):
        # Twist effort -> per-thruster effort -> pulse widths -> physical outputs.
        self.drive(thruster_pulses(twist_values(message), self.mixer, self.channel_map,
                                   self.reversed, self.decoupling))

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
