"""Interactive, bagged thruster identification and characterization."""

import argparse
from collections import Counter
from datetime import datetime
import glob
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

from mhseals_hardware.thruster_serial_node import (
    NEUTRAL_PWM,
    PWM_SCALE,
    THRUSTER_MIXER,
    validate_channel_map,
    validate_mixer,
)


POSITIONS = ('fl', 'fr', 'rr', 'rl')
POSITION_NAMES = {
    'fl': 'front-left',
    'fr': 'front-right',
    'rr': 'rear-right',
    'rl': 'rear-left',
}
TEST_SEQUENCE = (1, -1, -1, 1, 1, -1)
AXIS_GUIDANCE = {
    'surge': {
        1: ('ahead', 'move forward'),
        -1: ('astern', 'reverse'),
    },
    'sway': {
        1: ('on the port/left side', 'translate port/left'),
        -1: ('on the starboard/right side', 'translate starboard/right'),
    },
    'yaw': {
        1: ('around the full boat perimeter',
            'rotate counterclockwise mostly in place'),
        -1: ('around the full boat perimeter',
             'rotate clockwise mostly in place'),
    },
}


def channel_map_from_observations(observations):
    """Invert {physical channel: canonical position} into canonical order."""
    if set(observations) != {1, 2, 3, 4}:
        raise ValueError('all four physical outputs must be identified')
    if set(observations.values()) != set(POSITIONS):
        raise ValueError(
            'each canonical position must be assigned exactly once')
    return tuple(next(channel for channel, position in observations.items()
                      if position == expected)
                 for expected in POSITIONS)


def parse_int_list(text):
    """Parse a comma-separated integer list."""
    return tuple(int(value.strip()) for value in text.split(','))


def parse_float_list(text):
    """Parse a comma-separated float list."""
    return tuple(float(value.strip()) for value in text.split(','))


class BoatTest:
    """Own the test processes, operator prompts, and safe shutdown."""

    def __init__(self, args):
        self.args = args
        self.processes = {}
        self.node = Node('thruster_boat_test')
        self.event_publisher = self.node.create_publisher(
            String, '/thruster_test/events', 10)
        self.command_publisher = None
        self.channel_map = None
        self.serial_connection = None
        self.matrix = validate_mixer(args.thruster_matrix)

    def emit(self, phase, **details):
        """Publish a self-describing marker into the active bag."""
        payload = {
            'time': datetime.now().astimezone().isoformat(),
            'phase': phase,
            **details,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.event_publisher.publish(message)
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def start_process(self, name, command):
        """Start a child in its own process group for reliable cleanup."""
        print(f'Starting {name}: {" ".join(command)}')
        process = subprocess.Popen(command, start_new_session=True)
        self.processes[name] = process
        return process

    def stop_process(self, name, timeout=10):
        """Interrupt one child and escalate only if it will not exit."""
        process = self.processes.get(name)
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def start_measurement_stack(self):
        """Start real sensors, odometry, and an all-topic recorder."""
        self.start_process('sensors', [
            'ros2', 'launch', 'mhseals_nav', 'sensors.launch.py',
            'sim:=false',
        ])
        self.start_process('odometry', [
            'ros2', 'launch', 'mhseals_nav', 'odom.launch.py',
            'sim:=false', f'fcu_url:={self.args.fcu_url}',
        ])
        self.start_process('bag', [
            'ros2', 'bag', 'record', '-a', '-o', str(self.args.bag_output),
        ])
        self.wait_for_measurements()

    def wait_for_measurements(self, timeout=30):
        """Wait for useful motion data and summarize optional sensors."""
        wanted = {
            'IMU': {'sensor_msgs/msg/Imu'},
            'odometry': {'nav_msgs/msg/Odometry'},
            'GPS': {'sensor_msgs/msg/NavSatFix'},
            'LiDAR': {'sensor_msgs/msg/PointCloud2'},
            'camera': {
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CompressedImage',
            },
        }
        deadline = time.monotonic() + timeout
        available = set()
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            topic_types = {
                topic_type
                for _, types in self.node.get_topic_names_and_types()
                for topic_type in types
            }
            available = {
                label for label, types in wanted.items()
                if topic_types.intersection(types)
            }
            if {'IMU', 'odometry'}.issubset(available):
                break
        print('Measurement topics: ' + ', '.join(
            f'{label}={"ready" if label in available else "missing"}'
            for label in wanted))
        if not {'IMU', 'odometry'}.issubset(available):
            answer = input(
                'IMU and odometry are required to quantify drift. '
                'Continue anyway? [y/N] ')
            if answer.strip().lower() != 'y':
                raise RuntimeError('measurement preflight was not accepted')
        elif available != set(wanted):
            answer = input(
                'Some optional sensors are missing. Continue? [Y/n] ')
            if answer.strip().lower() == 'n':
                raise RuntimeError(
                    'optional sensor preflight was not accepted')

    def check_command_topic(self):
        """Refuse to compete with navigation or teleoperation commands."""
        publishers = self.node.get_publishers_info_by_topic('/cmd_vel')
        if publishers:
            names = ', '.join(f'{info.node_namespace}/{info.node_name}'
                              for info in publishers)
            raise RuntimeError(
                f'/cmd_vel already has publishers ({names}); stop them first')

    def send_serial(self, values, duration=0.0):
        """Send an output repeatedly so the Pico watchdog stays satisfied."""
        message = (
            ','.join(str(value) for value in values) + '\n').encode('ascii')
        deadline = time.monotonic() + duration
        while True:
            self.serial_connection.write(message)
            if time.monotonic() >= deadline:
                break
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

    def identify_thrusters(self):
        """Pulse physical outputs and collect the canonical software map."""
        print('\nIDENTIFICATION: boat must be secured, propellers clear, '
              'and all thrusters fully submerged.')
        self.serial_connection = serial.Serial(
            self.args.serial_port, self.args.baud_rate, timeout=0.1)
        time.sleep(2.5)
        self.send_serial([NEUTRAL_PWM] * 4, 0.5)
        observations = {}
        used = set()
        try:
            for channel in range(1, 5):
                input(
                    'Press Enter to pulse physical output '
                    f'{channel} at 15%... ')
                values = [NEUTRAL_PWM] * 4
                values[channel - 1] += round(PWM_SCALE * 0.15)
                self.emit('identification_start', physical_channel=channel,
                          pwm=values[channel - 1])
                self.send_serial(values, 0.75)
                self.send_serial([NEUTRAL_PWM] * 4, 0.5)
                self.emit('identification_stop', physical_channel=channel)
                while True:
                    position = input(
                        'Which thruster moved? [fl/fr/rr/rl] ').strip().lower()
                    if position in POSITIONS and position not in used:
                        break
                    print('Enter one unused position: fl, fr, rr, or rl.')
                expected = ('aft' if position.startswith('f') else 'ahead')
                answer = input(
                    f'Did positive thrust have the expected {expected} '
                    'longitudinal component? [y/N] ')
                if answer.strip().lower() != 'y':
                    raise RuntimeError(
                        f'{POSITION_NAMES[position]} polarity is reversed; '
                        'correct the ESC/motor direction before testing')
                observations[channel] = position
                used.add(position)
        finally:
            if self.serial_connection is not None:
                try:
                    self.send_serial([NEUTRAL_PWM] * 4, 0.5)
                finally:
                    self.serial_connection.close()
                    self.serial_connection = None
        return channel_map_from_observations(observations)

    def start_hardware(self):
        """Transfer serial ownership to the normal watchdog-protected node."""
        flat_matrix = [value for row in self.matrix for value in row]
        map_yaml = (
            '[' + ','.join(str(value) for value in self.channel_map) + ']')
        matrix_yaml = '[' + ','.join(str(value) for value in flat_matrix) + ']'
        self.start_process('hardware', [
            'ros2', 'run', 'mhseals_hardware', 'thruster_serial_node',
            '--ros-args',
            '-p', f'serial_port:={self.args.serial_port}',
            '-p', f'baud_rate:={self.args.baud_rate}',
            '-p', f'channel_map:={map_yaml}',
            '-p', f'thruster_matrix:={matrix_yaml}',
        ])
        time.sleep(3)
        if self.processes['hardware'].poll() is not None:
            raise RuntimeError('thruster serial node exited during startup')
        self.command_publisher = self.node.create_publisher(
            Twist, '/cmd_vel', 10)
        self.emit(
            'configuration',
            channel_map=self.channel_map,
            thruster_matrix=flat_matrix,
            convention='+x forward, +y port, +yaw counterclockwise',
        )
        print(f'Canonical channel map FL,FR,RR,RL = {list(self.channel_map)}')
        print('Reuse with: --channel-map ' +
              ','.join(str(value) for value in self.channel_map))

    def publish_command(self, axis=None, value=0.0, duration=0.1):
        """Publish one axis at 10 Hz for a bounded duration."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = Twist()
            if axis == 'surge':
                message.linear.x = value
            elif axis == 'sway':
                message.linear.y = value
            elif axis == 'yaw':
                message.angular.z = value
            self.command_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.09)

    def run_axis(self, axis):
        """Run the balanced six-trial low-power profile for one axis."""
        counts = Counter()
        for direction in TEST_SEQUENCE:
            counts[direction] += 1
            clearance, movement = AXIS_GUIDANCE[axis][direction]
            sign = '+' if direction > 0 else '-'
            print(f'\n{axis.upper()} {sign}, repeat {counts[direction]}/3: '
                  f'needs free space {clearance}; it will {movement}.')
            if axis == 'yaw':
                print('Keep lines loose and clear of every propeller.')
            input('Reposition if needed, confirm the area is clear, '
                  'then press Enter... ')
            self.emit('baseline_start', axis=axis, direction=direction,
                      repeat=counts[direction])
            self.publish_command(duration=5.0)
            self.emit('command_start', axis=axis, direction=direction,
                      amplitude=0.25, duration=3.0,
                      repeat=counts[direction])
            self.publish_command(axis, direction * 0.25, 3.0)
            self.emit('command_stop', axis=axis, direction=direction,
                      repeat=counts[direction])
            self.publish_command(duration=5.0)
            self.emit('settle_stop', axis=axis, direction=direction,
                      repeat=counts[direction])

    def characterize(self):
        """Offer a compact menu for all or selected planar axes."""
        while True:
            choice = input(
                '\nCharacterize [all/surge/sway/yaw/quit]: ').strip().lower()
            if choice == 'quit':
                return
            axes = ('surge', 'sway', 'yaw') if choice == 'all' else (choice,)
            if any(axis not in AXIS_GUIDANCE for axis in axes):
                print('Choose all, surge, sway, yaw, or quit.')
                continue
            for axis in axes:
                self.run_axis(axis)

    def run(self):
        """Run preflight, identification, and characterization."""
        self.start_measurement_stack()
        self.check_command_topic()
        if self.args.channel_map is None:
            self.channel_map = self.identify_thrusters()
        else:
            self.channel_map = validate_channel_map(self.args.channel_map)
        self.start_hardware()
        self.characterize()

    def shutdown(self):
        """Neutralize first, then flush the bag before stopping sensors."""
        if self.command_publisher is not None:
            try:
                self.emit('shutdown', reason='operator or process exit')
                self.publish_command(duration=1.0)
            except Exception:
                pass
        if self.serial_connection is not None:
            try:
                self.send_serial([NEUTRAL_PWM] * 4, 0.5)
                self.serial_connection.close()
            except Exception:
                pass
            self.serial_connection = None
        self.stop_process('hardware')
        self.stop_process('bag', timeout=20)
        self.stop_process('odometry')
        self.stop_process('sensors')
        self.node.destroy_node()


def discovered_serial_devices():
    """Return stable device links first, followed by ACM device names."""
    devices = glob.glob('/dev/serial/by-id/*') + glob.glob('/dev/ttyACM*')
    return list(dict.fromkeys(devices))


def prompt_hardware(args):
    """Collect hardware endpoints that cannot safely be guessed."""
    devices = discovered_serial_devices()
    if devices:
        print('Discovered serial devices:')
        for device in devices:
            print(f'  {device}')
    if args.serial_port is None:
        args.serial_port = input('Pico serial device: ').strip()
    if args.fcu_url is None:
        args.fcu_url = input(
            'MAVROS FCU URL (for example serial:///dev/ttyACM1:57600): '
        ).strip()
    if not args.serial_port or not args.fcu_url:
        raise ValueError(
            'both Pico serial device and MAVROS FCU URL are required')
    if args.fcu_url.startswith('serial://'):
        fcu_device = args.fcu_url[len('serial://'):].rsplit(':', 1)[0]
        if Path(fcu_device).resolve() == Path(args.serial_port).resolve():
            raise ValueError('Pico and FCU must use different serial devices')


def build_parser():
    """Build the deliberately small boat-test command interface."""
    parser = argparse.ArgumentParser(
        description='Identify and characterize the omni thrusters with rosbag')
    parser.add_argument('--serial-port', help='Pico serial device')
    parser.add_argument('--fcu-url', help='MAVROS FCU URL')
    parser.add_argument('--baud-rate', type=int, default=115200)
    parser.add_argument(
        '--channel-map', type=parse_int_list,
        help=('known canonical-to-physical FL,FR,RR,RL map; '
              'skips identification'))
    parser.add_argument(
        '--thruster-matrix', type=parse_float_list,
        default=tuple(value for row in THRUSTER_MIXER for value in row),
        help='12 comma-separated row-major FL,FR,RR,RL coefficients')
    parser.add_argument(
        '--bag-output', type=Path,
        default=Path('bags') / (
            'thruster_characterization_' +
            datetime.now().strftime('%Y%m%d_%H%M%S')),
        help='new rosbag output directory')
    return parser


def main(args=None):
    """Run the interactive boat test with guaranteed cleanup."""
    parsed = build_parser().parse_args(args)
    prompt_hardware(parsed)
    parsed.thruster_matrix = tuple(
        value
        for row in validate_mixer(parsed.thruster_matrix)
        for value in row
    )
    if parsed.channel_map is not None:
        parsed.channel_map = validate_channel_map(parsed.channel_map)
    if parsed.bag_output.exists():
        raise ValueError(f'bag output already exists: {parsed.bag_output}')
    parsed.bag_output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    test = BoatTest(parsed)
    try:
        test.run()
    except KeyboardInterrupt:
        print('\nTest interrupted; neutralizing and flushing the bag.')
    finally:
        test.shutdown()
        rclpy.try_shutdown()
    print(f'Bag saved to {parsed.bag_output}')


if __name__ == '__main__':
    main()
