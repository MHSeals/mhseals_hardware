"""Interactive TUI for bagged boat sensor and thruster characterization."""

import argparse
from collections import Counter
from datetime import datetime
import glob
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
import serial
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import String

from mhseals_hardware.thruster_mixer import (
    NEUTRAL_PWM,
    PWM_SCALE,
    THRUSTER_MIXER,
    channel_map_from_observations,
    validate_channel_map,
    validate_mixer,
)


POSITIONS = ('fl', 'fr', 'rr', 'rl')
POSITION_NAMES = {
    'fl': 'front-left', 'fr': 'front-right',
    'rr': 'rear-right', 'rl': 'rear-left',
}
THRUSTER_NUMBERS = {'fl': 1, 'fr': 2, 'rr': 3, 'rl': 4}
TEST_SEQUENCE = (1, -1, -1, 1, 1, -1)
AXIS_GUIDANCE = {
    'surge': {1: ('ahead', 'move forward'), -1: ('astern', 'reverse')},
    'sway': {
        1: ('on the port/left side', 'translate port/left'),
        -1: ('on the starboard/right side', 'translate starboard/right'),
    },
    'yaw': {
        1: ('around the boat', 'rotate counterclockwise in place'),
        -1: ('around the boat', 'rotate clockwise in place'),
    },
}
SENSOR_SPECS = (
    ('odometry', '/odom/mavros', Odometry, True),
    ('GPS', '/gps/fix', NavSatFix, True),
    ('IMU', '/imu/raw', Imu, True),
    ('LiDAR', '/points', PointCloud2, False),
    ('camera', '/front_camera/rgb/image', Image, False),
)


def parse_int_list(text):
    """Parse a comma-separated integer list."""
    return tuple(int(value.strip()) for value in text.split(','))


def parse_float_list(text):
    """Parse a comma-separated float list."""
    return tuple(float(value.strip()) for value in text.split(','))


class BoatTest:
    """Own the ROS status monitor, child processes, TUI, and safe shutdown."""

    def __init__(self, args):
        self.args = args
        self.console = Console()
        self.processes = {}
        self.node = Node('boat_test')
        self.event_publisher = self.node.create_publisher(
            String, '/boat_test/events', 10)
        self.command_publisher = None
        self.channel_map = None
        self.serial_connection = None
        self.matrix = validate_mixer(args.thruster_matrix)
        self.last_messages = {name: None for name, _, _, _ in SENSOR_SPECS}
        self.message_counts = Counter()
        self.completed = Counter()
        self.active_test = None
        self.bag_started = False
        self._spinning = True
        for name, topic, message_type, _ in SENSOR_SPECS:
            self.node.create_subscription(
                message_type, topic,
                lambda message, sensor=name: self.sensor_callback(sensor),
                qos_profile_sensor_data)
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

    def _spin(self):
        while self._spinning and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def sensor_callback(self, sensor):
        self.last_messages[sensor] = time.monotonic()
        self.message_counts[sensor] += 1

    def sensor_state(self, sensor, now=None):
        """Return status and age from actual received data, not topic names."""
        stamp = self.last_messages[sensor]
        if stamp is None:
            return 'MISSING', None
        age = (now or time.monotonic()) - stamp
        return ('READY' if age <= self.args.stale_after else 'STALE'), age

    def emit(self, phase, **details):
        payload = {
            'time': datetime.now().astimezone().isoformat(),
            'phase': phase,
            **details,
        }
        message = String(data=json.dumps(payload, sort_keys=True))
        self.event_publisher.publish(message)

    def start_process(self, name, command):
        self.console.print(f'[dim]Starting {name}: {" ".join(command)}[/]')
        process = subprocess.Popen(command, start_new_session=True)
        self.processes[name] = process
        return process

    def stop_process(self, name, timeout=10):
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
        """Start minimal MAVROS, optional sensors if requested, then rosbag."""
        self.start_process('odometry', [
            'ros2', 'launch', 'mhseals_nav', 'boat_test.launch.py',
            f'fcu_url:={self.args.fcu_url}',
        ])
        if self.args.optional_sensors:
            self.start_process('sensors', [
                'ros2', 'launch', 'mhseals_nav', 'sensors.launch.py',
                'sim:=false',
            ])
        self.start_process('bag', [
            'ros2', 'bag', 'record', '-a', '-o', str(self.args.bag_output),
        ])
        self.bag_started = True
        self.wait_for_measurements()

    def sensor_table(self):
        table = Table(title='Live sensor data', expand=True)
        table.add_column('Sensor')
        table.add_column('Topic')
        table.add_column('Required')
        table.add_column('Status')
        table.add_column('Messages', justify='right')
        now = time.monotonic()
        for name, topic, _, required in SENSOR_SPECS:
            state, age = self.sensor_state(name, now)
            color = {'READY': 'green', 'STALE': 'yellow', 'MISSING': 'red'}[state]
            detail = state if age is None else f'{state} ({age:.1f}s ago)'
            table.add_row(name, topic, 'yes' if required else 'optional',
                          f'[{color}]{detail}[/]', str(self.message_counts[name]))
        return table

    def thruster_table(self):
        table = Table(title='Thruster configuration: bow is at the top', expand=True)
        table.add_column('Number')
        table.add_column('Location')
        table.add_column('Physical output')
        table.add_column('Positive local thrust')
        mapping = self.channel_map or ('?', '?', '?', '?')
        for position, channel in zip(POSITIONS, mapping):
            local = 'aft (pushes boat astern)' if position[0] == 'f' \
                else 'forward (pushes boat ahead)'
            table.add_row(str(THRUSTER_NUMBERS[position]),
                          POSITION_NAMES[position], str(channel), local)
        return Group(Panel('1 (FL) ───── 2 (FR)\n   │           │\n4 (RL) ───── 3 (RR)',
                           title='Numbering', expand=False), table)

    def test_table(self):
        table = Table(title='Characterization runs', expand=True)
        table.add_column('Test')
        table.add_column('State')
        for axis in AXIS_GUIDANCE:
            if self.active_test == axis:
                state = '[yellow]RUNNING[/]'
            elif self.completed[axis]:
                state = f'[green]RAN ({self.completed[axis]} set(s))[/]'
            else:
                state = '[dim]NOT RUN[/]'
            table.add_row(axis, state)
        return table

    def dashboard(self):
        return Group(self.sensor_table(), self.thruster_table(), self.test_table())

    def wait_for_measurements(self):
        deadline = time.monotonic() + self.args.sensor_timeout
        with Live(self.dashboard(), console=self.console,
                  refresh_per_second=4) as live:
            while time.monotonic() < deadline:
                live.update(self.dashboard())
                if all(self.sensor_state(name)[0] == 'READY'
                       for name, _, _, required in SENSOR_SPECS if required):
                    return
                if any(process.poll() is not None for process in self.processes.values()):
                    break
                time.sleep(0.25)
        missing = [name for name, _, _, required in SENSOR_SPECS
                   if required and self.sensor_state(name)[0] != 'READY']
        answer = self.console.input(
            f'[red]Required live data missing: {", ".join(missing)}.[/] '
            'Continue without complete characterization data? [y/N] ')
        if answer.strip().lower() != 'y':
            raise RuntimeError('required sensor preflight was not accepted')

    def check_command_topic(self):
        publishers = [info for info in
                      self.node.get_publishers_info_by_topic('/cmd_vel')
                      if info.node_name != self.node.get_name()]
        if publishers:
            names = ', '.join(f'{info.node_namespace}/{info.node_name}'
                              for info in publishers)
            raise RuntimeError(f'/cmd_vel already has publishers: {names}')

    def send_serial(self, values, duration=0.0):
        message = (','.join(str(value) for value in values) + '\n').encode('ascii')
        deadline = time.monotonic() + duration
        while True:
            self.serial_connection.write(message)
            if time.monotonic() >= deadline:
                break
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

    def identify_thrusters(self):
        self.console.print(Panel(
            'Boat secured; all propellers clear and submerged.\n'
            'Identify outputs against 1=FL, 2=FR, 3=RR, 4=RL.',
            title='Thruster identification', style='yellow'))
        self.serial_connection = serial.Serial(
            self.args.serial_port, self.args.baud_rate, timeout=0.1)
        time.sleep(2.5)
        self.send_serial([NEUTRAL_PWM] * 4, 0.5)
        observations = {}
        used = set()
        try:
            for channel in range(1, 5):
                self.console.input(f'Press Enter to pulse physical output {channel} at 15%... ')
                values = [NEUTRAL_PWM] * 4
                values[channel - 1] += round(PWM_SCALE * 0.15)
                self.emit('identification_start', physical_channel=channel)
                self.send_serial(values, 0.75)
                self.send_serial([NEUTRAL_PWM] * 4, 0.5)
                self.emit('identification_stop', physical_channel=channel)
                while True:
                    position = self.console.input(
                        'Which moved? [fl/fr/rr/rl] ').strip().lower()
                    if position in POSITIONS and position not in used:
                        break
                    self.console.print('[red]Enter one unused position.[/]')
                expected = 'aft' if position.startswith('f') else 'forward'
                answer = self.console.input(
                    f'Did its positive local thrust point {expected}? [y/N] ')
                if answer.strip().lower() != 'y':
                    raise RuntimeError(
                        f'{POSITION_NAMES[position]} polarity is reversed')
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
        flat_matrix = [value for row in self.matrix for value in row]
        map_yaml = '[' + ','.join(str(value) for value in self.channel_map) + ']'
        matrix_yaml = '[' + ','.join(str(value) for value in flat_matrix) + ']'
        self.start_process('hardware', [
            'ros2', 'run', 'mhseals_hardware', 'thruster_serial_node',
            '--ros-args', '-p', f'serial_port:={self.args.serial_port}',
            '-p', f'baud_rate:={self.args.baud_rate}',
            '-p', f'channel_map:={map_yaml}',
            '-p', f'thruster_matrix:={matrix_yaml}',
        ])
        time.sleep(3)
        if self.processes['hardware'].poll() is not None:
            raise RuntimeError('thruster serial node exited during startup')
        self.command_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.emit('configuration', channel_map=self.channel_map,
                  thruster_numbers=THRUSTER_NUMBERS,
                  thruster_matrix=flat_matrix,
                  convention='+x forward, +y port, +yaw counterclockwise')

    def publish_command(self, axis=None, value=0.0, duration=0.1):
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
            time.sleep(0.1)

    def timed_phase(self, axis, value, duration):
        """Drive while retaining a live sensor/status display."""
        worker = threading.Thread(
            target=self.publish_command, args=(axis, value, duration))
        worker.start()
        with Live(self.dashboard(), console=self.console,
                  refresh_per_second=4) as live:
            while worker.is_alive():
                live.update(self.dashboard())
                time.sleep(0.25)
        worker.join()

    def run_axis(self, axis):
        counts = Counter()
        self.active_test = axis
        self.emit('test_set_start', axis=axis)
        try:
            for direction in TEST_SEQUENCE:
                counts[direction] += 1
                clearance, movement = AXIS_GUIDANCE[axis][direction]
                sign = '+' if direction > 0 else '-'
                self.console.print(
                    f'\n[bold]{axis.upper()} {sign} — {counts[direction]}/3[/]: '
                    f'clearance {clearance}; expected to {movement}.')
                self.console.input('Clear the area and press Enter to run... ')
                repeat = counts[direction]
                self.emit('baseline_start', axis=axis, direction=direction,
                          repeat=repeat)
                self.timed_phase(None, 0.0, self.args.baseline_duration)
                self.emit('command_start', axis=axis, direction=direction,
                          amplitude=self.args.amplitude,
                          duration=self.args.command_duration, repeat=repeat)
                self.timed_phase(axis, direction * self.args.amplitude,
                                 self.args.command_duration)
                self.emit('command_stop', axis=axis, direction=direction,
                          repeat=repeat)
                self.timed_phase(None, 0.0, self.args.settle_duration)
                self.emit('settle_stop', axis=axis, direction=direction,
                          repeat=repeat)
            self.completed[axis] += 1
            self.emit('test_set_complete', axis=axis,
                      run_number=self.completed[axis])
        finally:
            self.publish_command(duration=0.5)
            self.active_test = None

    def characterize(self):
        while True:
            self.console.print(self.dashboard())
            choice = self.console.input(
                '[bold]Select [all/surge/sway/yaw/status/quit]:[/] '
            ).strip().lower()
            if choice == 'quit':
                return
            if choice == 'status':
                continue
            axes = tuple(AXIS_GUIDANCE) if choice == 'all' else (choice,)
            if any(axis not in AXIS_GUIDANCE for axis in axes):
                self.console.print('[red]Unknown selection.[/]')
                continue
            for axis in axes:
                self.run_axis(axis)

    def run(self):
        self.start_measurement_stack()
        self.check_command_topic()
        self.channel_map = (self.identify_thrusters()
                            if self.args.channel_map is None
                            else validate_channel_map(self.args.channel_map))
        self.start_hardware()
        self.characterize()

    def shutdown(self):
        if self.command_publisher is not None:
            try:
                self.emit('shutdown', completed=dict(self.completed))
                self.publish_command(duration=1.0)
            except Exception:
                pass
        if self.serial_connection is not None:
            try:
                self.send_serial([NEUTRAL_PWM] * 4, 0.5)
                self.serial_connection.close()
            except Exception:
                pass
        self.stop_process('hardware')
        self.stop_process('bag', timeout=20)
        self.stop_process('sensors')
        self.stop_process('odometry')
        self._spinning = False
        self.spin_thread.join(timeout=1)
        self.node.destroy_node()


def discovered_serial_devices():
    devices = glob.glob('/dev/serial/by-id/*') + glob.glob('/dev/ttyACM*')
    return list(dict.fromkeys(devices))


def prompt_hardware(args):
    console = Console()
    devices = discovered_serial_devices()
    if devices:
        console.print('Discovered serial devices: ' + ', '.join(devices))
    if args.serial_port is None:
        args.serial_port = console.input('Pico serial device: ').strip()
    if args.fcu_url is None:
        args.fcu_url = console.input(
            'MAVROS FCU URL (e.g. serial:///dev/ttyACM1:57600): ').strip()
    if not args.serial_port or not args.fcu_url:
        raise ValueError('Pico serial device and MAVROS FCU URL are required')
    if args.fcu_url.startswith('serial://'):
        fcu_device = args.fcu_url[len('serial://'):].rsplit(':', 1)[0]
        if Path(fcu_device).resolve() == Path(args.serial_port).resolve():
            raise ValueError('Pico and FCU must use different serial devices')


def build_parser():
    parser = argparse.ArgumentParser(
        description='TUI for bagged omni-boat characterization')
    parser.add_argument('--serial-port', help='Pico serial device')
    parser.add_argument('--fcu-url', help='MAVROS FCU URL')
    parser.add_argument('--baud-rate', type=int, default=115200)
    parser.add_argument('--optional-sensors', action='store_true',
                        help='also launch camera and LiDAR drivers')
    parser.add_argument('--channel-map', type=parse_int_list,
                        help='canonical-to-physical FL,FR,RR,RL map')
    parser.add_argument('--thruster-matrix', type=parse_float_list,
                        default=tuple(v for row in THRUSTER_MIXER for v in row))
    parser.add_argument('--sensor-timeout', type=float, default=30.0)
    parser.add_argument('--stale-after', type=float, default=2.0)
    parser.add_argument('--amplitude', type=float, default=0.25)
    parser.add_argument('--baseline-duration', type=float, default=5.0)
    parser.add_argument('--command-duration', type=float, default=3.0)
    parser.add_argument('--settle-duration', type=float, default=5.0)
    parser.add_argument('--bag-output', type=Path,
                        default=Path('bags') / ('boat_test_' +
                        datetime.now().strftime('%Y%m%d_%H%M%S')))
    return parser


def main(args=None):
    parsed = build_parser().parse_args(args)
    prompt_hardware(parsed)
    parsed.thruster_matrix = tuple(
        value for row in validate_mixer(parsed.thruster_matrix) for value in row)
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
        Console().print('\n[yellow]Interrupted; neutralizing and flushing bag.[/]')
    finally:
        test.shutdown()
        rclpy.try_shutdown()
    Console().print(f'[green]Bag saved to {parsed.bag_output}[/]')


if __name__ == '__main__':
    main()
