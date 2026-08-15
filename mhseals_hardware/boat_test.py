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
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.manual_control import run_manual


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
TEST_CHOICES = {'1': 'surge', '2': 'sway', '3': 'yaw'}
MENU_CHOICES = ('surge', 'sway', 'yaw', 'all tests', 'manual control',
                'refresh status', 'finish and save bag')
PREFLIGHT_CHOICES = ('keep waiting', 'continue without sensor data',
                     'cancel test')
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
        self.process_logs = {}
        self.log_directory = (
            args.bag_output.parent / 'logs' / args.bag_output.name)
        self.node = Node('boat_test')
        self.event_publisher = self.node.create_publisher(
            String, '/boat_test/events', 10)
        self.command_publisher = None
        self.channel_map = None
        self.thruster_rotations = {}
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
        """Start a child, logging its output so it cannot corrupt the TUI."""
        # rosbag has interactive keyboard controls. Giving every child
        # /dev/null prevents it from consuming the TUI's input stream.
        popen_options = {
            'start_new_session': True,
            'stdin': subprocess.DEVNULL,
        }
        if not self.args.show_process_output:
            self.log_directory.mkdir(parents=True, exist_ok=True)
            log_path = self.log_directory / f'{name}.log'
            log_file = log_path.open('w', encoding='utf-8')
            self.process_logs[name] = log_file
            popen_options.update(stdout=log_file, stderr=subprocess.STDOUT)
        process = subprocess.Popen(command, **popen_options)
        self.processes[name] = process
        return process

    def stop_process(self, name, timeout=10):
        process = self.processes.get(name)
        if process is None:
            return
        if process.poll() is None:
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
        log_file = self.process_logs.pop(name, None)
        if log_file is not None:
            log_file.close()

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
        table = Table(
            title='Live sensor data  [green]●[/] ready  '
                  '[yellow]●[/] stale  [red]●[/] missing',
            expand=True)
        table.add_column('Sensor')
        table.add_column('Topic')
        table.add_column('Required')
        table.add_column('●', justify='center', width=3)
        table.add_column('Age', justify='right')
        table.add_column('Messages', justify='right')
        now = time.monotonic()
        for name, topic, _, required in SENSOR_SPECS:
            state, age = self.sensor_state(name, now)
            color = {'READY': 'green', 'STALE': 'yellow', 'MISSING': 'red'}[state]
            age_text = '-' if age is None else f'{age:.1f}s'
            table.add_row(name, topic, 'yes' if required else 'optional',
                          f'[{color}]●[/]', age_text,
                          str(self.message_counts[name]))
        return table

    def process_table(self):
        """Represent background ROS processes without displaying their output."""
        table = Table(title='Background processes', expand=True)
        table.add_column('Process')
        table.add_column('Status')
        table.add_column('Diagnostic log')
        for name, process in self.processes.items():
            return_code = process.poll()
            if return_code is None:
                status = '[green]RUNNING[/]'
            elif return_code == 0:
                status = '[dim]STOPPED[/]'
            else:
                status = f'[red]EXITED ({return_code})[/]'
            log_path = self.log_directory / f'{name}.log'
            log = '-' if self.args.show_process_output else str(log_path)
            table.add_row(name, status, log)
        return table

    def thruster_table(self):
        table = Table(title='Thruster configuration: bow is at the top', expand=True)
        table.add_column('Number')
        table.add_column('Location')
        table.add_column('Physical output')
        table.add_column('Prop rotation')
        table.add_column('Positive local thrust')
        mapping = self.channel_map or ('?', '?', '?', '?')
        for position, channel in zip(POSITIONS, mapping):
            local = 'aft (pushes boat astern)' if position[0] == 'f' \
                else 'forward (pushes boat ahead)'
            rotation = self.thruster_rotations.get(position, '?')
            table.add_row(str(THRUSTER_NUMBERS[position]),
                          POSITION_NAMES[position], str(channel), rotation,
                          local)
        return Group(Panel('1 (FL) ───── 2 (FR)\n   │           │\n4 (RL) ───── 3 (RR)',
                           title='Numbering', expand=False), table)

    def test_table(self, selected=0):
        table = Table(
            title='Select a characterization test by number', expand=True)
        table.add_column('Key', justify='center')
        table.add_column('Test')
        table.add_column('State')
        for index, (key, axis) in enumerate(TEST_CHOICES.items()):
            if self.active_test == axis:
                state = '[yellow]● RUNNING[/]'
            elif self.completed[axis]:
                state = f'[green]● RAN ({self.completed[axis]} set(s))[/]'
            else:
                state = '[dim]○ NOT RUN[/]'
            marker = '[bold cyan]›[/]' if selected == index else ' '
            table.add_row(f'{marker} {key}', axis, state)
        extra = (('4', 'all tests'), ('m', 'manual control'),
                 ('s', 'refresh status'), ('q', 'finish and save bag'))
        for offset, (key, label) in enumerate(extra, start=3):
            marker = '[bold cyan]›[/]' if selected == offset else ' '
            table.add_row(f'{marker} {key}', label, '')
        table.caption = ('↑/↓ select  •  Enter run  •  '
                         '1-4 direct test  •  M manual  •  Q finish')
        return table

    def preflight_panel(self, selected=0, timed_out=False):
        """Render controls that are active while measurements start."""
        rows = []
        for index, choice in enumerate(PREFLIGHT_CHOICES):
            marker = '[bold cyan]›[/]' if selected == index else ' '
            rows.append(f'{marker} {choice}')
        state = ('[yellow]Sensor wait timed out.[/]'
                 if timed_out else '[dim]Waiting for required live data…[/]')
        rows.extend(('', state, '[dim]↑/↓ select  •  Enter confirm  •  '
                    'W wait  •  C continue  •  Q cancel[/]'))
        return Panel('\n'.join(rows), title='Sensor preflight')

    def dashboard(self, selected=0, preflight=False, timed_out=False):
        selector = (self.preflight_panel(selected, timed_out)
                    if preflight else self.test_table(selected))
        return Group(self.process_table(), self.sensor_table(),
                     self.thruster_table(), selector)

    def wait_for_measurements(self):
        deadline = time.monotonic() + self.args.sensor_timeout
        selected = 0
        shortcuts = {'w': 0, 'c': 1, 'q': 2}
        with KeyReader() as keys, Live(
                self.dashboard(preflight=True), console=self.console,
                refresh_per_second=8) as live:
            while True:
                if all(self.sensor_state(name)[0] == 'READY'
                       for name, _, _, required in SENSOR_SPECS if required):
                    return
                if any(process.poll() is not None for process in self.processes.values()):
                    deadline = min(deadline, time.monotonic())
                if self.args.allow_missing_sensors:
                    return
                key = keys.read(timeout=0.1)
                if key == 'up':
                    selected = (selected - 1) % len(PREFLIGHT_CHOICES)
                elif key == 'down':
                    selected = (selected + 1) % len(PREFLIGHT_CHOICES)
                elif key in shortcuts:
                    selected = shortcuts[key]
                    key = 'enter'
                if key == 'enter':
                    choice = PREFLIGHT_CHOICES[selected]
                    if choice == 'continue without sensor data':
                        return
                    if choice == 'cancel test':
                        raise RuntimeError('sensor preflight cancelled')
                    deadline = time.monotonic() + self.args.sensor_timeout
                timed_out = time.monotonic() >= deadline
                live.update(self.dashboard(
                    selected, preflight=True, timed_out=timed_out))

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

    def select_option(self, title, options, shortcuts=None):
        """Select an option with arrows/Enter or an explicit shortcut."""
        selected = 0
        shortcuts = shortcuts or {}

        def render():
            rows = []
            for index, option in enumerate(options):
                marker = '[bold cyan]›[/]' if index == selected else ' '
                rows.append(f'{marker} {option}')
            rows.append('\n[dim]↑/↓ select  •  Enter confirm[/]')
            return Panel('\n'.join(rows), title=title, expand=False)

        with KeyReader() as keys, Live(
                render(), console=self.console,
                refresh_per_second=10) as live:
            while True:
                key = keys.read(timeout=0.1)
                if key == 'up':
                    selected = (selected - 1) % len(options)
                elif key == 'down':
                    selected = (selected + 1) % len(options)
                elif key == 'enter':
                    return options[selected]
                elif key in shortcuts:
                    return options[shortcuts[key]]
                live.update(render())

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
                choices = tuple(position for position in POSITIONS
                                if position not in used)
                position = self.select_option(
                    'Which thruster moved?', choices,
                    {str(index + 1): index
                     for index in range(len(choices))})
                rotation = self.select_option(
                    'Prop rotation (viewed from propeller toward motor)',
                    ('CCW', 'CW'), {'c': 0, 'w': 1})
                expected = 'aft' if position.startswith('f') else 'forward'
                answer = self.select_option(
                    f'Did positive local thrust point {expected}?',
                    ('yes', 'no'), {'y': 0, 'n': 1})
                if answer != 'yes':
                    raise RuntimeError(
                        f'{POSITION_NAMES[position]} polarity is reversed')
                observations[channel] = position
                used.add(position)
                self.thruster_rotations[position] = rotation
                self.emit('identification_result', physical_channel=channel,
                          position=position, rotation=rotation,
                          rotation_view='from propeller toward motor')
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
                  thruster_rotations=self.thruster_rotations,
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

    def select_test(self):
        """Select a test from the live dashboard with arrows or shortcuts."""
        selected = 0
        shortcuts = {
            '1': 0, '2': 1, '3': 2, '4': 3,
            'm': 4, 's': 5, 'q': 6,
        }
        with KeyReader() as keys, Live(
                self.dashboard(selected), console=self.console,
                refresh_per_second=8) as live:
            while True:
                key = keys.read(timeout=0.1)
                if key == 'up':
                    selected = (selected - 1) % len(MENU_CHOICES)
                elif key == 'down':
                    selected = (selected + 1) % len(MENU_CHOICES)
                elif key == 'enter':
                    return MENU_CHOICES[selected]
                elif key in shortcuts:
                    return MENU_CHOICES[shortcuts[key]]
                live.update(self.dashboard(selected))

    def characterize(self):
        while True:
            choice = self.select_test()
            if choice == 'finish and save bag':
                return
            if choice == 'refresh status':
                continue
            if choice == 'manual control':
                self.emit('manual_control_start')
                run_manual(self.command_publisher, self.console,
                           self.args.manual_amplitude)
                self.emit('manual_control_stop')
                continue
            axes = (tuple(AXIS_GUIDANCE) if choice == 'all tests'
                    else (choice,))
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
    parser.add_argument(
        '--allow-missing-sensors', action='store_true',
        help='continue to thruster tests when required sensor data is missing')
    parser.add_argument(
        '--show-process-output', action='store_true',
        help='show raw child-process output instead of logging it')
    parser.add_argument('--channel-map', type=parse_int_list,
                        help='canonical-to-physical FL,FR,RR,RL map')
    parser.add_argument('--thruster-matrix', type=parse_float_list,
                        default=tuple(v for row in THRUSTER_MIXER for v in row))
    parser.add_argument('--sensor-timeout', type=float, default=30.0)
    parser.add_argument('--stale-after', type=float, default=2.0)
    parser.add_argument('--amplitude', type=float, default=0.25)
    parser.add_argument('--manual-amplitude', type=float, default=0.25)
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
