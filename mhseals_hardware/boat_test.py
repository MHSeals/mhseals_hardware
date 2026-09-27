"""Interactive TUI for bagged boat sensor and thruster characterization."""

import argparse
from collections import Counter
from datetime import datetime
from functools import partial
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import socket
import sys
from types import SimpleNamespace
from mhseals_hardware.configuration import configure_args, save_config

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from sensor_msgs.msg import Image, CompressedImage, Imu, NavSatFix, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from mhseals_hardware.boat_monitor import BoatMonitor, finite_values
from mhseals_hardware.boat_dashboard import render_dashboard, VIEWS

from mhseals_hardware.thruster_mixer import (
    NEUTRAL_PWM,
    PWM_SCALE,
    THRUSTER_MIXER,
    channel_map_from_observations,
    validate_channel_map,
    validate_mixer,
)
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.fcu import default_fcu_url
from mhseals_hardware.keyboard_control import run_manual
from mhseals_hardware.odroid_pwm import DEFAULT_PWM_CHIPS, OdroidPWMOutputs, resolve_pwm_chip


POSITIONS = ('fl', 'fr', 'rr', 'rl')
POSITION_NAMES = {
    'fl': 'front-left', 'fr': 'front-right',
    'rr': 'rear-right', 'rl': 'rear-left',
}
THRUSTER_NUMBERS = {'fl': 1, 'fr': 2, 'rr': 3, 'rl': 4}
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
    ('odometry', '/odom/local', Odometry, True),
    ('FCU odometry', '/odom/mavros', Odometry, False),
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
        self.monitor = BoatMonitor(args.stale_after)
        self.view = 'overview'
        self.notice = 'Passive monitor; H explicitly arms hardware'
        self.command = (0.0, 0.0, 0.0)
        self.armed = False
        self.pwm_readback = ['not sampled'] * 4
        self.last_pwm_read = 0.
        self.specs = tuple((name, args.odom_topic if name == 'odometry' else
                            args.camera_topic if name == 'camera' else topic,
                            CompressedImage if name == 'camera' and args.camera_type == 'compressed'
                            else message_type, required)
                           for name, topic, message_type, required in SENSOR_SPECS)
        self.processes = {}
        self.process_logs = {}
        self.log_directory = (
            args.bag_output.parent / 'logs' / args.bag_output.name)
        self.node = Node('boat_test')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.event_publisher = self.node.create_publisher(
            String, '/boat_test/events', 10)
        self.command_publisher = None
        self.channel_map = validate_channel_map(args.channel_map)
        self.thruster_rotations = {}
        self.pwm_outputs = None
        self.matrix = validate_mixer(args.thruster_matrix)
        self.completed = Counter()
        self.active_test = None
        self.bag_started = False
        self._spinning = True
        self.monitor_fault = ''
        for name, topic, message_type, _ in self.specs:
            self.node.create_subscription(
                message_type, topic,
                partial(self.sensor_callback, name),
                qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, '/tf', self.tf_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            TFMessage, '/tf_static', lambda msg, info: self.tf_callback(msg, info, True),
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

    def _spin(self):
        try:
            while self._spinning and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            self.monitor_fault = str(error)
            self.monitor.event('Monitor fault: ' + self.monitor_fault)

    def sensor_callback(self, sensor, message):
        stamp = message.header.stamp
        stamp_age = self.node.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
        values, issue = {'sensor_frame': message.header.frame_id}, ''
        if isinstance(message, Odometry):
            velocity = message.twist.twist
            values = dict(vx=velocity.linear.x, vy=velocity.linear.y, wz=velocity.angular.z,
                          frame=message.header.frame_id, child=message.child_frame_id,
                          x=message.pose.pose.position.x, y=message.pose.pose.position.y)
            if not finite_values([values[k] for k in ('vx', 'vy', 'wz', 'x', 'y')]):
                issue = 'non-finite pose/velocity'
            elif message.child_frame_id != self.args.base_frame:
                issue = 'velocity frame is not ' + self.args.base_frame
            elif not message.header.frame_id:
                issue = 'empty odometry reference frame'
            q = message.pose.pose.orientation
            norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
            if not math.isfinite(norm) or abs(norm - 1) > .1:
                issue = issue or 'invalid odometry orientation'
            values['summary'] = f"vx {velocity.linear.x:+.2f} vy {velocity.linear.y:+.2f} wz {velocity.angular.z:+.2f}"
        elif isinstance(message, NavSatFix):
            values['summary'] = f'fix {message.status.status} | {message.latitude:.6f}, {message.longitude:.6f}'
            if message.status.status < 0:
                issue = 'GPS has no fix'
            elif (not finite_values((message.latitude, message.longitude)) or
                  not -90 <= message.latitude <= 90 or not -180 <= message.longitude <= 180):
                issue = 'invalid GPS coordinates'
            elif message.position_covariance_type == 0:
                issue = 'GPS covariance unknown'
            elif not finite_values(message.position_covariance) or any(
                    message.position_covariance[index] < 0 for index in (0, 4, 8)):
                issue = 'invalid GPS covariance'
            else:
                sigma = math.sqrt(max(message.position_covariance[0], message.position_covariance[4]))
                values['summary'] += f' | max XY-axis sigma {sigma:.2f}m'
        elif isinstance(message, Imu):
            q = message.orientation
            norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
            values['summary'] = f'wz {message.angular_velocity.z:+.3f} rad/s | {message.header.frame_id}'
            if not math.isfinite(norm) or abs(norm - 1) > .1:
                issue = 'invalid orientation quaternion'
            elif message.orientation_covariance[0] < 0:
                issue = 'IMU orientation unavailable'
            elif not finite_values((message.angular_velocity.x, message.angular_velocity.y,
                                    message.angular_velocity.z)):
                issue = 'invalid IMU angular velocity'
        elif isinstance(message, PointCloud2):
            values['summary'] = f'{message.width * message.height} points | {message.header.frame_id}'
            if not message.width or not message.data:
                issue = 'empty point cloud'
        else:
            values['summary'] = message.header.frame_id
            if not message.data:
                issue = 'empty image'
        if stamp.sec == 0 and stamp.nanosec == 0:
            issue = issue or 'zero message timestamp'
        if stamp_age < -.25 or stamp_age > self.args.stale_after:
            issue = issue or 'message timestamp outside freshness window'
        self.monitor.observe(sensor, values, issue, stamp_age)

    def tf_callback(self, message, info, static=False):
        now = self.node.get_clock().now().nanoseconds / 1e9
        gid = info.get('publisher_gid', b'') if isinstance(info, dict) else info.publisher_gid
        authority = bytes(gid).hex()
        for transform in message.transforms:
            q = transform.transform.rotation
            translation = transform.transform.translation
            components = (q.x, q.y, q.z, q.w, translation.x, translation.y, translation.z)
            if not finite_values(components) or abs(sum(v*v for v in components[:4]) - 1) > .1:
                self.monitor.event('Invalid TF quaternion/translation: ' + transform.child_frame_id)
                continue
            stamp = transform.header.stamp
            self.monitor.transform(transform.header.frame_id, transform.child_frame_id,
                                   now - stamp.sec - stamp.nanosec / 1e9, static, authority)

    def emit(self, phase, **details):
        payload = {
            'time': datetime.now().astimezone().isoformat(),
            'phase': phase,
            **details,
        }
        message = String(data=json.dumps(payload, sort_keys=True))
        self.event_publisher.publish(message)
        self.monitor.event(phase + (' ' + json.dumps(details, sort_keys=True) if details else ''))

    def start_process(self, name, command):
        """Start a child, logging its output so it cannot corrupt the TUI."""
        if name in self.processes:
            if self.processes[name].poll() is None:
                raise RuntimeError(f'{name} is already running')
            self.stop_process(name)
        # rosbag has interactive keyboard controls. Giving every child
        # /dev/null prevents it from consuming the TUI's input stream.
        popen_options = {
            'start_new_session': True,
            'stdin': subprocess.DEVNULL,
        }
        if not self.args.show_process_output:
            self.log_directory.mkdir(parents=True, exist_ok=True)
            log_path = self.log_directory / f'{name}.log'
            log_file = log_path.open('a', encoding='utf-8')
            self.process_logs[name] = log_file
            popen_options.update(stdout=log_file, stderr=subprocess.STDOUT)
        process = subprocess.Popen(command, **popen_options)
        self.processes[name] = process
        return process

    def stop_process(self, name, timeout=10):
        process = self.processes.get(name)
        if process is None:
            return
        def send(signum):
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                pass
        if process.poll() is None:
            send(signal.SIGINT)
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                send(signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    send(signal.SIGKILL)
                    process.wait()
        log_file = self.process_logs.pop(name, None)
        if log_file is not None:
            log_file.close()

    def start_measurement_stack(self):
        """Start minimal MAVROS, optional sensors if requested, then rosbag."""
        self.start_process('odometry', [
            'ros2', 'launch', 'mhseals_nav', 'odom.launch.py', 'sim:=false',
            f'fcu_url:={self.args.fcu_url}',
        ])
        if self.args.optional_sensors:
            self.start_process('sensors', [
                'ros2', 'launch', 'mhseals_nav', 'sensors.launch.py',
                'sim:=false',
            ])

    def dashboard(self):
        if time.monotonic() - self.last_pwm_read > 1:
            self.last_pwm_read = time.monotonic()
            for index, (chip, channel) in enumerate(zip(self.args.pwm_chips, self.args.pwm_channels)):
                try:
                    path = resolve_pwm_chip(chip) / f'pwm{channel}'
                    enabled = (path / 'enable').read_text().strip()
                    period = int((path / 'period').read_text())
                    duty = int((path / 'duty_cycle').read_text())
                    self.pwm_readback[index] = f'{"ON" if enabled == "1" else "OFF"} {duty/1000:g}us / {1e9/period:g}Hz' if period else 'period unset'
                except (OSError, ValueError):
                    self.pwm_readback[index] = 'unavailable / unexported'
        processes = '\n'.join(f'{name}: ' + ('RUNNING' if process.poll() is None else
                               f'EXITED {process.returncode}') +
                               f' | {self.log_directory / (name + ".log")}'
                               for name, process in self.processes.items())
        bag = self.processes.get('bag')
        return render_dashboard(self.monitor.snapshot(self.required_tf()), dict(
            view=self.view, armed=self.armed,
            notice=('MONITOR FAULT: ' + self.monitor_fault) if self.monitor_fault else self.notice,
            host=socket.gethostname(),
            domain=os.environ.get('ROS_DOMAIN_ID', '0'),
            specs=[(name, topic, required) for name, topic, _, required in self.specs],
            channel_map=self.channel_map, command=self.command, matrix=self.matrix,
            manual=self.active_test == 'manual',
            sensor_override=self.args.allow_missing_sensors,
            chips=self.args.pwm_chips, channels=self.args.pwm_channels,
            pwm_readback=self.pwm_readback,
            bag='RECORDING' if bag and bag.poll() is None else 'OFF', process_summary=processes),
            self.console.size.width, self.console.size.height)

    def required_tf(self):
        chains = [(self.args.map_frame, self.args.odom_frame),
                  (self.args.odom_frame, self.args.base_frame)]
        sensors = self.monitor.snapshot(required_tf=())['sensors']
        for name in ('GPS', 'IMU', 'LiDAR', 'camera'):
            frame = sensors.get(name, {}).get('values', {}).get('sensor_frame')
            if frame and frame != self.args.base_frame:
                chain = (self.args.base_frame, frame)
                if chain not in chains:
                    chains.append(chain)
        return tuple(chains)

    def check_command_topic(self):
        publishers = [info for info in
                      self.node.get_publishers_info_by_topic('/cmd_vel')
                      if info.node_name != self.node.get_name()]
        if publishers:
            names = ', '.join(f'{info.node_namespace}/{info.node_name}'
                              for info in publishers)
            raise RuntimeError(f'/cmd_vel already has publishers: {names}')

    def send_pwm(self, values, duration=0.0):
        """Hold direct Odroid PWM values for an identification phase."""
        deadline = time.monotonic() + duration
        with KeyReader() as keys:
            while True:
                self.pwm_outputs.set_pulse_widths(values)
                if time.monotonic() >= deadline:
                    break
                if keys.read(timeout=min(.05, max(0.0, deadline - time.monotonic()))) in ('space', 'q', 'escape'):
                    self.pwm_outputs.neutral()
                    raise RuntimeError('identification cancelled; neutral')

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
                elif key in ('q', 'escape'):
                    raise RuntimeError('selection cancelled')
                live.update(render())

    def identify_thrusters(self):
        self.console.print(Panel(
            'Boat secured; all propellers clear and submerged.\n'
            'Identify outputs against 1=FL, 2=FR, 3=RR, 4=RL.',
            title='Thruster identification', style='yellow'))
        self.pwm_outputs = OdroidPWMOutputs(
            self.args.pwm_chips, self.args.pwm_channels,
            self.args.frequency,
            mosfet_chip=self.args.mosfet_chip,
            mosfet_line=self.args.mosfet_line,
            mosfet_active_high=self.args.mosfet_active_high).open()
        observations = {}
        used = set()
        try:
            self.send_pwm([NEUTRAL_PWM] * 4, 5.0)
            for channel in range(1, 5):
                self.console.input(f'Press Enter to pulse physical output {channel} at 15%... ')
                values = [NEUTRAL_PWM] * 4
                values[channel - 1] += round(PWM_SCALE * 0.15)
                self.emit('identification_start', physical_channel=channel)
                self.send_pwm(values, 0.75)
                self.send_pwm([NEUTRAL_PWM] * 4, 0.5)
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
            if self.pwm_outputs is not None:
                try:
                    self.send_pwm([NEUTRAL_PWM] * 4, 0.5)
                finally:
                    self.pwm_outputs.close()
                    self.pwm_outputs = None
        return channel_map_from_observations(observations)

    def start_hardware(self):
        if any(info.node_name == 'thruster_pwm_node' for info in
               self.node.get_subscriptions_info_by_topic('/cmd_vel')):
            raise RuntimeError('another thruster driver already subscribes to /cmd_vel')
        flat_matrix = [value for row in self.matrix for value in row]
        map_yaml = '[' + ','.join(str(value) for value in self.channel_map) + ']'
        matrix_yaml = '[' + ','.join(str(value) for value in flat_matrix) + ']'
        chips_yaml = '[' + ','.join(f'"{value}"' for value in
                                    self.args.pwm_chips) + ']'
        channels_yaml = '[' + ','.join(str(value) for value in
                                       self.args.pwm_channels) + ']'
        self.start_process('hardware', [
            'ros2', 'run', 'mhseals_hardware', 'thruster_pwm_node',
            '--ros-args', '-p', f'pwm_chips:={chips_yaml}',
            '-p', f'pwm_channels:={channels_yaml}',
            '-p', f'frequency:={self.args.frequency}',
            '-p', f'mosfet_chip:={self.args.mosfet_chip}',
            '-p', f'mosfet_line:={self.args.mosfet_line}',
            '-p', f'mosfet_active_high:={str(self.args.mosfet_active_high).lower()}',
            '-p', f'command_timeout:={self.args.command_timeout}',
            '-p', f'channel_map:={map_yaml}',
            '-p', f'thruster_matrix:={matrix_yaml}',
        ])
        time.sleep(5)
        if self.processes['hardware'].poll() is not None:
            raise RuntimeError('thruster PWM node exited during startup')
        self.command_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.armed = True
        self.emit('configuration', channel_map=self.channel_map,
                  thruster_numbers=THRUSTER_NUMBERS,
                  thruster_rotations=self.thruster_rotations,
                  thruster_matrix=flat_matrix,
                  convention='+x forward, +y port, +yaw counterclockwise')

    def publish_command(self, axis=None, value=0.0, duration=0.1):
        if self.command_publisher is None:
            return
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
            self.command = (message.linear.x, message.linear.y, message.angular.z)
            time.sleep(0.1)

    def timed_phase(self, axis, value, duration):
        """Single-owner command loop: faults/Space cannot leave a worker driving."""
        deadline = time.monotonic() + duration
        try:
            with KeyReader() as keys, Live(self.dashboard(), console=self.console,
                                           screen=True, refresh_per_second=10) as live:
                while time.monotonic() < deadline:
                    self.assert_motion_ready()
                    self.notice = f'{axis or "NEUTRAL"} {value:+.0%} | {deadline-time.monotonic():.1f}s | Space abort'
                    key = keys.read(timeout=.05)
                    if key in ('space', 'q', 'escape'):
                        raise RuntimeError('trial cancelled by operator')
                    message = Twist()
                    if axis == 'surge':
                        message.linear.x = value
                    elif axis == 'sway':
                        message.linear.y = value
                    elif axis == 'yaw':
                        message.angular.z = value
                    self.command_publisher.publish(message)
                    self.command = (message.linear.x, message.linear.y, message.angular.z)
                    live.update(self.dashboard())
        finally:
            self.publish_command(duration=.1)

    def run_axis(self, axis):
        counts = Counter()
        self.active_test = axis
        self.emit('test_set_start', axis=axis)
        try:
            for direction in (1, -1) * self.args.repeats:
                counts[direction] += 1
                clearance, movement = AXIS_GUIDANCE[axis][direction]
                sign = '+' if direction > 0 else '-'
                self.console.print(
                    f'\n[bold]{axis.upper()} {sign} — {counts[direction]}/{self.args.repeats}[/]: '
                    f'clearance {clearance}; expected to {movement}.')
                self.console.input('Clear the area and press Enter to run... ')
                self.assert_motion_ready()
                repeat = counts[direction]
                self.emit('baseline_start', axis=axis, direction=direction,
                          repeat=repeat)
                baseline_start = time.monotonic()
                self.timed_phase(None, 0.0, self.args.baseline_duration)
                self.emit('command_start', axis=axis, direction=direction,
                          amplitude=self.args.amplitude,
                          duration=self.args.command_duration, repeat=repeat)
                command_start = time.monotonic()
                self.timed_phase(axis, direction * self.args.amplitude,
                                 self.args.command_duration)
                command_end = time.monotonic()
                self.emit('command_stop', axis=axis, direction=direction,
                          repeat=repeat)
                self.timed_phase(None, 0.0, self.args.settle_duration)
                self.emit('settle_stop', axis=axis, direction=direction,
                          repeat=repeat)
                result = self.monitor.summarize_trial(axis, direction, baseline_start,
                                                      command_start, command_end)
                self.emit('trial_result', **result)
                self.save_report()
            self.completed[axis] += 1
            self.emit('test_set_complete', axis=axis,
                      run_number=self.completed[axis])
        finally:
            self.publish_command(duration=0.5)
            self.active_test = None

    def select_test(self):
        """Select a test from the live dashboard with arrows or shortcuts."""
        shortcuts = {'1': 'surge', '2': 'sway', '3': 'yaw', 'm': 'manual control',
                     'h': 'arm', 'r': 'remap', 'i': 'identify', 'b': 'record',
                     'e': 'export', 'q': 'finish and save bag', 'space': 'stop'}
        with KeyReader() as keys, Live(
                self.dashboard(), console=self.console, screen=True,
                refresh_per_second=8) as live:
            while True:
                key = keys.read(timeout=0.1)
                if key in ('\t', 'right', 'left'):
                    self.view = VIEWS[(VIEWS.index(self.view) + (-1 if key == 'left' else 1)) % len(VIEWS)]
                elif key in ('+', '=', '-', '_'):
                    self.args.amplitude = round(max(.05, min(.5, self.args.amplitude +
                                                (.05 if key in ('+', '=') else -.05))), 2)
                    self.notice = f'Trial effort {self.args.amplitude:.0%}; {self.args.command_duration:g}s per pulse'
                elif key in shortcuts:
                    return shortcuts[key]
                if self.armed and self.processes['hardware'].poll() is not None:
                    self.disarm()
                    self.notice = 'Hardware process exited; disarmed'
                live.update(self.dashboard())

    def disarm(self):
        try:
            self.publish_command(duration=.2)
        finally:
            self.stop_process('hardware')
            if self.command_publisher is not None:
                self.node.destroy_publisher(self.command_publisher)
                self.command_publisher = None
            self.armed = False
            self.command = (0., 0., 0.)

    def assert_motion_ready(self):
        if self.monitor_fault:
            raise RuntimeError('monitor fault: ' + self.monitor_fault)
        if not self.armed or self.processes['hardware'].poll() is not None:
            raise RuntimeError('hardware is not armed; press H first')
        self.check_command_topic()
        snapshot = self.monitor.snapshot(self.required_tf())
        missing = [name for name, _, _, required in self.specs if required and
                   snapshot['sensors'].get(name, {}).get('status') != 'LIVE']
        if missing and not self.args.allow_missing_sensors:
            raise RuntimeError('sensor interlock: ' + ', '.join(missing))
        if self.args.require_tf and any(row['status'] != 'LIVE' for row in snapshot['tf']):
            raise RuntimeError('TF interlock: required chains are not healthy')

    def confirm_hardware(self, action):
        self.console.print(Panel(
            f'{action}\nREAL physical output. Propellers must be submerged; area clear.\n'
            'Keep a physical power cutoff available. Space aborts active phases.\n'
            + ('SENSOR INTERLOCK OVERRIDDEN by --allow-missing-sensors.\n'
               if self.args.allow_missing_sensors else '') +
            'Type ARM to confirm; anything else cancels.', style='yellow'))
        if self.console.input('Confirm: ').strip() != 'ARM':
            raise RuntimeError('arming cancelled')
        self.check_command_topic()

    def edit_mapping(self):
        self.disarm()
        available = [1, 2, 3, 4]
        mapping = []
        for position in POSITIONS:
            chosen = self.select_option(
                f'Physical output for {POSITION_NAMES[position]} (Q cancels)',
                tuple(str(number) for number in available),
                {str(number): index for index, number in enumerate(available)})
            mapping.append(int(chosen))
            available.remove(int(chosen))
        self.console.print(f'FL, FR, RR, RL -> {mapping}')
        if self.select_option('Persist mapping?', ('cancel', 'save'), {'s': 1}) != 'save':
            return
        self.persist_mapping(mapping)

    def persist_mapping(self, mapping):
        mapping = validate_channel_map(mapping)
        updated = dict(self.args.hardware_config, channel_map=list(mapping))
        saved = save_config(updated, self.args.config)
        self.args.hardware_config = updated
        self.args.channel_map = mapping
        self.channel_map = mapping
        self.emit('mapping_saved', channel_map=mapping, path=str(saved))
        self.notice = f'Mapping saved: {saved}; H to re-arm'

    def toggle_recording(self):
        process = self.processes.get('bag')
        if process is not None and process.poll() is None:
            self.stop_process('bag', timeout=20)
            self.notice = 'Recording stopped and flushed'
            return
        target = self.args.bag_output
        index = 1
        while target.exists():
            target = self.args.bag_output.with_name(self.args.bag_output.name + f'_{index:02d}')
            index += 1
        target.parent.mkdir(parents=True, exist_ok=True)
        self.start_process('bag', ['ros2', 'bag', 'record', '-a', '-o', str(target)])
        self.bag_started = True
        self.notice = f'Recording requested: {target}'
        self.emit('recording_requested', path=str(target))

    def save_report(self):
        import tempfile
        report = self.args.bag_output.with_suffix('.report.json')
        report.parent.mkdir(parents=True, exist_ok=True)
        data = dict(time=datetime.now().astimezone().isoformat(),
                    hardware_config=self.args.hardware_config,
                    topics={name: topic for name, topic, _, _ in self.specs},
                    telemetry=self.monitor.snapshot(self.required_tf()))
        fd, temporary = tempfile.mkstemp(prefix='.boat-report-', dir=report.parent)
        try:
            with os.fdopen(fd, 'w') as output:
                json.dump(data, output, indent=2, allow_nan=False)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, report)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        self.notice = f'Report saved: {report}'
        return report

    def manual_publish(self, message):
        # Final neutral must always be permitted even after an interlock fails.
        command = (message.linear.x, message.linear.y, message.angular.z)
        if any(command):
            self.assert_motion_ready()
        self.command = command
        if self.command_publisher is not None:
            self.command_publisher.publish(message)

    def manual_dashboard(self, label, amplitude, event_mode):
        self.notice = f'MANUAL {amplitude:.0%} | ' + ('multi-key' if event_mode else 'single-key fallback')
        return self.dashboard()

    def characterize(self):
        while True:
            choice = self.select_test()
            if choice == 'finish and save bag':
                return
            try:
                if self.args.monitor_only and choice not in ('stop', 'record', 'export'):
                    self.notice = 'Monitor-only mode: physical/configuration actions disabled'
                    continue
                if choice in ('stop', 'arm') and self.armed:
                    self.disarm()
                    self.notice = 'Disarmed; outputs neutral/disabled'
                elif choice == 'arm':
                    self.confirm_hardware('Arm PWM at neutral')
                    self.start_hardware()
                    self.notice = 'Armed at neutral; select M or a test'
                elif choice == 'remap':
                    self.edit_mapping()
                elif choice == 'identify':
                    self.disarm()
                    self.confirm_hardware('Identify four outputs with individual pulses')
                    self.persist_mapping(self.identify_thrusters())
                elif choice == 'record':
                    self.toggle_recording()
                elif choice == 'export':
                    self.save_report()
                elif choice == 'manual control':
                    self.assert_motion_ready()
                    self.emit('manual_control_start')
                    self.active_test = 'manual'
                    try:
                        run_manual(SimpleNamespace(publish=self.manual_publish), self.console,
                                   self.args.manual_amplitude, self.args.manual_repeat_timeout,
                                   self.args.manual_initial_timeout,
                                   status_panel=self.manual_dashboard)
                    finally:
                        self.publish_command(duration=.2)
                        self.emit('manual_control_stop')
                        self.active_test = None
                elif choice in AXIS_GUIDANCE:
                    self.assert_motion_ready()
                    self.run_axis(choice)
                    self.view = 'results'
            except (RuntimeError, OSError, ValueError) as error:
                self.disarm()
                self.notice = str(error)
                self.emit('action_cancelled', reason=str(error))

    def run(self):
        if self.args.start_stack:
            if self.node.get_publishers_info_by_topic('/imu/raw'):
                raise RuntimeError('IMU already has a publisher; use passive attachment instead of --start-stack')
            self.start_measurement_stack()
        if self.args.record:
            self.toggle_recording()
        if self.args.identify_thrusters:
            self.confirm_hardware('Identify physical outputs')
            self.persist_mapping(self.identify_thrusters())
        self.characterize()

    def shutdown(self):
        if self.command_publisher is not None:
            try:
                self.emit('shutdown', completed=dict(self.completed))
                self.publish_command(duration=1.0)
            except Exception:
                pass
        if self.pwm_outputs is not None:
            try:
                self.pwm_outputs.neutral()
            except Exception:
                pass
            finally:
                self.pwm_outputs.close()
        self.stop_process('hardware')
        self.stop_process('bag', timeout=20)
        self.stop_process('sensors')
        self.stop_process('odometry')
        self._spinning = False
        self.spin_thread.join(timeout=1)
        self.executor.shutdown(timeout_sec=1)
        self.node.destroy_node()


def prompt_hardware(args):
    console = Console()
    if args.fcu_url is None:
        default_url = default_fcu_url()
        args.fcu_url = console.input(
            f'MAVROS FCU URL [cyan][{default_url}][/]: ').strip() or default_url


def build_parser():
    parser = argparse.ArgumentParser(
        description='Passive-first sensor dashboard and opt-in boat characterization',
        allow_abbrev=False)
    parser.add_argument('--fcu-url', help='MAVROS FCU URL')
    parser.add_argument('--start-stack', action='store_true', help='launch odom stack; default attaches to existing topics only')
    parser.add_argument('--record', action='store_true', help='start rosbag recording on entry (B toggles)')
    parser.add_argument('--monitor-only', action='store_true', help='disable physical actions and mapping changes')
    parser.add_argument('--odom-topic', default='/odom/local')
    parser.add_argument('--camera-topic', default='/front_camera/rgb/image')
    parser.add_argument('--camera-type', choices=('raw', 'compressed'), default='raw')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--require-tf', action='store_true', help='interlock motion on required TF-chain health as well as sensors')
    parser.add_argument('--repeats', type=int, default=1, help='trials per direction (1..5)')
    parser.add_argument('--identify-thrusters', action='store_true',
                        help='pulse outputs to identify and persist a new channel map')
    parser.add_argument('--pwm-chips',
                        type=lambda value: tuple(v.strip() for v in value.split(',')),
                        default=DEFAULT_PWM_CHIPS,
                        help='four comma-separated Linux pwmchip paths')
    parser.add_argument('--pwm-channels', type=parse_int_list,
                        default=(0, 0, 0, 0))
    parser.add_argument('--frequency', type=float, default=50.0)
    parser.add_argument('--mosfet-chip', help='GPIO bank label or explicit device path')
    parser.add_argument('--mosfet-line', type=int)
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
    parser.add_argument('--stale-after', type=float, default=2.0)
    parser.add_argument('--amplitude', type=float, default=0.25)
    parser.add_argument('--manual-amplitude', type=float, default=0.9)
    parser.add_argument('--baseline-duration', type=float, default=5.0)
    parser.add_argument('--command-duration', type=float, default=3.0)
    parser.add_argument('--settle-duration', type=float, default=5.0)
    parser.add_argument('--bag-output', type=Path,
                        default=Path('bags') / ('boat_test_' +
                        datetime.now().strftime('%Y%m%d_%H%M%S')))
    return parser


def main(args=None):
    argv = sys.argv[1:] if args is None else list(args)
    parser = build_parser()
    if '--monitor-only' in argv and ('--save-config' in argv or '--identify-thrusters' in argv):
        parser.error('--monitor-only cannot identify thrusters or save hardware configuration')
    parsed = configure_args(parser, argv)
    if parsed.start_stack:
        prompt_hardware(parsed)
    for name in ('stale_after', 'baseline_duration', 'command_duration', 'settle_duration'):
        value = getattr(parsed, name)
        if not math.isfinite(value) or not 0 < value <= 60:
            raise ValueError(f'{name} must be within (0, 60] seconds')
    if not math.isfinite(parsed.amplitude) or not 0 < parsed.amplitude <= .5:
        raise ValueError('test amplitude must be within (0, 0.5]')
    if not 1 <= parsed.repeats <= 5:
        raise ValueError('repeats must be within 1..5')
    if parsed.optional_sensors and not parsed.start_stack:
        raise ValueError('--optional-sensors requires --start-stack')
    if parsed.monitor_only and (parsed.identify_thrusters or parsed.save_config):
        raise ValueError('--monitor-only cannot identify thrusters or save hardware configuration')
    parsed.thruster_matrix = tuple(
        value for row in validate_mixer(parsed.thruster_matrix) for value in row)
    if parsed.channel_map is not None:
        parsed.channel_map = validate_channel_map(parsed.channel_map)
    if parsed.bag_output.exists():
        raise ValueError(f'bag output already exists: {parsed.bag_output}')
    rclpy.init()
    test = BoatTest(parsed)
    try:
        test.run()
    except (KeyboardInterrupt, EOFError):
        Console().print('\n[yellow]Interrupted; neutralizing and flushing bag.[/]')
    finally:
        try:
            test.shutdown()
        finally:
            rclpy.try_shutdown()
    Console().print('Boat test closed; hardware disarmed. Recordings/reports are only created when requested.')


if __name__ == '__main__':
    main()
