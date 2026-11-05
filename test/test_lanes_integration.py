"""Lanes by unit, end to end: the real velocity controller, command_mux and driver on the topics.

A velocity step on topics.auto moves a simulated boat (a first-order surge plant fed by the
driver's topic outputs through the mux's applied effort), and manual effort reaches the driver
unchanged, bypassing the controller, which resets while the mux input is not auto. Once nav
stops, the mux goes idle and stays idle: the controller's reset publishes nothing.
"""

import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from mhseals_hardware import command_mux, thruster_pwm_node, velocity_controller  # noqa: E402
from mhseals_hardware.configuration import save_config  # noqa: E402
from test_velocity_law import config as characterized  # noqa: E402

TAU = 0.5        # s, the plant's surge time constant
TOP_SPEED = 1.0  # m/s at full surge effort


@pytest.fixture
def rig(tmp_path, monkeypatch):
    config = dict(characterized(), output_backend='topic')
    path = tmp_path / 'hardware.yaml'
    save_config(config, path)
    topics = config['topics']
    monkeypatch.setenv('ROS_DOMAIN_ID', '198')
    rclpy.init(args=['--ros-args', '-p', f'config_file:={path}'])
    nodes = [velocity_controller.VelocityController(), command_mux.CommandMux(),
             thruster_pwm_node.ThrusterPWMNode()]
    probe = rclpy.create_node('probe')
    nodes.append(probe)
    boat = {'x': 0.0, 'v': 0.0, 'applied': (0.0, 0.0, 0.0), 'effort': [], 'input': None,
            'inputs': [], 'command_timeout': config['command_timeout']}

    def applied(message):
        boat['applied'] = (message.linear.x, message.linear.y, message.angular.z)
    probe.create_subscription(Twist, topics['applied'], applied, 10)
    probe.create_subscription(Twist, topics['effort'],
                              lambda m: boat['effort'].append(m.linear.x), 10)
    def state(message):
        boat['input'] = json.loads(message.data)['input']
        boat['inputs'].append(boat['input'])
    probe.create_subscription(String, topics['mux_state'], state, command_mux.STATE_QOS)
    odom = probe.create_publisher(Odometry, config['controller']['odom_topic'], 10)
    pubs = {'auto': probe.create_publisher(Twist, topics['auto'], 10),
            'manual': probe.create_publisher(Twist, topics['manual'], 10)}
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    def run(seconds, **send):
        end, last = time.monotonic() + seconds, time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last >= 0.05:
                dt, last = now - last, now
                boat['v'] += (TOP_SPEED * boat['applied'][0] - boat['v']) * dt / TAU
                boat['x'] += boat['v'] * dt
                message = Odometry()
                message.header.frame_id = 'odom'
                message.pose.pose.orientation.w = 1.0
                message.twist.twist.linear.x = boat['v']
                odom.publish(message)
                for lane, surge in send.items():
                    command = Twist()
                    command.linear.x = surge
                    pubs[lane].publish(command)
            executor.spin_once(timeout_sec=0.005)
    run(1.0)
    try:
        yield run, boat
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


def test_a_velocity_step_moves_the_boat_and_manual_effort_bypasses_the_controller(rig):
    run, boat = rig
    assert boat['input'] == 'idle' and boat['x'] == pytest.approx(0.0, abs=1e-6)
    run(4.0, auto=0.3)                                     # a /cmd_vel step, m/s
    assert boat['input'] == 'auto'
    assert boat['x'] > 0.3 and boat['v'] > 0.1, boat       # the boat moved forward
    run(1.0, auto=0.3, manual=0.5)                         # keys while nav still commands
    assert boat['input'] == 'manual'
    assert boat['applied'] == pytest.approx((0.5, 0.0, 0.0))   # the keys' effort, unchanged
    assert boat['effort'][-1] == 0.0                       # the controller resets: neutral


def test_the_mux_stays_idle_after_nav_stops(rig):
    run, boat = rig
    run(1.0, auto=0.3)
    assert boat['input'] == 'auto'
    run(boat['command_timeout'] + 0.5)                         # nav stops: the auto lane goes stale
    assert boat['input'] == 'idle'
    boat['inputs'].clear()
    run(3 * boat['command_timeout'])                           # no zero from the controller refreshes it
    assert boat['input'] == 'idle' and 'auto' not in boat['inputs'], boat['inputs']
