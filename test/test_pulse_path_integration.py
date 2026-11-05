"""Test pulses end to end: the real command_mux and thruster_pwm_node, the topic outputs a sim reads.

The mux forwards /cmd_vel/applied at 25 Hz, zeros too while idle. A test pulse republished at
the TUI's loop rate must still reach /thrusters/pwm for its whole duration (11-04: it was
overwritten within 40 ms, so the boat never moved), and manual commands still drive after.
"""

import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import String, UInt16MultiArray  # noqa: E402

from mhseals_hardware import command_mux, thruster_pwm_node  # noqa: E402
from mhseals_hardware.configuration import load_defaults, save_config  # noqa: E402

PULSE = [1500, 1500, 1740, 1500]
NEUTRAL = [1500] * 4


@pytest.fixture
def rig(tmp_path, monkeypatch):
    path = tmp_path / 'hardware.yaml'
    save_config(dict(load_defaults(), output_backend='topic'), path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '197')
    rclpy.init(args=['--ros-args', '-p', f'config_file:={path}',
                     '-r', 'thruster_pwm_node:cmd_vel:=/cmd_vel/applied'])
    mux, driver = command_mux.CommandMux(), thruster_pwm_node.ThrusterPWMNode()
    probe = rclpy.create_node('probe')
    seen = {'pwm': [], 'output': []}
    probe.create_subscription(UInt16MultiArray, '/thrusters/pwm',
                              lambda m: seen['pwm'].append((time.monotonic(), list(m.data))), 50)
    probe.create_subscription(String, '/thruster_pwm_node/output',
                              lambda m: seen['output'].append(json.loads(m.data)), 50)
    pubs = dict(legacy=probe.create_publisher(UInt16MultiArray, '/thruster_pwm_node/test_pulses', 10),
                held=probe.create_publisher(String, '/thruster_pwm_node/test_pulses/held', 10),
                manual=probe.create_publisher(Twist, '/cmd_vel/manual', 10))
    executor = SingleThreadedExecutor()
    for node in (mux, driver, probe):
        executor.add_node(node)

    def run(seconds, send=None, every=0.1):
        end, last = time.monotonic() + seconds, 0.0
        while time.monotonic() < end:
            if send and time.monotonic() - last >= every:
                send()
                last = time.monotonic()
            executor.spin_once(timeout_sec=0.005)
    run(1.5)  # the mux state reaches the driver: idle
    try:
        yield run, pubs, seen
    finally:
        for node in (probe, driver, mux):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


def carried(seen, since, until):
    frames = [data for t, data in seen['pwm'] if since <= t <= until]
    return frames, sum(data == PULSE for data in frames)


@pytest.mark.parametrize('topic', ['legacy', 'held'])
def test_a_pulse_reaches_the_outputs_for_its_whole_duration(rig, topic):
    run, pubs, seen = rig
    if topic == 'legacy':
        send = lambda: pubs['legacy'].publish(UInt16MultiArray(data=PULSE))
    else:
        send = lambda: pubs['held'].publish(String(data=json.dumps(
            {'pulses': PULSE, 'hold_s': 1.0, 'seq': 3})))
    start = time.monotonic()
    run(3.0, send)
    frames, on = carried(seen, start + 0.2, time.monotonic())   # after the first message lands
    assert len(frames) > 40 and on == len(frames), f'{on} of {len(frames)} frames carried the pulse'
    assert seen['output'][-1]['pulses'] == PULSE and seen['output'][-1]['source'] == 'test'
    run(1.5)                                                     # the sender stops: neutral
    assert seen['pwm'][-1][1] == NEUTRAL


def test_manual_drives_after_a_pulse_ends(rig):
    run, pubs, seen = rig
    run(1.0, lambda: pubs['legacy'].publish(UInt16MultiArray(data=PULSE)))
    run(1.0)
    surge = Twist()
    surge.linear.x = 0.5
    run(1.0, lambda: pubs['manual'].publish(surge), every=0.05)   # the mux goes manual
    assert seen['pwm'][-1][1] not in (NEUTRAL, PULSE)
    assert seen['output'][-1]['source'] == 'cmd_vel'
