"""
Real thruster_pwm_node and command_mux processes refuse a duplicate-key reload.

Runs only where ROS is installed. Both nodes are separate processes on a private
ROS domain with localhost discovery; the driver uses the topic backend (nothing
opens GPIO/PWM) and the mux reads no RC. The test node reads only the published
``~/status`` and ``~/state``.
"""
import json
import os
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def write(path, text):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(text)
    os.replace(temporary, path)  # a new inode, as an editor's atomic save


def config(timeout, mux_timeout, state_file):
    return (f'output_backend: topic\ncommand_timeout: {timeout}\ncommand_mux:\n'
            f'  cmd_timeout: {mux_timeout}\n  state_file: {state_file}\n')


@pytest.fixture
def fleet(tmp_path):
    env = dict(os.environ, HOME=str(tmp_path), XDG_RUNTIME_DIR=str(tmp_path / 'run'),
               ROS_DOMAIN_ID=str(100 + os.getpid() % 100),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_LOCALHOST_ONLY='1')
    (tmp_path / 'run').mkdir()
    env.pop('MHSEALS_HARDWARE_CONFIG', None)
    path = tmp_path / 'hardware.yaml'
    write(path, config(0.3, 0.7, tmp_path / 'holds.json'))
    procs = []
    for module in ('thruster_pwm_node', 'command_mux'):
        procs.append(subprocess.Popen(
            [sys.executable, '-m', f'mhseals_hardware.{module}', '--ros-args',
             '-p', f'config_file:={path}'],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True))
    os.environ.update(ROS_DOMAIN_ID=env['ROS_DOMAIN_ID'],
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    rclpy.init()
    node = rclpy.create_node('reload_probe')
    seen = {}
    node.create_subscription(String, '/thruster_pwm_node/status',
                             lambda m: seen.__setitem__('pwm', json.loads(m.data)), LATCHED)
    node.create_subscription(String, '/command_mux/state',
                             lambda m: seen.__setitem__('mux', json.loads(m.data)), LATCHED)

    def wait(predicate, seconds=15.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(k in seen for k in ('pwm', 'mux')) and predicate(seen):
                return seen
        logs = [p.stdout.read() if p.poll() is not None else '' for p in procs]
        raise AssertionError(f'condition not reached: {json.dumps(seen)[:600]}\n{logs}')

    try:
        yield path, wait, tmp_path
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for proc in procs:
            proc.terminate()
        for proc in procs:
            try:
                proc.wait(5)
            except subprocess.TimeoutExpired:
                proc.kill()


def test_duplicate_key_reload_leaves_driver_and_mux_unchanged(fleet):
    path, wait, tmp_path = fleet
    state_file = tmp_path / 'holds.json'
    first = wait(lambda s: s['pwm']['error'] == '')
    sha0 = first['pwm']['effective_config_sha']
    assert first['mux']['configuration']['effective']['cmd_timeout'] == 0.7

    # Positive control: a valid edit is consumed by both nodes.
    write(path, config(0.4, 0.8, state_file))
    good = wait(lambda s: s['pwm']['effective_config_sha'] != sha0
                and s['mux']['configuration']['effective']['cmd_timeout'] == 0.8)
    sha1 = good['pwm']['effective_config_sha']
    assert good['pwm']['error'] == ''

    # A duplicate key is refused by both and neither applies a partial file.
    write(path, config(0.2, 0.9, state_file) + 'command_timeout: 0.1\n')
    bad = wait(lambda s: 'duplicate' in s['pwm']['error']
               and 'duplicate' in s['mux']['configuration']['error'])
    time.sleep(2.0)
    bad = wait(lambda s: True)
    assert bad['pwm']['effective_config_sha'] == sha1
    assert bad['mux']['configuration']['effective']['cmd_timeout'] == 0.8
    assert bad['mux']['configuration']['pending'] is None

    # Recovery with a valid file clears the error and applies.
    write(path, config(0.5, 0.6, state_file))
    fixed = wait(lambda s: s['pwm']['error'] == ''
                 and s['pwm']['effective_config_sha'] != sha1
                 and s['mux']['configuration']['error'] == ''
                 and s['mux']['configuration']['effective']['cmd_timeout'] == 0.6)
    assert fixed['pwm']['effective_config_sha'] not in (sha0, sha1)
