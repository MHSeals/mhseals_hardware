"""Exercise actual node methods without importing ROS or constructing outputs."""
import ast
import json
import signal
import time  # noqa: F401  (the extracted node methods read it from this namespace)
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from mhseals_hardware.configuration import config_sha, load_config, validate_config
from mhseals_hardware.estop import RemoteEstop
from mhseals_hardware.outputs import OUTPUT_KEYS
from mhseals_hardware.thruster_mixer import (
    validate_channel_map, validate_decoupling, validate_mixer, validate_reverse_ratio,
    validate_reversed,
)

pytestmark = pytest.mark.smoke


def extract(file, name, **extra):
    path = Path(__file__).parents[1] / 'mhseals_hardware' / file
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name != '__init__']
    constants = [n for n in tree.body if isinstance(n, ast.Assign) and
                 any(isinstance(t, ast.Name) and t.id in
                     ('PARAMETER_KEYS', 'HEARTBEAT_STALL')
                     for t in n.targets)]
    env = dict(globals(), SetParametersResult=lambda **kw: NS(**kw),
               String=lambda **kw: NS(**kw), Parameter=lambda name, **kw: NS(name=name, **kw),
               **extra)
    exec(compile(ast.fix_missing_locations(ast.Module(body=constants + [cls], type_ignores=[])),
                 str(path), 'exec'), env)
    return env[name], env


def pwm():
    cls, env = extract('thruster_pwm_node.py', 'ThrusterPWMNode')
    node = cls()
    node.config = dict(load_config(None), output_backend='topic')  # the arm snapshot, -p routing
    node.outputs, node.session = NS(), 'inert'
    node.events, node.statuses = [], []
    node.status_pub = NS(publish=lambda msg: node.statuses.append(json.loads(msg.data)))
    node.test, node.source, node.output_pub = None, 'neutral', NS(publish=lambda msg: None)
    node.refused = node.ended = node.version = None
    log = lambda message, **_: node.events.append(message)  # noqa: E731
    node.get_logger = lambda: NS(info=log, error=log)
    node.declared = {key for key, value in node.config.items() if not isinstance(value, dict)}
    node.estop = RemoteEstop(node.outputs, node.config['command_mux'])
    node.apply(node.config)
    return node, env


def test_only_the_live_keys_change_in_a_running_driver():
    node, env = pwm()
    for key in node.declared - set(env['PARAMETER_KEYS']) | {'config_file'}:
        result = node.check_parameters([NS(name=key)])
        assert not result.successful and 'next arm' in result.reason
    assert node.check_parameters([NS(name='command_timeout', value=1.0)]).successful
    assert not hasattr(node, 'reload_callback') and not hasattr(node, 'sync_parameters')


def test_status_names_one_snapshot_sha_of_what_the_driver_runs():
    node, _ = pwm()
    node.publish_status()
    status = node.statuses[-1]
    assert status['config_sha'] == config_sha(dict(load_config(None), output_backend='topic'))
    assert not {'requested_config_sha', 'effective_config_sha', 'pending_restart_keys'} & set(status)
    node.commit_parameters([NS(name='command_timeout', value=1.0)])
    assert node.command_timeout == 1.0
    assert node.statuses[-1]['config_sha'] == config_sha(node.config) != status['config_sha']


def test_estop_latch_refuses_drive_and_publishes():
    node, _ = pwm()
    sent = []
    node.outputs.set_pulse_widths = sent.append
    node.outputs.neutral = lambda: sent.append('neutral')
    run, stop = [1500] * 5 + [2000, 1500, 1000], [1500] * 5 + [1000, 1500, 1000]
    node.rc_callback(NS(channels=run))
    node.rc_callback(NS(channels=stop))
    assert sent == ['neutral'] and node.estop.latched
    estop = node.statuses[-1]['estop']
    assert estop['latched'] and estop['cause'] == 'rc-stop' and estop['since_ns'] > 0
    node.drive([1600] * 4)
    node.rc_callback(NS(channels=run))            # switch back: still latched
    assert sent == ['neutral'] and node.estop.latched


class Line:
    """A held MOSFET enable recording its levels (heartbeat flips and the cut)."""

    def __init__(self):
        self.request, self.enabled, self.levels = object(), True, []

    def set_enabled(self, enabled):
        self.enabled = enabled
        self.levels.append(enabled)

    def beat(self):
        if self.enabled:
            self.levels.append('flip')


def powered_node():
    node, _ = pwm()
    node.outputs.mosfet = Line()
    node.outputs.neutral = lambda: None
    node.estop = RemoteEstop(node.outputs, node.config['command_mux'])
    return node


def test_operator_estop_service_cuts_latches_and_reports():
    node = powered_node()
    response = node.estop_service(None, NS())
    assert response.success and response.message == 'latched (operator)'
    assert node.outputs.mosfet.levels == [False]
    assert node.statuses[-1]['estop']['cause'] == 'operator'
    assert node.estop_service(None, NS()).success      # repeat: still off, same cause
    assert node.outputs.mosfet.levels == [False]


def test_operator_estop_reports_a_failed_cut():
    node = powered_node()

    def broken(enabled):
        raise OSError('line busy')
    node.outputs.mosfet.set_enabled = broken
    response = node.estop_service(None, NS())
    assert not response.success and 'may still be on' in response.message
    assert node.estop.latched


def test_heartbeat_gap_latches_instead_of_resuming():
    node = powered_node()
    node.last_beat = None
    node.heartbeat()                               # the first beat has no gap to judge
    node.heartbeat()
    assert node.outputs.mosfet.levels == ['flip', 'flip'] and not node.estop.latched
    node.last_beat -= 0.2                          # the node stalled past the circuit timeout
    node.heartbeat()
    assert node.estop.cause == 'stall' and node.outputs.mosfet.levels[-1] is False
    node.heartbeat()
    assert node.outputs.mosfet.levels == ['flip', 'flip', False]


def test_a_failed_cut_never_beats_and_the_watchdog_retries_it():
    node = powered_node()
    node.last_command_time, node.timed_out = time.monotonic(), False
    attempts = []

    def broken(enabled):
        attempts.append(enabled)
        raise OSError('line busy')
    working, node.outputs.mosfet.set_enabled = node.outputs.mosfet.set_enabled, broken
    node.last_beat = None
    node.heartbeat()
    assert not node.estop_service(None, NS()).success and node.estop.latched
    assert node.outputs.mosfet.enabled                # the write failed: still powered
    for _ in range(3):
        node.heartbeat()                              # latched: never flips the line on
    assert node.outputs.mosfet.levels == ['flip']
    node.watchdog_callback()
    assert attempts == [False, False]                 # the cut, then the watchdog's retry
    node.outputs.mosfet.set_enabled = working
    node.watchdog_callback()
    assert node.outputs.mosfet.levels == ['flip', False] and not node.outputs.mosfet.enabled
    node.watchdog_callback()
    node.heartbeat()
    assert node.outputs.mosfet.levels == ['flip', False]


def test_destroy_node_ignores_ctrl_c_and_sigterm_while_it_closes_the_outputs():
    cls, _ = extract('thruster_pwm_node.py', 'ThrusterPWMNode')
    seen = []

    class Base:
        def destroy_node(self):
            seen.append('destroyed')

    node = type('Node', (cls, Base), {})()
    node.outputs = NS(close=lambda: seen.append(
        (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))))
    before = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)
    node.destroy_node()
    assert seen == [(signal.SIG_IGN, signal.SIG_IGN), 'destroyed']
    assert (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)) == before
