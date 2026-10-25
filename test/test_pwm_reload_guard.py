"""Exercise actual node methods without importing ROS or constructing outputs."""
import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from mhseals_hardware.configuration import ConfigWatcher, config_sha, load_defaults, save_config, validate_config
from mhseals_hardware.outputs import OUTPUT_KEYS
from mhseals_hardware.thruster_mixer import (
    validate_channel_map, validate_decoupling, validate_mixer, validate_reverse_ratio,
    validate_reversed,
)


def extract(file, name, **extra):
    path = Path(__file__).parents[1] / 'mhseals_hardware' / file
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name != '__init__']
    constants = [n for n in tree.body if isinstance(n, ast.Assign) and
                 any(isinstance(t, ast.Name) and t.id in
                     ('LIVE_KEYS', 'RESTART_KEYS', 'PLANT_KEYS') for t in n.targets)]
    env = dict(globals(), SetParametersResult=lambda **kw: NS(**kw),
               String=lambda **kw: NS(**kw), Parameter=lambda name, **kw: NS(name=name, **kw),
               **extra)
    exec(compile(ast.fix_missing_locations(ast.Module(body=constants + [cls], type_ignores=[])),
                 str(path), 'exec'), env)
    return env[name], env


def pwm():
    cls, env = extract('thruster_pwm_node.py', 'ThrusterPWMNode')
    node = cls()
    node.file_config = load_defaults()
    node.file_config['decoupling_negative'] = list(node.file_config['decoupling'])
    node.config = dict(node.file_config, output_backend='topic')  # startup override
    node.requested_config, node.pending_restart_keys = node.file_config, []
    node.outputs, node.error, node.session = NS(), '', 'inert'
    node.events, node.statuses = [], []
    node.status_pub = NS(publish=lambda msg: node.statuses.append(json.loads(msg.data)))
    node.get_logger = lambda: NS(info=node.events.append, error=node.events.append)
    node.declared, node.own_write = set(node.config), False
    node.has_parameter = lambda key: True
    node.get_parameter = lambda key: NS(value=node.config[key])
    node.set_parameters = lambda params: [NS(successful=True) for p in params]
    node.apply(node.config)
    return node, env


def request(node, candidate):
    node.watch = NS(poll=lambda: candidate)
    node.reload_callback()


CHANGES = {
    'output_backend': 'topic', 'pwm_topic': '/other', 'pwm_chips': ['/sys/other'] * 4,
    'pwm_channels': [1, 0, 0, 0], 'mosfet_chip': 'gpio4', 'mosfet_line': 3,
    'mosfet_active_high': False, 'frequency': 60, 'channel_map': [2, 1, 3, 4],
    'reversed': [True] * 4, 'thruster_matrix': [0.5] * 12, 'decoupling': [0.5] * 9,
    'decoupling_negative': [0.4] * 9, 'reverse_thrust_ratio': 0.5,
    'characterization': {'velocity_gain': {'surge': 2}},
    'hardware_epoch': 1,
}


@pytest.mark.parametrize('key', CHANGES)
@pytest.mark.parametrize('opened,mux', [(True, 'hold'), (None, 'idle'), (False, 'rc')])
def test_protected_reload_preserves_every_effective_object(key, opened, mux):
    node, env = pwm()
    assert set(CHANGES) == set(env['RESTART_KEYS'])
    if opened is not None:
        node.outputs.opened = opened
    node.timed_out, node.mux_input = True, mux
    before = dict(node.__dict__)
    candidate = copy.deepcopy(node.file_config)
    candidate.update({key: CHANGES[key], 'command_timeout': 1.0})
    node.apply = node.sync_parameters = lambda *a: pytest.fail('mutation before guard')
    request(node, candidate)
    for attr in ('config', 'file_config', 'outputs', 'mixer', 'decoupling', 'channel_map'):
        assert getattr(node, attr) is before[attr]
    assert node.command_timeout == 0.5
    status = node.statuses[-1]
    assert status['pending_restart_keys'] == [key]
    assert status['config_sha'] == config_sha(before['file_config'])
    assert status['requested_config_sha'] == config_sha(candidate)
    assert status['effective_config_sha'] == config_sha(before['config'])
    assert status['requested_config_sha'] != status['effective_config_sha']
    error = node.error
    request(node, None)
    assert node.error == error and node.pending_restart_keys == [key]


def test_coupled_refusal_then_scalar_correction_and_revert():
    node, _ = pwm()
    candidate = dict(node.file_config, decoupling=[0.5] * 9, characterization={'metadata': 1})
    for _ in range(2):
        request(node, candidate)
        assert node.pending_restart_keys == ['characterization', 'decoupling']
    mirrored = []
    node.set_parameters = lambda params: mirrored.extend(params) or [NS(successful=True)]
    request(node, dict(node.file_config, command_timeout=1))
    assert node.command_timeout == 1.0 and mirrored[0].value == 1.0
    assert node.config['output_backend'] == 'topic'
    assert not node.error and node.pending_restart_keys == []
    request(node, dict(node.file_config, reversed=[True] * 4))
    request(node, node.file_config)
    assert not node.error and node.pending_restart_keys == []


@pytest.mark.parametrize('value', [True, False, float('nan'), float('inf'), 0, -1, 2.1])
def test_invalid_timeout_preserves_watchdog(value):
    node, _ = pwm()
    old = node.file_config
    request(node, dict(old, command_timeout=value))
    assert node.error and node.file_config is old and node.command_timeout == 0.5


def test_parameter_writes_cannot_bypass_guard():
    node, env = pwm()
    for key in set(env['RESTART_KEYS']) | {'config_file', 'pulse_guard_topic'}:
        assert not node.refuse_external([NS(name=key)]).successful
    assert not node.refuse_external([NS(name='command_timeout')]).successful


def test_settled_watcher_leaves_requested_bytes_and_refusal_visible(tmp_path):
    node, _ = pwm()
    path = tmp_path / 'hardware.yaml'
    save_config(node.file_config, path)
    node.watch = ConfigWatcher(path)
    save_config(dict(node.file_config, reversed=[True] * 4), path)
    requested_bytes = path.read_bytes()
    node.reload_callback()  # first observation is not settled
    node.reload_callback()
    assert node.pending_restart_keys == ['reversed']
    node.reload_callback()  # unchanged file produces no event
    assert node.error == 'restart required: reversed'
    assert path.read_bytes() == requested_bytes
