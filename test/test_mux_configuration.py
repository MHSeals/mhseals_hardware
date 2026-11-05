"""Actual mux startup and live sets against inert ROS bindings and real arbitration."""
from array import array
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

pytestmark = pytest.mark.smoke

from mhseals_hardware.command_arbitration import Arbitration, INIT_KEYS, mux_section
from mhseals_hardware.configuration import declared, load_defaults, plain


def full(section):
    return {**mux_section(load_defaults()), **section}


def yaml_config(section):
    """``section`` as the file holds it: command_timeout top-level, the rest under command_mux."""
    rest = {k: v for k, v in section.items() if k != 'command_timeout'}
    top = {'command_timeout': section['command_timeout']} if 'command_timeout' in section else {}
    return {'command_mux': {**load_defaults()['command_mux'], **rest}, **top}


class InertNode:
    overrides = {}
    def __init__(self, name):
        self.parameters, self.logs = {}, []

    def declare_parameter(self, name, value):
        self.parameters[name] = self.overrides.get(name, value)  # a -p override wins

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def __getattr__(self, name):
        if name.startswith(('create_', 'add_')):
            return lambda *a: None
        raise AttributeError(name)

    def get_logger(self):
        return NS(info=self.logs.append, error=self.logs.append)


def mux(section, overrides=None, **replace):
    path = Path(__file__).parents[1] / 'mhseals_hardware/command_mux.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = [ast.Name(id='InertNode', ctx=ast.Load())]
    InertNode.overrides = overrides or {}
    change = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'change_key')
    env = dict(json=json, INIT_KEYS=INIT_KEYS, VOLATILE_RC=('age_s', 'rate_hz', 'interval_p95_s'),
               InertNode=InertNode, Arbitration=Arbitration,
               time=NS(monotonic=lambda: 0., time_ns=lambda: 1), Twist=object, String=lambda **kw: NS(**kw), RCIn=object,
               STATE_QOS=None, qos_profile_sensor_data=None,
               load_config=lambda path: dict(load_defaults(), **yaml_config(section)),
               mux_section=mux_section,
               declared=declared, plain=plain, MUX_SCHEMA='mhseals.mux/1',
               hold_store=NS(path=lambda section: section.get('state_file') or '/default',
                                load=lambda path: {}),
               SetParametersResult=lambda **kw: NS(**kw),
               **{n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
                  if isinstance(n, ast.Assign) and n.targets[0].id.endswith('_PARAMETERS')})
    env.update(replace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[change, cls], type_ignores=[])), str(path), 'exec'), env)
    return env['CommandMux']()


def test_empty_yaml_preserves_explicit_startup_scale_and_timeout():
    n = mux({}, dict(rc_scale=[.1, .2, .1], command_timeout=.9, ramp=2.))
    a = n.arbitration
    a.update_rc([1500, 2000, 1500, 1500, 1500, 1800, 1500, 1100], 0.)
    assert a.select(0.)[1][0] == pytest.approx(.1)
    a.update_manual((.4, 0, 0), 1.)
    assert a.select(1.8)[0] == 'manual'
    assert a.select(1.91)[0] == 'hold'  # stale RC safety hold, manual expired
    assert a.ramp == 2.
    b = mux({}, dict(command_timeout=.9, ramp=2.)).arbitration
    b.update_auto((1., 0, 0), 0.)
    assert b.select(0.)[1][0] == 0.
    b.update_auto((1., 0, 0), .5)
    assert b.select(.5)[1][0] == pytest.approx(.25)


def test_yaml_fallback_and_explicit_invocation_precedence():
    n = mux({'rc_scale': [.2]*3, 'command_timeout': .7, 'ramp': 0.}, {'command_timeout': .9})
    assert n.arbitration.rc_scale == (.2,)*3
    assert n.arbitration.command_timeout == .9


@pytest.mark.parametrize('name,value', [('config_file', '/x'), ('command_timeout', .9)])
def test_arm_parameter_write_refuses(name, value):
    n = mux({})  # command_timeout is shared with the driver and controller: fixed per arm
    result = n._check_parameters([NS(name=name, value=value)])
    assert not result.successful and result.reason == 'applies at the next arm: ' + name
    assert n.arbitration.command_timeout == .5


@pytest.mark.parametrize('name,value', [('rc_scale', [.5, -.1, .3]), ('rc_scale', [.5, .5]),
                                        ('ramp', -1.)])
def test_bad_live_parameter_writes_refuse_and_keep_the_old(name, value):
    n = mux({})
    result = n._check_parameters([NS(name=name, value=value)])
    assert not result.successful and result.reason
    assert n.arbitration.command_timeout == .5


def test_live_parameter_set_applies_at_once_even_while_driving():
    n = mux({})
    a = n.arbitration
    sets = [NS(name='rc_scale', value=array('d', [.2] * 3))]
    assert n._check_parameters(sets).successful
    n._commit_parameters(sets)
    assert a.rc_scale == (.2,) * 3
    a.update_manual((.4, 0, 0), 0.)
    n._commit_parameters([NS(name='ramp', value=2.)])
    assert a.ramp == 2. and n.arbitration is a  # tuning never remaps a channel, so no wait
    assert published(n)['effective']['rc_scale'] == [.2] * 3


def published(n):
    sent = []
    n.state_pub = NS(publish=sent.append)
    n.publish_state(force=True)
    return json.loads(sent[-1].data)['configuration']


def test_ros_array_override_is_json_safe_and_detached():
    scale = array('d', [.1, .2, .1])
    n = mux({}, {'rc_scale': scale})
    scale[0] = .9
    assert published(n)['effective']['rc_scale'] == [.1, .2, .1]
    assert n.arbitration.rc_scale == (.1, .2, .1)
