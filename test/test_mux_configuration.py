"""Actual mux initialization/reload against inert ROS bindings and real arbitration."""
from array import array
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from mhseals_hardware.command_arbitration import Arbitration, INIT_KEYS


class InertNode:
    overrides = {}
    def __init__(self, name):
        self._parameter_overrides = self.overrides
        self.parameters, self.logs = {}, []

    def declare_parameter(self, name, value):
        self.parameters[name] = self._parameter_overrides.get(name, value)

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
               InertNode=InertNode, Arbitration=Arbitration, RELOAD_PERIOD=1.,
               time=NS(monotonic=lambda: 0., time_ns=lambda: 1), Twist=object, String=lambda **kw: NS(**kw), RCIn=object,
               STATE_QOS=None, qos_profile_sensor_data=None,
               node_startup_inputs=lambda node, path, **kw: ({'command_mux': section}, None),
               ConfigWatcher=lambda path: NS(poll=lambda: None),
               hold_store=NS(default_path=lambda: '/default', load=lambda path: {}),
               SetParametersResult=lambda **kw: NS(**kw))
    env.update(replace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[change, cls], type_ignores=[])), str(path), 'exec'), env)
    return env['CommandMux']()


def test_empty_yaml_preserves_explicit_startup_scale_and_timeout():
    n = mux({}, dict(rc_scale=[.1, .2, .1], cmd_timeout=.9, ramp=2.))
    a = n.arbitration
    a.update_rc([1500, 2000, 1500, 1500, 1500, 1800, 1500, 1100], 0.)
    assert a.select(0.)[1][0] == pytest.approx(.1)
    a.update_manual((.4, 0, 0), 1.)
    assert a.select(1.8)[0] == 'manual'
    assert a.select(1.91)[0] == 'hold'  # stale RC safety hold, manual expired
    assert a.ramp == 2.
    b = mux({}, dict(cmd_timeout=.9, ramp=2.)).arbitration
    b.update_auto((1., 0, 0), 0.)
    assert b.select(0.)[1][0] == 0.
    b.update_auto((1., 0, 0), .5)
    assert b.select(.5)[1][0] == pytest.approx(.25)


def test_yaml_fallback_and_explicit_invocation_precedence():
    n = mux({'rc_scale': [.2]*3, 'cmd_timeout': .7, 'ramp': 0.}, {'cmd_timeout': .9})
    assert n.arbitration.rc_scale == (.2,)*3
    assert n.arbitration.cmd_timeout == .9
    assert n._resolve_section({'cmd_timeout': .1})['cmd_timeout'] == .9


def test_reload_defers_human_then_applies_at_idle_and_state_file_refuses():
    n = mux({})
    old = n.arbitration
    old.update_manual((.4, 0, 0), 0.)
    n.watch.poll = lambda: {'command_mux': {'rc_scale': [.2]*3, 'cmd_timeout': .9}}
    n.reload_callback()
    assert n.pending and old.cmd_timeout == .5
    old.manual_time = None
    n.watch.poll = lambda: None
    n.reload_callback()
    assert n.pending is None and old.cmd_timeout == .9 and old.rc_scale == (.2,)*3
    n.watch.poll = lambda: {'command_mux': {'state_file': '/other', 'cmd_timeout': .2}}
    n.reload_callback()
    assert n._reload_error == 'restart required: state_file'
    assert n.pending is None and old.cmd_timeout == .9 and n.state_file == '/default'


@pytest.mark.parametrize('name', ['rc_scale', 'cmd_timeout', 'ramp', 'config_file'])
def test_unused_live_parameter_writes_refuse(name):
    n = mux({})
    assert not n._reject_parameters([NS(name=name)]).successful


def published(n):
    sent = []
    n.state_pub = NS(publish=sent.append)
    n.publish_state(force=True)
    return json.loads(sent[-1].data)['configuration']


def test_ros_array_override_status_and_pending_are_json_safe_and_detached():
    scale = array('d', [.1, .2, .1])
    n = mux({}, {'rc_scale': scale})
    scale[0] = .9
    assert published(n)['invocation_overrides']['rc_scale'] == [.1, .2, .1]
    n.arbitration.update_manual((.4, 0, 0), 0.)
    n.watch.poll = lambda: {'command_mux': {'ramp': 2.}}
    n.reload_callback()
    assert published(n)['pending']['rc_scale'] == [.1, .2, .1]
    assert n.arbitration.rc_scale == (.1, .2, .1)


def test_reload_error_is_current_and_clears_only_after_valid_acceptance():
    n = mux({})
    n._reload_error = 'restart required: state_file'
    old = n.arbitration
    def bad():
        raise ValueError('bad rc_scale')
    n.watch.poll = bad
    n.reload_callback()
    assert published(n)['error'] == 'config reload rejected: bad rc_scale'
    assert n.arbitration is old and old.cmd_timeout == .5
    n.watch.poll = lambda: {'command_mux': {'cmd_timeout': .9}}
    n.reload_callback()
    assert published(n)['error'] == '' and old.cmd_timeout == .9
