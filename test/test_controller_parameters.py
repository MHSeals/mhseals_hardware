"""Actual controller callbacks with inert Jazzy-order parameter storage."""
from array import array
import ast
import copy
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import numpy
import pytest

from mhseals_hardware import control_validation as validation
from mhseals_hardware.velocity_law import Plant, VelocityLaw
from test_velocity_law import config


class ParameterOwner:
    """Jazzy 7.1.12: validate all callbacks, store, then post-set callbacks."""
    def set_parameters_atomically(self, parameters):
        for callback in (self._validate_gains, self.late):
            result = callback(parameters)
            if not result.successful:
                return result
        self.values.update({p.name: p.value for p in parameters})
        self._commit_gains(parameters)
        return result

    def set_parameters(self, parameters):
        return [self.set_parameters_atomically([p]) for p in parameters]


def node():
    path = Path(__file__).parents[1] / 'mhseals_hardware/velocity_controller.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name != '__init__']
    cls.bases = [ast.Name(id='ParameterOwner', ctx=ast.Load())]
    env = dict(vars(validation), ParameterOwner=ParameterOwner, copy=copy, numpy=numpy,
               SetParametersResult=lambda **kw: NS(**kw), time=NS(monotonic=lambda: 1.),
               twist_values=lambda x: x, twist_from=lambda x: tuple(x))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), env)
    n = env['VelocityController']()
    n._law_lock = threading.RLock()
    n._law = VelocityLaw(Plant(config()))
    n.values = {key: getattr(n._law, key).tolist() if key in validation.VECTOR_GAINS
                else getattr(n._law, key) for key in validation.GAIN_KEYS}
    n.values.update(command_timeout=.5, feedback=True)
    n.get_parameter = lambda key: NS(value=n.values[key])
    n.late = lambda ps: NS(successful=True)
    n._fault, n.sent = '', []
    n._pub = NS(publish=n.sent.append)
    return n


def patch(**kw):
    return [NS(name=k, value=v) for k, v in kw.items()]


def test_late_rejection_does_not_change_law_or_consumed_effort():
    n = node()
    n._law.observe((0, 0, 0), 0, 1.)
    n._command((.12, 0, 0))
    before, law = n.sent[-1], n._law
    n.late = lambda ps: NS(successful=False, reason='later callback')
    assert not n.set_parameters_atomically(patch(kp=[9., 9., 9.])).successful
    assert n._law is law
    n._command((.12, 0, 0))
    assert n.sent[-1] == pytest.approx(before)


def test_commit_preserves_observer_and_reprojects_integral():
    n = node()
    old = n._law
    old.drift[:] = (.1, -.2)
    old.integral[:] = .2
    assert n.set_parameters_atomically(patch(kp=[2., 2., 2.], integral_limit=[.1]*3)).successful
    assert n._law is not old and n._law.plant is old.plant
    assert n._law.integral == pytest.approx([.1]*3)
    assert old.integral == pytest.approx([.2]*3)
    assert n._law.drift == pytest.approx([.1, -.2])
    n._law.observe((0, 0, 0), 0, 1.)
    n._command((.12, 0, 0))
    assert n.sent[-1][0] > .2  # actual new PI law consumed


@pytest.mark.parametrize('name', validation.RESTART_KEYS)
def test_restart_parameter_refused(name):
    n = node()
    old = n._law
    assert not n.set_parameters_atomically(patch(**{name: '/new'})).successful
    assert n._law is old


def test_commands_cannot_observe_partially_committed_batch():
    n = node()
    entered, release, done = threading.Event(), threading.Event(), threading.Event()
    def late(ps):
        entered.set()
        assert release.wait(2)
        return NS(successful=True)
    n.late = late
    setter = threading.Thread(target=lambda: n.set_parameters_atomically(patch(kp=[2.]*3, ki=[3.]*3)))
    setter.start()
    assert entered.wait(2)
    command = threading.Thread(target=lambda: (n._command((.12, 0, 0)), done.set()))
    command.start()
    assert not done.wait(.03)
    release.set()
    setter.join(2)
    command.join(2)
    assert done.is_set()
    assert list(n._law.kp) == [2.]*3 and list(n._law.ki) == [3.]*3


def test_ros_service_double_array_consumes_new_law_and_rejection_retains_it():
    n = node()
    n._law.observe((0, 0, 0), 0, 1.)
    n._command((.12, 0, 0))
    before = n.sent[-1][0]
    assert n.set_parameters_atomically(patch(kp=array('d', [2.]*3))).successful
    n._command((.12, 0, 0))
    assert n.sent[-1][0] > before
    accepted = n._law
    assert not n.set_parameters_atomically(patch(ki=array('d', [float('nan')]*3))).successful
    assert n._law is accepted
    n.late = lambda ps: NS(successful=False)
    assert not n.set_parameters_atomically(patch(kp=array('d', [3.]*3))).successful
    assert n._law is accepted
