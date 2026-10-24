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
from mhseals_hardware.startup_config import SELECTION_PARAMETERS
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
    env = dict(vars(validation), SELECTION_PARAMETERS=SELECTION_PARAMETERS,
               ParameterOwner=ParameterOwner, copy=copy, numpy=numpy,
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


def constructor(overrides, resources):
    """Execute the unchanged class body, including __init__, with inert ROS seams."""
    from mhseals_hardware.configuration import ConfigWatcher
    from mhseals_hardware.startup_config import node_startup_inputs

    class InertNode(ParameterOwner):
        def __init__(self, name):
            self._parameter_overrides = {k: NS(value=v) for k, v in overrides.items()}
            self.values = {}
            self.late = lambda ps: NS(successful=True)

        def declare_parameter(self, name, default, *, ignore_override=False):
            parameter = self._parameter_overrides.get(name)
            self.values[name] = default if ignore_override or parameter is None else parameter.value

        def get_parameter(self, name):
            return NS(value=self.values[name])

        def create_publisher(self, kind, topic, depth):
            resources.append(('publisher', topic))
            return NS(publish=lambda message: resources.append(('message', topic, message)))

        def create_subscription(self, kind, topic, callback, depth):
            resources.append(('subscription', topic))

        def create_timer(self, *args):
            pass

        def add_on_set_parameters_callback(self, callback):
            pass

        def add_post_set_parameters_callback(self, callback):
            pass

        def get_logger(self):
            return NS(error=lambda message: resources.append(('error', message)))

    path = Path(__file__).parents[1] / 'mhseals_hardware/velocity_controller.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    clock = NS(now=1.)
    env = dict(vars(validation), SELECTION_PARAMETERS=SELECTION_PARAMETERS,
        Node=InertNode, copy=copy, numpy=numpy, threading=threading, Plant=Plant,
        VelocityLaw=VelocityLaw, node_startup_inputs=node_startup_inputs,
        ConfigWatcher=ConfigWatcher, PLANT_KEYS=('thruster_matrix', 'characterization',
            'decoupling', 'decoupling_negative'), Twist=lambda: (0., 0., 0.),
        Vector3Stamped=object, Odometry=object, yaw_of=lambda q: 0.,
        SetParametersResult=lambda **kw: NS(**kw),
        time=NS(monotonic=lambda: clock.now), twist_values=lambda x: x,
        twist_from=lambda x: tuple(x))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                 str(path), 'exec'), env)
    return env['VelocityController'], clock


def test_actual_constructor_consumes_yaml_and_explicit_invocation(tmp_path):
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(kp=[2., 0., 0.], ki=[0., 0., 0.],
        input_topic='/selected/input', output_topic='/selected/output', odom_topic='/selected/odom')
    save_config(selected, path)
    resources = []
    cls, _ = constructor(dict(config_file=str(path), config_root=str(tmp_path / 'snapshots'),
        config_scope='yaml', kp=array('d', [3., 0., 0.])), resources)
    n = cls()
    n._law.observe((0., 0., 0.), 0., 1.)
    n._command((.12, 0., 0.))
    # Independent expectation: model .12/.6 plus selected proportional 3*.12.
    assert resources[-1] == ('message', '/selected/output', pytest.approx((.56, 0., 0.)))
    assert ('subscription', '/selected/input') in resources
    assert ('subscription', '/selected/odom') in resources
    from mhseals_hardware.startup_config import startup_inputs
    doc, _ = startup_inputs(root=tmp_path / 'snapshots', scope='yaml')
    assert doc['baseline']['controller']['kp'] == [2., 0., 0.]
    assert doc['origins']['controller.kp'][-1]['source'] == 'ROS invocation'


def test_managed_controller_conflict_refuses_before_resources(tmp_path):
    from mhseals_hardware.startup_config import startup_inputs
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    save_config(config(), path)
    _, selected = startup_inputs(path, root=tmp_path / 'snapshots', scope='same')
    resources = []
    cls, _ = constructor(dict(config_root=str(tmp_path / 'snapshots'), config_scope='same',
        config_candidate=selected.ticket.candidate, input_topic='/conflict'), resources)
    with pytest.raises(ValueError, match='conflicts'):
        cls()
    assert resources == []


def test_constructor_consumes_pi_bounds_feedback_and_both_deadlines(tmp_path):
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(kp=[2., 0., 0.], ki=[4., 0., 0.],
        integral_limit=[.01]*3, drift_time_constant=0., odom_timeout=.11,
        command_timeout=.05)
    save_config(selected, path)
    resources = []
    cls, clock = constructor(dict(config_file=str(path), config_root=str(tmp_path / 'snapshots'),
        config_scope='consumption'), resources)
    n = cls()
    n._law.observe((0., 0., 0.), 0., 1.)
    n._command((.12, 0., 0.))
    assert resources[-1][-1] == pytest.approx((.44, 0., 0.))
    clock.now = 1.1
    n._command((.12, 0., 0.))
    assert resources[-1][-1] == pytest.approx((.48, 0., 0.))  # integral clipped to .01
    assert n.set_parameters_atomically(patch(feedback=False)).successful
    n._command((.12, 0., 0.))
    assert resources[-1][-1] == pytest.approx((.2, 0., 0.))
    clock.now = 1.12
    n._command((.12, 0., 0.))
    assert resources[-1][-1] == (0., 0., 0.)  # selected odometry deadline
    n._law.observe((0., 0., 0.), 0., clock.now)
    n._command((.12, 0., 0.))
    clock.now += .06
    n._watchdog()
    assert resources[-1][-1] == (0., 0., 0.)  # selected command deadline
    assert n._law.last_step is None


@pytest.mark.parametrize('command_limit,expected', [(.05, .1/.6), (.2, (.1-.02)/.6)])
def test_constructor_consumes_drift_time_and_command_gate(tmp_path, command_limit, expected):
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(feedback=False, drift_time_constant=.8,
        drift_command_limit=command_limit, odom_timeout=1.)
    save_config(selected, path)
    resources = []
    cls, clock = constructor(dict(config_file=str(path), config_root=str(tmp_path / 'snapshots'),
        config_scope='drift'), resources)
    n = cls()
    n._law.observe((0., 0., 0.), 0., 1.)
    n._command((.1, 0., 0.))
    # The first effort predicts .1 m/s; measured .2 leaves a .1 residual.
    # alpha=.2/(.8+.2), unless the selected command gate freezes learning.
    clock.now = 1.2
    n._law.observe((.2, 0., 0.), 0., clock.now)
    n._command((.1, 0., 0.))
    assert resources[-1][-1] == pytest.approx((expected, 0., 0.))


def test_constructor_restart_consumes_snapshot_then_fresh_yaml(tmp_path):
    from mhseals_hardware.configuration import save_config
    from mhseals_hardware.startup_config import startup_inputs
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(kp=[2., 0., 0.], ki=[0.]*3, output_topic='/first')
    save_config(selected, path)
    overrides = dict(config_file=str(path), config_root=str(tmp_path / 'snapshots'),
                     config_scope='same')
    resources = []
    cls, _ = constructor(overrides, resources)
    n = cls()
    n._law.integral[:] = .1
    law = n._law
    selected['controller'].update(kp=[3., 0., 0.], output_topic='/second')
    save_config(selected, path)
    n._watch.mtime = None
    n._watch.seen = n._watch._stat()
    n._reload()
    assert n._reload_error == 'restart required: controller'
    assert n._law is law and list(law.integral) == [.1]*3
    _, snapshot = startup_inputs(root=tmp_path / 'snapshots', scope='same')
    overrides.update(config_candidate=snapshot.ticket.candidate, kp=[2., 0., 0.])
    cls, _ = constructor(overrides, resources)
    same = cls()  # matching override retains the original captured authority
    same._law.observe((0., 0., 0.), 0., 1.)
    same._command((.12, 0., 0.))
    assert resources[-1] == ('message', '/first', pytest.approx((.44, 0., 0.)))
    overrides.pop('config_candidate')
    overrides.pop('kp')
    overrides['config_scope'] = 'fresh'
    cls, _ = constructor(overrides, resources)
    fresh = cls()
    fresh._law.observe((0., 0., 0.), 0., 1.)
    fresh._command((.12, 0., 0.))
    assert resources[-1] == ('message', '/second', pytest.approx((.56, 0., 0.)))
    # Returning the desired YAML to the original selected section clears pending state.
    selected['controller'].update(kp=[2., 0., 0.], output_topic='/first')
    save_config(selected, path)
    n._watch.mtime = None
    n._watch.seen = n._watch._stat()
    n._reload()
    assert n._reload_error == '' and n._law is law


@pytest.mark.parametrize('fault', ['missing', 'corrupt', 'wrong-candidate'])
def test_actual_constructor_selection_fault_creates_no_resources(tmp_path, fault):
    from mhseals_hardware.startup_config import startup_inputs
    _, selected = startup_inputs(root=tmp_path / 'snapshots', scope='same')
    overrides = dict(config_root=str(tmp_path / 'snapshots'), config_scope='same',
                     config_candidate=selected.ticket.candidate)
    artifact = tmp_path / 'snapshots/same/initializing/hardware-pure.json'
    if fault == 'missing':
        artifact.unlink()
    elif fault == 'corrupt':
        artifact.write_text('{}')
    else:
        overrides['config_candidate'] = 'wrong'
    resources = []
    cls, _ = constructor(overrides, resources)
    with pytest.raises((ValueError, FileNotFoundError)):
        cls()
    assert resources == []


def test_actual_constructor_rejects_legacy_recipe_before_resources(tmp_path):
    from mhseals_config._bootstrap import begin_bootstrap, finish_bootstrap
    from mhseals_config._evidence import Recipe
    from mhseals_hardware.startup_config import prepare_inputs
    prepared = prepare_inputs()
    legacy = 'hardware-pure-startup/v1'
    evaluators = {legacy: Recipe(legacy, lambda document, evidence: True)}
    recipes = {p: legacy for p in prepared['documents']}
    root = tmp_path / 'snapshots'
    ticket = begin_bootstrap(root, 'legacy', prepared['baseline'], prepared['documents'],
        recipes, source_identity=prepared['source'], participant_identities=recipes,
        closure='inert', evaluators=evaluators)
    observations = {p: dict(scope='legacy', candidate=ticket.candidate,
        source=prepared['source'], identity=legacy, recipe=legacy, evidence={}) for p in recipes}
    finish_bootstrap(root, ticket, observations, evaluators=evaluators)
    resources = []
    cls, _ = constructor(dict(config_root=str(root), config_scope='legacy'), resources)
    with pytest.raises(ValueError):
        cls()
    assert resources == []


@pytest.mark.parametrize('text', [
    'controller: &x {unknown: *x}',
    'controller: &x {controller: *x}',
    'controller: {kp: &x [*x, 0, 0]}',
    'unknown: &x {unknown: *x}',
])
def test_recursive_yaml_refuses_before_constructor_resources(tmp_path, text):
    path = tmp_path / 'recursive.yaml'
    path.write_text(text)
    resources = []
    cls, _ = constructor(dict(config_file=str(path),
        config_root=str(tmp_path / 'snapshots'), config_scope='recursive'), resources)
    with pytest.raises(ValueError) as error:
        cls()
    assert str(path) in str(error.value) and 'line=' in str(error.value)
    assert resources == []
