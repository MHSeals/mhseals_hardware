"""Actual controller callbacks with inert Jazzy-order parameter storage."""
from array import array
import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy
import pytest

from mhseals_hardware import control_validation as validation
from mhseals_hardware.command_arbitration import mux_input
from mhseals_hardware.configuration import plain
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
    env = dict(vars(validation),
               ParameterOwner=ParameterOwner, plain=plain, copy=copy, numpy=numpy, mux_input=mux_input,
               Twist=lambda: (0., 0., 0.),
               SetParametersResult=lambda **kw: NS(**kw), time=NS(monotonic=lambda: 1.),
               twist_values=lambda x: x, twist_from=lambda x: tuple(x))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), env)
    n = env['VelocityController']()
    n._law = VelocityLaw(Plant(config()), drift_limit=.3)  # below the test plant's .4 m/s
    n.values = {key: getattr(n._law, key).tolist() if key in validation.VECTOR_GAINS
                else getattr(n._law, key) for key in validation.GAIN_KEYS}
    n.values.update(command_timeout=.5, feedback=True)
    n.get_parameter = lambda key: NS(value=n.values[key])
    n.late = lambda ps: NS(successful=True)
    n._held, n.sent = False, []
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
    assert n.set_parameters_atomically(patch(drift_limit=.05 * 5 ** .5)).successful
    assert n._law.drift == pytest.approx([.05, -.1])  # reprojected onto the lowered limit
    n._law.observe((0, 0, 0), 0, 1.)
    n._command((.12, 0, 0))
    assert n.sent[-1][0] > .2  # actual new PI law consumed


def test_a_live_drift_limit_at_the_slowest_planar_limit_is_refused():
    n = node()
    law = n._law
    slowest = min(law.plant.limits[:2])
    result = n.set_parameters_atomically(patch(drift_limit=float(slowest)))
    assert not result.successful and 'drift_limit' in result.reason
    assert n._law is law and n.values['drift_limit'] == law.drift_limit


@pytest.mark.parametrize('name', validation.TOPIC_KEYS + ('config_file', 'command_timeout'))
def test_an_arm_parameter_is_refused(name):
    n = node()
    old = n._law
    result = n.set_parameters_atomically(patch(**{name: '/new'}))
    assert not result.successful and 'next arm' in result.reason
    assert n._law is old


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
    from mhseals_hardware.configuration import declared, load_config

    class InertNode(ParameterOwner):
        def __init__(self, name):
            self.values = {}
            self.late = lambda ps: NS(successful=True)

        def declare_parameter(self, name, default):
            self.values[name] = overrides.get(name, default)  # a -p override wins

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
    env = dict(vars(validation),
        Node=InertNode, plain=plain, copy=copy, numpy=numpy, Plant=Plant, mux_input=mux_input,
        VelocityLaw=VelocityLaw, load_config=load_config, declared=declared,
        Twist=lambda: (0., 0., 0.),
        Vector3Stamped=object, Odometry=object, String=object, STATE_QOS=1, yaw_of=lambda q: 0.,
        SetParametersResult=lambda **kw: NS(**kw),
        time=NS(monotonic=lambda: clock.now), twist_values=lambda x: x,
        twist_from=lambda x: tuple(x))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                 str(path), 'exec'), env)
    return lambda: driving(env['VelocityController']()), clock


AUTO = NS(data=json.dumps({'schema': 'mhseals.mux/1', 'input': 'auto'}))


def driving(n):
    """A controller whose mux reports auto: it drives."""
    n._mux_state(AUTO)
    return n


def test_actual_constructor_consumes_yaml_and_explicit_invocation(tmp_path):
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(kp=[2., 0., 0.], ki=[0., 0., 0.], odom_topic='/selected/odom')
    selected['topics'].update(auto='/selected/input', effort='/selected/output')
    save_config(selected, path)
    resources = []
    cls, _ = constructor(dict(config_file=str(path), kp=array('d', [3., 0., 0.])), resources)
    n = cls()
    n._law.observe((0., 0., 0.), 0., 1.)
    n._command((.12, 0., 0.))
    # Independent expectation: model .12/.6 plus selected proportional 3*.12.
    assert resources[-1] == ('message', '/selected/output', pytest.approx((.56, 0., 0.)))
    assert ('subscription', '/selected/input') in resources
    assert ('subscription', '/selected/odom') in resources


def test_constructor_consumes_pi_bounds_feedback_and_both_deadlines(tmp_path):
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(kp=[2., 0., 0.], ki=[4., 0., 0.],
        integral_limit=[.01]*3, drift_time_constant=0., odom_timeout=.11)
    selected['command_timeout'] = .05
    save_config(selected, path)
    resources = []
    cls, clock = constructor(dict(config_file=str(path)), resources)
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
    assert n._law.last_step is None  # selected command deadline: the law resets


@pytest.mark.parametrize('command_limit,expected', [(.05, .1/.6), (.2, (.1-.02)/.6)])
def test_constructor_consumes_drift_time_and_command_gate(tmp_path, command_limit, expected):
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(feedback=False, drift_time_constant=.8,
        drift_command_limit=command_limit, odom_timeout=1.)
    save_config(selected, path)
    resources = []
    cls, clock = constructor(dict(config_file=str(path)), resources)
    n = cls()
    n._law.observe((0., 0., 0.), 0., 1.)
    n._command((.1, 0., 0.))
    # The first effort predicts .1 m/s; measured .2 leaves a .1 residual.
    # alpha=.2/(.8+.2), unless the selected command gate freezes learning.
    clock.now = 1.2
    n._law.observe((.2, 0., 0.), 0., clock.now)
    n._command((.1, 0., 0.))
    assert resources[-1][-1] == pytest.approx((expected, 0., 0.))


def test_a_yaml_edit_waits_for_the_next_arm(tmp_path):
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(kp=[2., 0., 0.], ki=[0.]*3)
    selected['topics']['effort'] = '/first'
    save_config(selected, path)
    overrides = dict(config_file=str(path))
    resources = []
    cls, _ = constructor(overrides, resources)
    n = cls()
    law = n._law
    selected['controller']['kp'] = [3., 0., 0.]
    selected['topics']['effort'] = '/second'
    save_config(selected, path)
    assert not hasattr(n, '_reload') and n._law is law and list(law.kp) == [2., 0., 0.]
    fresh = constructor(overrides, resources)[0]()  # the next arm consumes the edited YAML
    fresh._law.observe((0., 0., 0.), 0., 1.)
    fresh._command((.12, 0., 0.))
    assert resources[-1] == ('message', '/second', pytest.approx((.56, 0., 0.)))


def test_constructor_consumes_obstacle_topic_distance_and_timeout(tmp_path):
    from mhseals_hardware.configuration import save_config
    path = tmp_path / 'hardware.yaml'
    selected = config()
    selected['controller'].update(obstacle_topic='/selected/obstacle', proximity_distance=.3,
                                  obstacle_timeout=.2)
    save_config(selected, path)
    resources = []
    cls, clock = constructor(dict(config_file=str(path)), resources)
    n = cls()
    assert ('subscription', '/selected/obstacle') in resources
    n._obstacle(NS(vector=NS(x=0., y=-.25, z=0.)))
    assert n._law.near_obstacle(clock.now + .2) == pytest.approx((0., -1.))
    assert n._law.near_obstacle(clock.now + .21) is None  # selected timeout
    n._obstacle(NS(vector=NS(x=0., y=-.35, z=0.)))
    assert n._law.near_obstacle(clock.now) is None         # selected distance
