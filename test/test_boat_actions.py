"""Exercise the production action loop with deterministic input/output adapters."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


def action_fixture(keys, fault=False):
    clock = SimpleNamespace(now=0.)
    sent = []
    class Reader:
        def __enter__(self):
            return self
        def __exit__(self, *_):
            pass
        def read(self, timeout):
            clock.now += timeout
            return keys.pop(0) if keys else None
    class Live(Reader):
        def __init__(self, *args, **kwargs):
            pass
        def update(self, *args):
            pass
    def sleep(duration):
        clock.now += duration
    def twist():
        return SimpleNamespace(linear=SimpleNamespace(x=0., y=0.), angular=SimpleNamespace(z=0.))
    source = Path(__file__).parents[1] / 'mhseals_hardware/boat_test.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'BoatTest')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
               ('timed_phase', 'publish_command', 'manual_publish', 'assert_motion_ready')]
    stub = ast.ClassDef(name='Actions', bases=[], keywords=[], body=methods, decorator_list=[])
    namespace = dict(time=SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep),
                     KeyReader=Reader, Live=Live, Twist=twist)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[stub], type_ignores=[])),
                 str(source), 'exec'), namespace)
    action = namespace['Actions']()
    action.command_publisher = SimpleNamespace(publish=lambda message: sent.append(
        (message.linear.x, message.linear.y, message.angular.z)))
    action.console = None
    action.dashboard = lambda: None
    action.armed = True
    action.monitor_fault = ''
    action.processes = {'hardware': SimpleNamespace(poll=lambda: None)}
    action.check_command_topic = lambda: None
    action.specs = [('odometry', '/odom/local', None, True)]
    action.args = SimpleNamespace(allow_missing_sensors=False, require_tf=True)
    action.required_tf = lambda: ()
    action.monitor = SimpleNamespace(snapshot=lambda _: dict(
        sensors={'odometry': {'status': 'STALE' if fault and clock.now >= .05 else 'LIVE'}},
        tf=[{'status': 'LIVE'}]))
    return action, sent, twist


@pytest.mark.parametrize('fault,keys', [(False, [None, 'space']), (True, [None])])
def test_stop_or_sensor_loss_ends_with_neutral_and_no_background_writer(fault, keys):
    action, sent, _ = action_fixture(keys, fault)
    with pytest.raises(RuntimeError):
        action.timed_phase('surge', .25, 1)
    assert sent[0] == (.25, 0, 0)
    assert sent[-1] == (0, 0, 0)
    assert action.command == (0, 0, 0)


def test_successful_phase_always_finishes_neutral():
    action, sent, _ = action_fixture([])
    action.timed_phase('sway', -.25, .2)
    assert any(command == (0, -.25, 0) for command in sent)
    assert sent[-1] == (0, 0, 0)


def test_manual_interlock_blocks_motion_but_allows_final_neutral():
    action, sent, twist = action_fixture([])
    action.armed = False
    command = twist()
    command.linear.x = .25
    with pytest.raises(RuntimeError):
        action.manual_publish(command)
    assert not sent
    action.manual_publish(twist())
    assert sent == [(0, 0, 0)]


@pytest.mark.parametrize('optional_sensors', [False, True])
def test_measurement_startup_owns_one_shared_stack(optional_sensors):
    source = Path(__file__).parents[1] / 'mhseals_hardware/boat_test.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'BoatTest')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == 'start_measurement_stack')
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'),
         namespace)
    processes = []
    owner = SimpleNamespace(
        args=SimpleNamespace(fcu_url='serial:///dev/test:57600',
                             optional_sensors=optional_sensors),
        start_process=lambda name, command: processes.append((name, command)))
    namespace['start_measurement_stack'](owner)
    assert len(processes) == 1
    name, command = processes[0]
    assert name == 'odometry'  # Existing shutdown code owns this process group.
    assert command[:4] == ['ros2', 'launch', 'mhseals_nav', 'boat_test.launch.py']
    assert 'fcu_url:=serial:///dev/test:57600' in command
    assert f'start_sensors:={str(optional_sensors).lower()}' in command
