"""Pure compiler probes do not initialize a ROS node or an output driver."""
import pytest

from mhseals_hardware.startup_config import compile_inputs, evaluate, observe


def test_baseline_session_and_origins(tmp_path):
    profile = tmp_path / 'profile.yaml'
    profile.write_text('command_timeout: 0.4\n')
    operator = tmp_path / 'hardware.yaml'
    operator.write_text('command_timeout: 0.5\n')
    document = compile_inputs(operator, profile=profile,
                              session=('command_timeout: 0.8\n', 'invocation'))
    assert document['baseline']['command_timeout'] == 0.5
    assert document['effective']['command_timeout'] == 0.8
    assert [o['layer'] for o in document['origins']['command_timeout']] == [
        'defaults', 'profile', 'operator', 'session']
    assert document['origins']['command_timeout'][-1]['source'] == 'invocation'
    assert compile_inputs(operator)['effective']['command_timeout'] == 0.5
    assert operator.read_text() == 'command_timeout: 0.5\n'


def test_nondefault_actual_mixer_and_law(tmp_path):
    operator = tmp_path / 'hardware.yaml'
    operator.write_text('''command_timeout: 0.7
thruster_matrix: [0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5]
characterization:
  velocity_gain:
    surge: {surge: 0.5, sway: 0.0, yaw: 0.0}
    sway: {surge: 0.0, sway: 0.5, yaw: 0.0}
    yaw: {surge: 0.0, sway: 0.0, yaw: 0.5}
  motion_limits:
    velocity: {surge: 0.5, sway: 0.5, yaw: 0.5}
''')
    document = compile_inputs(operator)
    evidence = observe(document)
    assert evidence['mixer'][0] == [0.125, 0.0, 0.0, 0.125]
    assert evidence['law'] == pytest.approx([0.2, 0.0, 0.0])
    assert evidence['timeout'] == 0.7
    assert evaluate(document, evidence)
    evidence['mixer'][0][0] = 0.0
    assert not evaluate(document, evidence)
    assert not evaluate(document, {'verified': True})


@pytest.mark.parametrize('text', ['command_timeout: 0.5\ncommand_timeout: 0.6\n',
                                  'command_timeout: .nan\n', 'unknown: 1\n'])
def test_rejects_invalid_sources(tmp_path, text):
    operator = tmp_path / 'hardware.yaml'
    operator.write_text(text)
    with pytest.raises(ValueError):
        compile_inputs(operator)


def test_bootstrap_same_scope_ignores_authoring_edits(tmp_path):
    from mhseals_hardware.startup_config import startup_inputs
    operator = tmp_path / 'hardware.yaml'
    operator.write_text('command_timeout: 0.5\n')
    root = tmp_path / 'snapshots'
    first, selected = startup_inputs(operator, root=root, scope='same',
                                     session=('command_timeout: 0.8\n', 'invocation'))
    operator.write_text('command_timeout: 0.6\n')
    restarted, again = startup_inputs(operator, root=root, scope='same')
    fresh, _ = startup_inputs(operator, root=root, scope='fresh')
    assert first == restarted
    assert selected.ticket == again.ticket
    assert fresh['effective']['command_timeout'] == 0.6
    assert restarted['baseline']['command_timeout'] == 0.5
    assert restarted['effective']['command_timeout'] == 0.8
    assert selected.manifest.baseline.artifacts[0].name == 'hardware-baseline.json'


@pytest.mark.parametrize('fault', ['missing', 'corrupt'])
def test_required_artifact_fault_prevents_actual_pwm_outputs(tmp_path, fault):
    import ast
    from pathlib import Path
    from types import SimpleNamespace
    from mhseals_hardware.startup_config import node_startup_inputs, startup_inputs

    root = tmp_path / 'snapshots'
    startup_inputs(root=root, scope='same')
    artifact = root / 'same/initializing/hardware-pure.json'
    if fault == 'missing':
        artifact.unlink()
    else:
        artifact.write_text('{}')
    calls = []

    class Node:
        def __init__(self, name):
            self._parameter_overrides = {}
            self.parameters = dict(config_root=str(root), config_scope='same')

        def declare_parameter(self, name, value):
            self.parameters.setdefault(name, value)

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

    source = Path(__file__).parents[1] / 'mhseals_hardware/thruster_pwm_node.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'ThrusterPWMNode')
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name == '__init__']
    namespace = dict(Node=Node, node_startup_inputs=node_startup_inputs,
                     make_outputs=lambda *args, **kwargs: calls.append('outputs'))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                 str(source), 'exec'), namespace)
    with pytest.raises((ValueError, FileNotFoundError)):
        namespace['ThrusterPWMNode']()
    assert calls == []
