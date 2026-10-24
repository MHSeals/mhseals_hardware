"""Pure compiler probes do not initialize a ROS node or an output driver."""
from mhseals_hardware.startup_config import compile_inputs, evaluate, observe
import pytest


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
    operator.write_text("""command_timeout: 0.7
thruster_matrix: [0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5]
characterization:
  velocity_gain:
    surge: {surge: 0.5, sway: 0.0, yaw: 0.0}
    sway: {surge: 0.0, sway: 0.5, yaw: 0.0}
    yaw: {surge: 0.0, sway: 0.0, yaw: 0.5}
  motion_limits:
    velocity: {surge: 0.5, sway: 0.5, yaw: 0.5}
""")
    document = compile_inputs(operator)
    evidence = observe(document)
    assert evidence['mixer'][0] == [0.125, 0.0, 0.0, 0.125]
    assert evidence['law'] == pytest.approx([0.28, 0.0, 0.0])
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
            self.parameters = {'config_root': str(root), 'config_scope': 'same'}

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
    namespace = dict(Node=Node, node_startup_inputs=node_startup_inputs,  # noqa: C408
                     make_outputs=lambda *args, **kwargs: calls.append('outputs'))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                 str(source), 'exec'), namespace)
    with pytest.raises((ValueError, FileNotFoundError)):
        namespace['ThrusterPWMNode']()
    assert calls == []


@pytest.mark.parametrize('spelling', ['1', '1.0'])
def test_float_setting_accepts_numeric_yaml_styles(tmp_path, spelling):
    operator = tmp_path / 'hardware.yaml'
    operator.write_text('command_timeout: ' + spelling + '\n')
    document = compile_inputs(operator)
    assert document['effective']['command_timeout'] == 1.0
    assert type(document['effective']['command_timeout']) is float
    operator.write_text('command_timeout: true\n')
    with pytest.raises(ValueError):
        compile_inputs(operator)


def test_rounding_tolerance_preserves_evidence_shape_and_finiteness(tmp_path):
    from mhseals_hardware.startup_config import plain, startup_inputs
    operator = tmp_path / 'hardware.yaml'
    operator.write_text("""characterization:
  velocity_gain:
    surge: {surge: 2.7, sway: 0.0, yaw: 0.0}
    sway: {surge: 0.0, sway: 2.7, yaw: 0.0}
    yaw: {surge: 0.0, sway: 0.0, yaw: 2.7}
  motion_limits:
    velocity: {surge: 2.7, sway: 2.7, yaw: 2.7}
""")
    document, _ = startup_inputs(operator, root=tmp_path / 'snapshots', scope='rounding')
    evidence = observe(document)
    assert evaluate(document, evidence)
    for bad in (float('nan'), float('inf'), True, 0.1):
        corrupted = plain(evidence)
        corrupted['law'][0] = bad
        assert not evaluate(document, corrupted)
    assert not evaluate(document, dict(evidence, identity='another'))
    assert not evaluate(document, dict(evidence, law=evidence['law'][:2]))


def test_controller_leaf_precedence_and_locations(tmp_path):
    profile = tmp_path / 'profile.yaml'
    profile.write_text('controller: {kp: [1, 2, 3], feedback: false}\n')
    operator = tmp_path / 'operator.yaml'
    operator.write_text('controller:\n  odom_timeout: 1\n')
    doc = compile_inputs(operator, profile=profile,
        session=('controller: {kp: [2, 3, 4]}', 'ROS invocation'))
    assert doc['baseline']['controller']['kp'] == [1., 2., 3.]
    assert doc['effective']['controller']['kp'] == [2., 3., 4.]
    assert doc['effective']['controller']['feedback'] is False
    assert doc['effective']['controller']['odom_timeout'] == 1.0
    assert doc['origins']['controller.odom_timeout'][-1]['line'] == 2
    assert [o['layer'] for o in doc['origins']['controller.kp']] == [
        'defaults', 'profile', 'session']
    operator.write_text('{}')
    assert compile_inputs(operator)['effective']['controller']['odom_timeout'] == .3


@pytest.mark.parametrize('text', ['controller: {kp: [1, 2, 3], kp: [2, 3, 4]}',
    'controller: {<<: {feedback: false}}', 'controller: &self {kp: *self}',
    'controller: {odom_timeout: true}', 'controller: null'])
def test_controller_strict_sources(tmp_path, text):
    path = tmp_path / 'operator.yaml'
    path.write_text(text)
    with pytest.raises(ValueError):
        compile_inputs(path)


def test_controller_snapshot_retains_complete_baseline_and_session(tmp_path):
    from mhseals_hardware.startup_config import startup_inputs
    path = tmp_path / 'hardware.yaml'
    path.write_text('controller: {kp: [2, 3, 4], input_topic: /custom}\n')
    first, selected = startup_inputs(path, root=tmp_path / 'snapshots', scope='same',
        session=('controller: {feedback: false}', 'invocation'))
    path.unlink()
    again, _ = startup_inputs(path, root=tmp_path / 'snapshots', scope='same')
    assert first == again
    baseline = selected.documents['hardware-baseline']['effective']['controller']
    assert baseline['feedback'] is True
    assert again['effective']['controller']['feedback'] is False
    assert again['effective']['controller']['input_topic'] == '/custom'
