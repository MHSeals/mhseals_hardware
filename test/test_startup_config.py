"""Node startup reads the arm snapshot once: defaults, the operator file, then ``-p`` parameters."""
from array import array
from pathlib import Path

import pytest
import yaml

from mhseals_hardware.configuration import declared, load_config, load_defaults, load_strict_yaml

pytestmark = pytest.mark.smoke


class Node:
    """rclpy's declare/get contract: a ``-p`` override wins over the declared default."""

    def __init__(self, **overrides):
        self.overrides, self.values = overrides, {}

    def declare_parameter(self, name, default):
        self.values[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return type('P', (), {'value': self.values[name]})


def test_a_p_override_wins_over_the_yaml(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_timeout: 0.4\nmanual_amplitude: 0.3\n')
    config = declared(Node(command_timeout=0.8), load_config(path))
    assert config['command_timeout'] == 0.8  # the override
    assert config['manual_amplitude'] == 0.3  # the file
    assert config['controller'] == load_defaults()['controller']  # mappings are not parameters


def test_a_section_declares_only_the_named_keys_and_reads_ros_arrays_as_lists(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('controller: {odom_timeout: 1}\n')
    node = Node(kp=array('d', [2., 3., 4.]), pwm_topic='/other')
    section = declared(node, load_config(path)['controller'], ['kp', 'odom_timeout'])
    assert section['kp'] == [2., 3., 4.] and type(section['kp']) is list
    assert section['odom_timeout'] == 1.0 and type(section['odom_timeout']) is float
    assert set(node.values) == {'kp', 'odom_timeout'}


def test_missing_operator_file_is_defaults(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    (tmp_path / 'empty.yaml').write_text('')
    assert load_config(None) == load_config(tmp_path / 'empty.yaml')


def test_explicit_missing_file_is_an_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_config(tmp_path / 'absent.yaml')


@pytest.mark.parametrize('text', [
    'controller: {odom_timeout: true}', 'controller: null', 'controller: {nope: 1}',
    'command_timeout: true', 'unknown: 1'])
def test_invalid_operator_values_refuse(tmp_path, text):
    path = tmp_path / 'hardware.yaml'
    path.write_text(text)
    with pytest.raises(ValueError):
        load_config(path)


@pytest.mark.parametrize('text', [
    'a: 1\na: 2', 'a: {<<: {b: 1}}', 'a: &x [1]\nb: *x', 'a: .inf', 'a: -.inf',
    '? [k]\n: v', '- 1', 'a: [\n'])
def test_strict_loader_refuses_and_locates(tmp_path, text):
    path = tmp_path / 'f.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match=r'f\.yaml'):
        load_strict_yaml(path)


def test_strict_loader_accepts_plain_and_empty(tmp_path):
    path = tmp_path / 'f.yaml'
    path.write_text('a: 1\nb: {c: [1.5, x]}\n')
    assert load_strict_yaml(path) == {'a': 1, 'b': {'c': [1.5, 'x']}}
    path.write_text('# nothing\n')
    assert load_strict_yaml(path) == {}


def test_schema_covers_real_keys_with_required_fields():
    schema = yaml.safe_load((Path(__file__).parents[1] / 'config/schema.yaml').read_text())
    defaults = load_defaults()
    for key, row in schema.items():
        top, _, leaf = key.partition('.')
        assert top in defaults and (not leaf or leaf in defaults[top]), key
        assert row['type'] in ('float', 'int', 'bool', 'str', 'list')
        assert row['tier'] in ('live', 'reconfigure', 'restart', 'new-session')
        assert isinstance(row['units'], str) and row['units']
        assert isinstance(row['doc'], str) and row['doc']
        value = defaults[top][leaf] if leaf else defaults[top]
        assert (value is None and row.get('nullable')) or type(value).__name__ == row['type'], key
        if 'range' in row:
            assert len(row['range']) == 2
    assert schema['command_timeout']['tier'] == 'restart'  # one value per arm for all three stages


def _constant(module, name):
    """A module-level tuple, read from source so the ROS nodes need no rclpy here."""
    import ast
    path = Path(__file__).parents[1] / 'mhseals_hardware' / f'{module}.py'
    return next(ast.literal_eval(node.value) for node in ast.parse(path.read_text()).body
                if isinstance(node, ast.Assign) and any(
                    getattr(t, 'id', None) == name for t in node.targets))


def test_schema_live_tier_matches_each_nodes_live_set():
    from mhseals_hardware.control_validation import GAIN_KEYS
    schema = yaml.safe_load((Path(__file__).parents[1] / 'config/schema.yaml').read_text())
    live = {'': _constant('thruster_pwm_node', 'PARAMETER_KEYS'),
            'command_mux': _constant('command_mux', 'LIVE_PARAMETERS'),
            'controller': GAIN_KEYS + ('feedback',)}
    for key, row in schema.items():
        section, _, leaf = key.rpartition('.')
        assert (row['tier'] == 'live') == (leaf in live.get(section, ())), key


def test_default_yaml_is_loaded_strictly(tmp_path, monkeypatch):
    import mhseals_hardware.configuration as configuration
    bad = tmp_path / 'default.yaml'
    bad.write_text('frequency: 50\nfrequency: 50\n')
    monkeypatch.setattr(configuration, 'config_resource', lambda name: bad)
    with pytest.raises(ValueError, match=r'default\.yaml:2.*duplicate'):
        configuration.load_defaults()


def test_unknown_key_error_names_line(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('frequency: 50\nbogus_key: 1\n')
    with pytest.raises(ValueError, match=r'unknown hardware settings.*hardware\.yaml:2'):
        load_config(path)
