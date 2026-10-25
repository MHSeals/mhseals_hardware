"""Node startup loads package defaults, then the operator file, then launch parameters."""
from array import array
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

from mhseals_hardware.configuration import load_defaults, load_strict_yaml, startup_config


def node(**overrides):
    return NS(_parameter_overrides={k: NS(value=v) for k, v in overrides.items()})


def test_layers_and_baseline(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_timeout: 0.4\ncontroller: {odom_timeout: 1}\n')
    effective, baseline = startup_config(node(command_timeout=0.8), path)
    assert baseline['command_timeout'] == 0.4
    assert effective['command_timeout'] == 0.8
    assert baseline['controller']['odom_timeout'] == 1.0
    assert type(baseline['controller']['odom_timeout']) is float
    assert effective['controller']['kp'] == load_defaults()['controller']['kp']


def test_missing_operator_file_is_defaults(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    effective, baseline = startup_config(node(), None)
    assert effective == baseline


def test_explicit_missing_file_is_an_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        startup_config(node(), tmp_path / 'absent.yaml')


def test_controller_mode_takes_only_controller_overrides(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('')
    effective, _ = startup_config(
        node(kp=array('d', [2., 3., 4.]), pwm_topic='/other'), path, hardware_parameters=False)
    assert effective['controller']['kp'] == [2., 3., 4.]
    assert effective['pwm_topic'] == load_defaults()['pwm_topic']  # driver keys are not overridden


@pytest.mark.parametrize('text', [
    'controller: {odom_timeout: true}', 'controller: null', 'controller: {nope: 1}',
    'command_timeout: true', 'unknown: 1'])
def test_invalid_operator_values_refuse(tmp_path, text):
    path = tmp_path / 'hardware.yaml'
    path.write_text(text)
    with pytest.raises(ValueError):
        startup_config(node(), path)


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
        assert row['units'] and row['doc']
        if 'range' in row:
            assert len(row['range']) == 2
    assert schema['command_timeout']['tier'] == 'live'
