"""Operator YAML is parsed strictly: duplicate/merge/recursive input is refused, last good kept."""
from pathlib import Path

import pytest

from mhseals_hardware.configuration import (
    ConfigWatcher, load_config, update_config,
)
from mhseals_hardware.startup_config import node_startup_inputs

from test_mux_configuration import mux

BAD = {
    'duplicate': 'command_timeout: 0.3\ncommand_timeout: 0.4\n',
    'duplicate-nested': 'controller:\n  kp: [1, 1, 1]\n  kp: [2, 2, 2]\n',
    'merge': 'base: &b {a: 1}\ncommand_mux:\n  <<: *b\n',
    'recursive': 'command_mux: &x {mode_ch: *x}\n',
    'unknown': 'mosfet_lien: 3\n',
    'syntax': 'command_timeout: [\n',
}


@pytest.mark.parametrize('name', BAD)
def test_load_config_refuses_with_value_error(tmp_path, name):
    path = tmp_path / 'hardware.yaml'
    path.write_text(BAD[name])
    with pytest.raises(ValueError):
        load_config(path)
    with pytest.raises(ValueError):
        update_config({'command_timeout': 0.2}, path)
    assert path.read_text() == BAD[name]  # a refused file is never rewritten


def test_duplicate_error_is_located(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text(BAD['duplicate'])
    with pytest.raises(ValueError, match=r'hardware\.yaml.*line=1.*hardware\.yaml.*line=2'):
        load_config(path)


def test_valid_file_still_loads_and_empty_is_no_overrides(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_timeout: 1\ncontroller:\n  kp: [1, 1, 1]\n')
    config = load_config(path)
    assert config['command_timeout'] == 1.0 and type(config['command_timeout']) is float
    assert config['controller']['kp'] == [1, 1, 1]
    path.write_text('# nothing\n')
    assert load_config(path) == load_defaults_merged()


def load_defaults_merged():
    from mhseals_hardware.configuration import _merge_overrides
    return _merge_overrides({})


def settle(watch):
    assert watch.poll() is None  # first sight of a change
    return watch.poll


def test_watcher_keeps_last_good_and_never_applies_duplicate(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_timeout: 0.3\n')
    watch = ConfigWatcher(path)
    path.write_text('command_timeout: 0.4\n')
    settle(watch)
    good = watch.poll
    assert watch.last_good is None
    applied = []

    def tick():  # the callback shape shared by driver, controller and mux
        try:
            config = watch.poll()
        except ValueError as error:
            return str(error)
        if config is not None:
            applied.append(config)

    path.write_text('command_timeout: 0.5\n')
    path.write_text('command_timeout: 0.6\ncommand_timeout: 0.7\n')
    assert tick() is None  # settling
    assert 'duplicate' in tick()
    assert applied == [] and watch.last_good is None
    path.write_text('command_timeout: 0.2\n')
    tick()
    tick()
    assert [c['command_timeout'] for c in applied] == [0.2]
    assert watch.last_good is applied[-1]
    path.write_text(BAD['duplicate'])
    tick()
    assert 'duplicate' in tick()
    assert watch.last_good['command_timeout'] == 0.2 and len(applied) == 1


def real_startup(monkeypatch, tmp_path):
    monkeypatch.setenv('MHSEALS_CONFIG_ROOT', str(tmp_path / 'snap'))
    monkeypatch.delenv('MHSEALS_CONFIG_SCOPE', raising=False)
    return lambda section, path: mux(section, {'config_file': str(path)},
                                     node_startup_inputs=node_startup_inputs)


def test_mux_startup_is_managed_and_records_snapshot(tmp_path, monkeypatch):
    build = real_startup(monkeypatch, tmp_path)
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_mux:\n  cmd_timeout: 0.9\n')
    node = build({}, path)
    assert node.arbitration.cmd_timeout == 0.9
    assert node.startup_selection is not None
    assert list((tmp_path / 'snap').rglob('active.json'))  # the recorded snapshot


@pytest.mark.parametrize('name', ['duplicate', 'merge', 'recursive'])
def test_mux_startup_refuses_bad_file_without_snapshot(tmp_path, monkeypatch, name):
    build = real_startup(monkeypatch, tmp_path)
    path = tmp_path / 'hardware.yaml'
    path.write_text(BAD[name])
    with pytest.raises(ValueError):
        build({}, path)
    assert not list((tmp_path / 'snap').rglob('active.json'))
