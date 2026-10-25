"""Operator YAML is parsed strictly: duplicate/merge/recursive input is refused, last good kept."""
from mhseals_hardware.configuration import (
    ConfigWatcher, load_config, startup_config, update_config,
)

import pytest

from test_mux_configuration import mux

BAD = {
    'duplicate': 'command_timeout: 0.3\ncommand_timeout: 0.4\n',
    'duplicate-nested': 'controller:\n  kp: [1, 1, 1]\n  kp: [2, 2, 2]\n',
    'duplicate-mux': 'command_mux:\n  cmd_timeout: 0.9\n  cmd_timeout: 0.8\n',
    'merge': 'command_mux:\n  <<: {cmd_timeout: 0.9}\n',
    'recursive': 'command_mux:\n  modes: &x [*x]\n',
    'alias': 'controller:\n  kp: &g [1, 1, 1]\n  ki: *g\n',
    'nan': 'command_timeout: .nan\n',
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
    with pytest.raises(ValueError, match=r'hardware\.yaml:2: duplicate key.*hardware\.yaml:1'):
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
    assert applied == []
    path.write_text('command_timeout: 0.2\n')
    tick()
    tick()
    assert [c['command_timeout'] for c in applied] == [0.2]
    path.write_text(BAD['duplicate'])
    tick()
    assert 'duplicate' in tick()
    assert [c['command_timeout'] for c in applied] == [0.2]  # nothing new applied


def build_mux(path):
    return mux({}, {'config_file': str(path)}, startup_config=startup_config)


@pytest.mark.parametrize('name', ['duplicate', 'duplicate-mux', 'merge', 'recursive'])
def test_mux_startup_refuses_bad_file(tmp_path, name):
    path = tmp_path / 'hardware.yaml'
    path.write_text(BAD[name])
    with pytest.raises(ValueError):
        build_mux(path)


def test_mux_startup_reads_the_operator_file(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_mux:\n  cmd_timeout: 0.9\n')
    assert build_mux(path).arbitration.cmd_timeout == 0.9


def test_unknown_key_keeps_wording_and_origin(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('no_such_setting: 1\n')
    wording = r'unknown hardware settings: \[.no_such_setting.\] at .*hardware\.yaml'
    with pytest.raises(ValueError, match=wording):
        load_config(path)
