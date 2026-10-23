import argparse
import os

import pytest

from mhseals_hardware.configuration import (
    ConfigWatcher, configure_args, load_config, save_config, update_config, validate_config,
)
from mhseals_hardware.keyboard import decode_key


def test_persistent_mapping_and_pins(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    config = load_config()
    config['channel_map'] = [4, 3, 2, 1]
    config['mosfet_line'] = 12
    target = save_config(config)
    assert target.exists()
    assert load_config() == config


def test_explicit_missing_config_fails(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_config(tmp_path / 'missing.yaml')


def test_unknown_keys_fail(tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text('mosfet_lien: 3\n')
    with pytest.raises(ValueError, match='unknown'):
        load_config(config)


@pytest.mark.parametrize('value', [float('nan'), -1, 0, 100])
def test_invalid_watchdog_fails(value, tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    config = load_config()
    config['command_timeout'] = value
    with pytest.raises(ValueError):
        validate_config(config)


def test_cli_overrides_then_saves(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    parser = argparse.ArgumentParser()
    parser.add_argument('--mosfet-line', type=int)
    args = configure_args(parser, ['--mosfet-line', '5', '--save-config'])
    assert args.mosfet_line == 5
    assert load_config()['mosfet_line'] == 5


@pytest.mark.parametrize(('sequence', 'key'), [
    (b'\x1b[119;1:1u', 'w'),
    (b'\x1b[119;1:2u', 'w'),
    (b'\x1b[119;1:3u', 'release:w'),
    (b'\x1b[1;1:3A', 'release:up'),
    (b'\x1b[A', 'up'),
    (b' ', 'space'),
])
def test_terminal_events(sequence, key):
    assert decode_key(sequence) == key


def test_empty_config_variable_means_default(monkeypatch, tmp_path):
    from mhseals_hardware.configuration import config_path
    monkeypatch.setenv('MHSEALS_HARDWARE_CONFIG', '')
    monkeypatch.setenv('HOME', str(tmp_path))
    assert config_path() == tmp_path / '.config/mhseals/hardware.yaml'


def test_update_only_persists_explicit_settings(tmp_path):
    import yaml
    from mhseals_hardware.configuration import update_config
    path = tmp_path / 'hardware.yaml'
    path.write_text('mosfet_line: 12\n')
    update_config({'channel_map': [2, 1, 3, 4]}, path)
    assert yaml.safe_load(path.read_text()) == {'mosfet_line': 12, 'channel_map': [2, 1, 3, 4]}
    assert load_config(path)['command_timeout'] > 0
    before = path.read_text()
    with pytest.raises(ValueError):
        update_config({'channel_map': [1]}, path)
    assert path.read_text() == before


def test_watcher_reports_an_edit_once_and_rejects_a_bad_one(tmp_path):
    path = tmp_path / 'hardware.yaml'

    def edit(text, tick):
        path.write_text(text)
        os.utime(path, ns=(tick, tick))  # a coarse filesystem clock could repeat an mtime

    edit('command_timeout: 0.4\n', 1)
    watch = ConfigWatcher(path)
    assert watch.poll() is None
    edit('command_timeout: 0.3\n', 2)
    assert watch.poll()['command_timeout'] == 0.3
    assert watch.poll() is None
    edit('command_timeout: -1\n', 3)
    with pytest.raises(ValueError):
        watch.poll()
    assert watch.poll() is None  # a bad edit is reported once, not every tick
