import argparse

import pytest

from mhseals_hardware.configuration import (
    configure_args, load_config, save_config, validate_config,
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
