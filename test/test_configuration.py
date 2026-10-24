import argparse
import os

import pytest

from mhseals_hardware.configuration import (
    ConfigWatcher, configure_args, load_config, load_defaults, save_config, update_config, validate_config,
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


@pytest.mark.parametrize('text', ['', '\n', '# only a comment\n'])
def test_empty_or_comment_only_file_means_no_overrides(text, tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text(text)
    reference = tmp_path / 'reference.yaml'
    reference.write_text('{}\n')
    assert load_config(path) == load_config(reference)
    update_config({}, path)


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
    assert watch.poll() is None  # first sight of a change: wait for it to settle
    assert watch.poll()['command_timeout'] == 0.3
    assert watch.poll() is None
    edit('command_timeout: -1\n', 3)
    assert watch.poll() is None
    with pytest.raises(ValueError):
        watch.poll()
    assert watch.poll() is None  # a bad edit is reported once, not every tick


def test_watcher_ignores_a_half_written_file(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('channel_map: [3, 1, 2, 4]\nreversed: [true, false, false, true]\n')
    watch = ConfigWatcher(path)
    path.write_text('channel_map: [3, 1, 2, 4]\n')  # first chunk of an in-place rewrite
    assert watch.poll() is None
    path.write_text('channel_map: [3, 1, 2, 4]\nreversed: [true, false, false, false]\n')
    assert watch.poll() is None  # still changing between ticks
    config = watch.poll()
    assert config['channel_map'] == [3, 1, 2, 4]
    assert config['reversed'] == [True, False, False, False]


@pytest.mark.parametrize('text', [
    'reversed: [true, false\n',  # unterminated flow sequence: a half-saved file
    'channel_map: 5\n',
    'pwm_channels: 5\n',
    'characterization: {motion_limits: [\n',
])
def test_watcher_reports_yaml_and_type_errors_as_value_error(tmp_path, text):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_timeout: 0.4\n')
    os.utime(path, ns=(1, 1))
    watch = ConfigWatcher(path)
    path.write_text(text)
    os.utime(path, ns=(2, 2))
    assert watch.poll() is None
    with pytest.raises(ValueError):
        watch.poll()
    assert watch.poll() is None


def test_unset_negative_decoupling_follows_decoupling_and_is_not_frozen_on_save(tmp_path):
    import yaml
    path = tmp_path / 'hardware.yaml'
    path.write_text(yaml.safe_dump({'decoupling': [1, 0, 0, 0, 1, 0, .1, 0, 1]}))
    config = load_config(path)
    assert config['decoupling_negative'] == [1, 0, 0, 0, 1, 0, .1, 0, 1]
    save_config(config, path)
    assert yaml.safe_load(path.read_text())['decoupling_negative'] is None


def test_update_keeps_a_following_negative_following_and_a_real_split(tmp_path):
    import yaml
    path = tmp_path / 'hardware.yaml'
    first, second, split = ([1, 0, 0, 0, 1, 0, x, 0, 1] for x in (.1, .2, 0))
    path.write_text(yaml.safe_dump({'decoupling': first, 'decoupling_negative': first}))
    update_config({'decoupling': second}, path)
    assert 'decoupling_negative' not in yaml.safe_load(path.read_text())
    assert load_config(path)['decoupling_negative'] == second
    path.write_text(yaml.safe_dump({'decoupling': first, 'decoupling_negative': split}))
    update_config({'decoupling': second}, path)
    assert yaml.safe_load(path.read_text())['decoupling_negative'] == split


@pytest.mark.parametrize('curves', [
    {'heave+': {'d1': 1.0, 'd2': 0.0}}, {'surge+': {'d1': 1.0}}, {'surge+': {'d1': float('nan'), 'd2': 0.0}}])
def test_malformed_speed_curves_fail(curves):
    from mhseals_hardware.configuration import load_defaults, validate_config
    with pytest.raises(ValueError, match='speed_curves'):
        validate_config(dict(load_defaults(), characterization={'speed_curves': curves}))


@pytest.mark.parametrize('key,values', [
    ('decoupling', [1, 0, 0, 0, 1, 0, 0.05, 0, 1]),
    ('decoupling_negative', [1, 0, 0, 0, 1, 0, 0, 0, 1]),
    ('thruster_matrix', [-1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, -1]),
])
def test_matrices_are_normalized_to_floats(key, values, tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text(f'{key}: {values}\n')
    loaded = load_config(path)[key]
    assert loaded == [float(v) for v in values]
    assert all(type(v) is float for v in loaded)


@pytest.mark.parametrize('key', ['command_timeout', 'manual_amplitude', 'manual_yaw_scale'])
def test_integer_scalars_keep_the_float_default_type(key, tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text(f'{key}: 1\n')
    assert type(load_config(path)[key]) is float


def _update_many(path, key, value, count):
    from mhseals_hardware.configuration import update_config
    for _ in range(count):
        update_config({key: value}, path)


def test_concurrent_updates_do_not_lose_edits(tmp_path):
    import multiprocessing
    from mhseals_hardware.configuration import update_config
    path = tmp_path / 'hardware.yaml'
    update_config({'manual_amplitude': 0.1}, path)
    decoupling = [1.0, 0, 0, 0, 1.0, 0, 0.3, 0, 1.0]
    jobs = [multiprocessing.Process(target=_update_many, args=(path, 'decoupling', decoupling, 60)),
            multiprocessing.Process(target=_update_many, args=(path, 'manual_amplitude', 0.4, 60))]
    for job in jobs:
        job.start()
    for job in jobs:
        job.join()
    config = load_config(path)
    assert config['decoupling'][6] == 0.3 and config['manual_amplitude'] == 0.4


@pytest.mark.parametrize('text', ['reversed: [true, false', 'manual_amplitude: "x"\n'])
def test_watcher_turns_a_syntax_or_type_error_into_value_error(text, tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_timeout: 0.4\n')
    os.utime(path, ns=(1, 1))
    watch = ConfigWatcher(path)
    path.write_text(text)
    os.utime(path, ns=(2, 2))
    assert watch.poll() is None
    with pytest.raises(ValueError):
        watch.poll()


def test_command_mux_section_is_validated(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_mux: {mode_ch: 7}\n')
    assert load_config(path)['command_mux'] == {'mode_ch': 7}
    for text in ('command_mux: {bogus: 1}\n', 'command_mux: {modes: [0]}\n', 'command_mux: 3\n'):
        path.write_text(text)
        with pytest.raises(ValueError):
            load_config(path)


def test_hardware_epoch_defaults_to_zero_and_can_be_bumped(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('')
    assert load_config(str(path))['hardware_epoch'] == 0
    update_config({'hardware_epoch': 3}, str(path))
    assert load_config(str(path))['hardware_epoch'] == 3


@pytest.mark.parametrize('value', [-1, 1.5, '2', True, None])
def test_invalid_hardware_epoch_fails(value, tmp_path):
    with pytest.raises(ValueError, match='hardware_epoch'):
        update_config({'hardware_epoch': value}, str(tmp_path / 'hardware.yaml'))


def test_controller_partial_merge_and_independent_deadline(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_timeout: 0.4\ncontroller: {kp: [1, 2, 3], command_timeout: 1}\n')
    selected = load_config(path)
    assert selected['command_timeout'] == 0.4
    assert selected['controller']['command_timeout'] == 1.0
    assert type(selected['controller']['command_timeout']) is float
    assert selected['controller']['kp'] == [1., 2., 3.]
    assert selected['controller']['ki'] == load_defaults()['controller']['ki']


@pytest.mark.parametrize('section', ['null', '{unknown: 1}', '{kp: [1, 2]}',
    '{ki: [true, 0, 0]}', '{feedback: 1}', '{odom_timeout: 0}',
    '{command_timeout: .inf}', '{input_topic: relative}', '{output_topic: /bad//topic}'])
def test_controller_invalid_section(tmp_path, section):
    path = tmp_path / 'hardware.yaml'
    path.write_text('controller: ' + section + '\n')
    with pytest.raises(ValueError):
        load_config(path)
