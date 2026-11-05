import argparse
import os

import pytest
import yaml

from mhseals_hardware.configuration import (
    config_path, configure_args, load_config, load_defaults, save_config, update_config,
    validate_config,
)
from mhseals_hardware.keyboard import decode_key

pytestmark = pytest.mark.smoke


def test_persistent_mapping_and_pins(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    config = load_config()
    config['channel_map'] = [4, 3, 2, 1]
    config['reversed'] = [True, False, False, False]
    target = save_config(config)   # every key, board addresses included, as an arm snapshot does
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
    parser.add_argument('--reverse-thrust-ratio', type=float)
    args = configure_args(parser, ['--reverse-thrust-ratio', '0.8', '--save-config'])
    assert args.reverse_thrust_ratio == 0.8
    assert load_config()['reverse_thrust_ratio'] == 0.8
    assert yaml.safe_load(config_path().read_text()) == {'reverse_thrust_ratio': 0.8}  # only the flag


def test_a_board_address_is_set_only_in_the_defaults(tmp_path, monkeypatch):
    """The boot access service prepares default.yaml's PWM and MOSFET files, so no other layer moves them."""
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    path = tmp_path / 'hardware.yaml'
    path.write_text(f'mosfet_line: {load_defaults()["mosfet_line"] + 1}\n')
    with pytest.raises(ValueError, match='default.yaml'):
        load_config(path)
    chips = load_defaults()['pwm_chips']
    with pytest.raises(ValueError, match='pwm_chips'):
        update_config({'pwm_chips': chips[1:] + chips[:1]}, tmp_path / 'other.yaml')
    with pytest.raises(ValueError, match='mosfet_chip'):   # a snapshot writer refuses at save, not at arm
        save_config(dict(load_defaults(), mosfet_chip='other'), tmp_path / 'other.yaml')
    assert not (tmp_path / 'other.yaml').exists()
    # A bench tool may still open another line for one run, but --save-config never persists it.
    def bench():
        parser = argparse.ArgumentParser()
        parser.add_argument('--mosfet-line', type=int)
        return parser
    assert configure_args(bench(), ['--mosfet-line', '5']).mosfet_line == 5
    with pytest.raises(ValueError, match='mosfet_line'):
        configure_args(bench(), ['--mosfet-line', '5', '--save-config'])
    assert not config_path().exists()
    configure_args(bench(), ['--save-config'])   # no override: no empty operator file
    assert not config_path().exists()


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
    path.write_text('reverse_thrust_ratio: 0.8\n')
    update_config({'channel_map': [2, 1, 3, 4]}, path)
    assert yaml.safe_load(path.read_text()) == {'reverse_thrust_ratio': 0.8, 'channel_map': [2, 1, 3, 4]}
    assert load_config(path)['command_timeout'] > 0
    before = path.read_text()
    with pytest.raises(ValueError):
        update_config({'channel_map': [1]}, path)
    assert path.read_text() == before


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
    other = list(first)
    path.write_text(yaml.safe_dump({'decoupling': first, 'decoupling_negative': other}))
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


def test_command_mux_section_is_validated(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_mux: {mode_ch: 7}\n')
    assert load_config(path)['command_mux'] == dict(load_defaults()['command_mux'], mode_ch=7)
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


@pytest.mark.parametrize('value', [0, 0.0, 10, 20.0, 100])
def test_mosfet_heartbeat_is_off_or_fast_enough(value, tmp_path):
    update_config({'mosfet_heartbeat': value}, str(tmp_path / 'hardware.yaml'))
    assert load_config(str(tmp_path / 'hardware.yaml'))['mosfet_heartbeat'] == value


@pytest.mark.parametrize('value', [0.5, 3.3, 9.99, -1, 101, float('nan'), True, '20'])
def test_slow_or_invalid_mosfet_heartbeat_fails(value, tmp_path):
    with pytest.raises(ValueError, match='mosfet_heartbeat'):
        update_config({'mosfet_heartbeat': value}, str(tmp_path / 'hardware.yaml'))


def test_an_operator_file_with_the_old_heartbeat_key_loads_with_a_warning(tmp_path, caplog):
    path = tmp_path / 'hardware.yaml'
    path.write_text('mosfet_heartbeat_hz: 20\n')
    with caplog.at_level('WARNING', logger='mhseals_hardware.configuration'):
        assert load_config(str(path))['mosfet_heartbeat'] == 20
    assert 'mosfet_heartbeat_hz is deprecated' in caplog.text
    update_config({'frequency': load_config(str(path))['frequency']}, str(path))
    assert 'mosfet_heartbeat_hz' not in path.read_text()
    assert load_config(str(path))['mosfet_heartbeat'] == 20


def test_old_controller_topic_keys_still_arm_with_one_warning(tmp_path, monkeypatch):
    from mhseals_hardware import configuration
    warned = []
    monkeypatch.setattr(configuration.log, 'warning', lambda text, *args: warned.append(text % args))
    path = tmp_path / 'hardware.yaml'
    path.write_text('controller: {input_topic: /cmd_vel, kp: [1, 2, 3]}\n')
    config = load_config(str(path))
    assert 'input_topic' not in config['controller'] and config['controller']['kp'] == [1, 2, 3]
    assert len(warned) == 1 and 'controller.input_topic no longer exist' in warned[0]


def test_old_stage_timeouts_migrate_with_one_line_and_still_arm(tmp_path, monkeypatch):
    from mhseals_hardware import configuration
    warned = []
    monkeypatch.setattr(configuration.log, 'warning', lambda text, *args: warned.append(text % args))
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_timeout: 0.4\ncommand_mux: {cmd_timeout: 0.9, ramp: 2.0}\n'
                    'controller: {command_timeout: 1}\n')
    config = load_config(str(path))
    assert config['command_timeout'] == 0.4 and config['command_mux']['ramp'] == 2.0
    assert 'cmd_timeout' not in config['command_mux']
    assert 'command_timeout' not in config['controller']
    assert len(warned) == 1
    assert 'controller.command_timeout' in warned[0] and 'command_mux.cmd_timeout' in warned[0]
    update_config({'manual_amplitude': 0.5}, path)  # the next write drops them
    assert 'cmd_timeout' not in path.read_text() and path.read_text().count('command_timeout') == 1


def test_command_timeout_inside_command_mux_refuses(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('command_mux: {command_timeout: 0.9}\n')
    with pytest.raises(ValueError, match='top-level'):
        load_config(path)


@pytest.mark.parametrize('section', ['{auto: relative}', '{extra: /a}', '[1]'])
def test_a_bad_topics_section_is_refused(tmp_path, section):
    path = tmp_path / 'hardware.yaml'
    path.write_text('topics: ' + section + '\n')
    with pytest.raises(ValueError, match='topics'):
        load_config(path)


def test_an_operator_file_with_both_heartbeat_names_is_refused(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('mosfet_heartbeat_hz: 20\nmosfet_heartbeat: 30\n')
    with pytest.raises(ValueError, match='keep mosfet_heartbeat'):
        load_config(str(path))


def test_schema_documents_the_mosfet_heartbeat_floor():
    from pathlib import Path
    import yaml
    schema = yaml.safe_load((Path(__file__).parents[1] / 'config/schema.yaml').read_text())
    assert schema['mosfet_heartbeat']['range'] == [0, 100]
    assert '10-100' in schema['mosfet_heartbeat']['doc']


def test_controller_partial_merge(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('controller: {kp: [1, 2, 3]}\n')
    selected = load_config(path)
    assert selected['controller']['kp'] == [1., 2., 3.]
    assert selected['controller']['ki'] == load_defaults()['controller']['ki']


@pytest.mark.parametrize('section', ['null', '{unknown: 1}', '{kp: [1, 2]}',
    '{ki: [true, 0, 0]}', '{feedback: 1}', '{odom_timeout: 0}',
    '{odom_timeout: .inf}', '{odom_topic: relative}', '{obstacle_topic: /bad//topic}'])
def test_controller_invalid_section(tmp_path, section):
    path = tmp_path / 'hardware.yaml'
    path.write_text('controller: ' + section + '\n')
    with pytest.raises(ValueError):
        load_config(path)


@pytest.mark.parametrize('writer', [save_config, update_config])
def test_matrix_rows_persist_through_actual_save_paths(tmp_path, writer):
    import yaml

    target = tmp_path / 'hardware.yaml'
    config = load_defaults()
    matrices = {
        'thruster_matrix': [-1.2, .3, 2.0, .4, -1.5, -.6, 1.7, -.8, .9, 1.0, 1.1, -1.2],
        'decoupling': [1.1, -.2, .3, .4, 1.5, -.6, -.7, .8, 1.9],
        'decoupling_negative': [.9, .2, -.3, -.4, 1.2, .6, .7, -.8, 1.4],
    }
    config.update(matrices, manual_amplitude=.37)
    expected = config if writer is save_config else {**matrices, 'manual_amplitude': .37}
    writer(expected, target)
    text = target.read_text()
    assert yaml.safe_load(text) == expected
    assert load_config(target) == config
    for key, values in matrices.items():
        block = text.split(key + ': [\n', 1)[1].split('\n]', 1)[0].splitlines()
        assert len(block) == len(values) // 3
        assert all(len(line.strip().rstrip(',').split(',')) == 3 for line in block)
    assert not list(tmp_path.glob('.hardware-*'))


def test_saved_shared_lists_reload_without_aliases(tmp_path):
    from mhseals_hardware.configuration import load_config, update_config
    path = tmp_path / 'hardware.yaml'
    gains = [1.0, 2.0, 3.0]
    section = dict(load_defaults()['controller'], kp=gains, ki=gains)
    update_config({'controller': section}, path)
    text = path.read_text()
    assert '&' not in text and '*id' not in text
    assert load_config(path)['controller']['ki'] == gains


def test_one_topic_on_two_lanes_is_refused(tmp_path):
    """applied == effort would feed the mux its own output as the auto lane, so command_timeout never fires."""
    topics = dict(load_defaults()['topics'], applied='/cmd_vel_effort')
    path = tmp_path / 'hardware.yaml'
    path.write_text('topics: {' + ', '.join(f'{k}: {v}' for k, v in topics.items()) + '}\n')
    with pytest.raises(ValueError, match='topics.effort and topics.applied both name /cmd_vel_effort'):
        load_config(path)
