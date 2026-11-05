"""load_config's layers: defaults < the vessel profile's thrusters: < the operator file."""

import pytest

pytestmark = pytest.mark.smoke

from mhseals_hardware import vessel
from mhseals_hardware.configuration import channel_map_source, load_config, update_config


def empty(tmp_path):
    """An operator file with no overrides."""
    (tmp_path / 'empty.yaml').write_text('')
    return tmp_path / 'empty.yaml'


@pytest.fixture
def boat(tmp_path, monkeypatch):
    """A fleet profile and a vessel extending it; ``write(thrusters)`` sets the vessel's thrusters: text."""
    (tmp_path / 'roboboat').mkdir()
    (tmp_path / 'roboboat' / 'profile.yaml').write_text('hull: {shape: monohull}\n')
    (tmp_path / 'boat').mkdir()
    (tmp_path / 'boat' / 'network.env').write_text('')
    monkeypatch.setenv('ASTRO_NETWORK_ENV', str(tmp_path / 'boat' / 'network.env'))

    def write(thrusters, before=''):
        (tmp_path / 'boat' / 'profile.yaml').write_text(f'extends: roboboat\n{before}thrusters: {thrusters}\n')
    write('{channel_map: [1, 3, 2, 4]}')
    return write


def test_the_vessel_profile_sets_the_channel_map(boat, tmp_path):
    assert load_config(empty(tmp_path))['channel_map'] == [1, 3, 2, 4]
    # The driver's ready line names the layer that won.
    assert channel_map_source(empty(tmp_path)) == f'vessel {(tmp_path / "boat").resolve()}'


def test_the_operator_file_wins_over_the_vessel(boat, tmp_path):
    operator = tmp_path / 'hardware.yaml'
    operator.write_text('channel_map: [2, 1, 3, 4]\n')
    assert load_config(operator)['channel_map'] == [2, 1, 3, 4]
    assert channel_map_source(operator) == str(operator)   # not the profile underneath


def test_without_a_vessel_env_the_defaults_stand(tmp_path):
    assert load_config(empty(tmp_path))['channel_map'] == [1, 2, 3, 4]
    assert channel_map_source(empty(tmp_path)) == 'defaults'


def test_a_typo_in_the_vessel_thrusters_fails(boat, tmp_path):
    boat('{chanel_map: [1, 3, 2, 4]}')
    with pytest.raises(ValueError, match='chanel_map'):
        load_config(empty(tmp_path))


def test_a_vessel_channel_map_that_is_no_permutation_fails(boat, tmp_path):
    boat('{channel_map: [1, 1, 2, 4]}')
    with pytest.raises(ValueError):
        load_config(empty(tmp_path))


def test_a_vessel_key_outside_the_wiring_fails(boat, tmp_path):
    boat('{estop_cuts_power: false}')
    with pytest.raises(ValueError, match='estop_cuts_power'):
        load_config(empty(tmp_path))


@pytest.mark.parametrize('before', ['lidar: {model: VLP16\n', 'thrusters: {reversed: [true, false, false, false]}\n'])
def test_an_unreadable_vessel_profile_refuses_instead_of_the_defaults(boat, tmp_path, before):
    """A syntax error elsewhere in the profile, or a duplicate thrusters:, never brings back [1, 2, 3, 4]."""
    boat('{channel_map: [1, 3, 2, 4]}', before)
    with pytest.raises(ValueError):
        load_config(empty(tmp_path))


def test_a_missing_vessel_file_refuses(boat, tmp_path, monkeypatch):
    (tmp_path / 'roboboat' / 'profile.yaml').unlink()
    with pytest.raises(ValueError, match='missing'):
        load_config(empty(tmp_path))
    monkeypatch.setenv('ASTRO_NETWORK_ENV', str(tmp_path / 'nowhere.env'))
    with pytest.raises(ValueError, match='missing'):
        load_config(empty(tmp_path))


def test_a_workspace_without_a_vessel_env_has_no_vessel(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    assert load_config(empty(tmp_path))['channel_map'] == [1, 2, 3, 4]


def test_a_symlinked_env_reads_the_vessel_it_points_at(boat, tmp_path, monkeypatch):
    (tmp_path / 'ws' / 'config').mkdir(parents=True)
    (tmp_path / 'ws' / 'config' / 'boat-network.env').symlink_to(tmp_path / 'boat' / 'network.env')
    monkeypatch.delenv('ASTRO_NETWORK_ENV')
    monkeypatch.setenv('ROS_WS', str(tmp_path / 'ws'))
    assert load_config(empty(tmp_path))['channel_map'] == [1, 3, 2, 4]
    assert vessel.directory() == (tmp_path / 'boat').resolve()   # the driver's ready line names the profile


def test_update_config_validates_over_the_vessel_layer(boat, tmp_path):
    update_config({'reversed': [True, False, False, False]}, tmp_path / 'hardware.yaml')
    assert load_config(tmp_path / 'hardware.yaml')['channel_map'] == [1, 3, 2, 4]


def test_a_vessel_profile_edit_is_seen_by_the_next_load(boat, tmp_path):
    """Bench check step 6 fixes the profile's value; a long-lived process (TUI, agent) sees it on its next load."""
    assert load_config(empty(tmp_path))['channel_map'] == [1, 3, 2, 4]
    boat('{channel_map: [4, 3, 2, 1]}')
    assert load_config(empty(tmp_path))['channel_map'] == [4, 3, 2, 1]
