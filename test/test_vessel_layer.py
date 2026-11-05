"""load_config's layers: defaults < the vessel profile's thrusters: < the operator file."""

import pytest

from mhseals_hardware.configuration import load_config


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

    def write(thrusters):
        (tmp_path / 'boat' / 'profile.yaml').write_text(f'extends: roboboat\nthrusters: {thrusters}\n')
    write('{channel_map: [1, 3, 2, 4]}')
    return write


def test_the_vessel_profile_sets_the_channel_map(boat, tmp_path):
    assert load_config(empty(tmp_path))['channel_map'] == [1, 3, 2, 4]


def test_the_operator_file_wins_over_the_vessel(boat, tmp_path):
    operator = tmp_path / 'hardware.yaml'
    operator.write_text('channel_map: [2, 1, 3, 4]\n')
    assert load_config(operator)['channel_map'] == [2, 1, 3, 4]


def test_without_a_vessel_env_the_defaults_stand(tmp_path):
    assert load_config(empty(tmp_path))['channel_map'] == [1, 2, 3, 4]


def test_a_typo_in_the_vessel_thrusters_fails(boat, tmp_path):
    boat('{chanel_map: [1, 3, 2, 4]}')
    with pytest.raises(ValueError, match='chanel_map'):
        load_config(empty(tmp_path))


def test_a_vessel_channel_map_that_is_no_permutation_fails(boat, tmp_path):
    boat('{channel_map: [1, 1, 2, 4]}')
    with pytest.raises(ValueError):
        load_config(empty(tmp_path))
