import json
import pytest

from pytest import approx

from mhseals_hardware.thrust_characterization import characterize_samples, read_json_samples


def samples():
    rows = []
    commands = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                (0, 0, 1), (0, 0, -1), (1, 1, 0), (-1, 1, -1))
    for stamp, command in enumerate(commands):
        rows.append({'time': stamp, 'command': command,
                     'velocity': (0.6 * command[0], 0.4 * command[1],
                                  0.3 * command[2])})
    return rows


def test_fits_gain_and_limits():
    result = characterize_samples(samples())
    assert result['sample_count'] == 8
    assert result['velocity_gain']['surge']['surge'] == approx(0.6)
    assert result['motion_limits']['velocity']['sway'] == approx(0.4)
    assert len(result['thruster_matrix']) == 12
    assert result['motion_limits']['combined_directions']


def test_json_samples_accepts_sample_wrapper(tmp_path):
    path = tmp_path / 'samples.json'
    path.write_text(json.dumps({'samples': samples()}))
    assert len(read_json_samples(path)) == 8


def test_underexcited_bag_is_rejected():
    with pytest.raises(ValueError, match='excite'):
        characterize_samples([
            {'time': i, 'command': (1, 0, 0), 'velocity': (i * .1, 0, 0)}
            for i in range(8)])
