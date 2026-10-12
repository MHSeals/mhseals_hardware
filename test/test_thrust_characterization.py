import json
import pytest

from pytest import approx

from mhseals_hardware.thrust_characterization import (
    characterize_samples, nav2_profile, read_json_samples)


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


def test_nav2_profile_applies_measured_rotation_limits(tmp_path):
    source = tmp_path / 'nav2.yaml'
    source.write_text('''
controller_server:
  ros__parameters:
    FollowPath:
      desired_linear_vel: 1.0
      max_angular_accel: 1.0
      rotate_to_heading_angular_vel: 1.0
velocity_smoother:
  ros__parameters:
    max_velocity: [1.0, 1.0, 1.0]
    min_velocity: [-1.0, -1.0, -1.0]
    max_accel: [1.0, 1.0, 1.0]
    max_decel: [-1.0, -1.0, -1.0]
behavior_server:
  ros__parameters:
    max_rotational_vel: 1.0
    rotational_acc_lim: 1.0
''')
    result = characterize_samples(samples())
    profile = nav2_profile(result, source)
    follow = profile['controller_server']['ros__parameters']['FollowPath']
    behavior = profile['behavior_server']['ros__parameters']
    assert follow['rotate_to_heading_angular_vel'] == approx(.3)
    assert follow['max_angular_accel'] == approx(.3)
    assert behavior['max_rotational_vel'] == approx(.3)
    assert behavior['rotational_acc_lim'] == approx(.3)
