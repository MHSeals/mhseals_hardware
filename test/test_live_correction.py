"""The driver's correction C changes live through SetParameters: validated, applied, read back."""
import math
from types import SimpleNamespace as NS

import pytest

from test_pwm_reload_guard import pwm

pytestmark = pytest.mark.smoke

IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
CORRECTION = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.1, 0.0, 1.0]


def parameter(name, value):
    return NS(name=name, value=value, type_='DOUBLE_ARRAY')


def live_node():
    node, env = pwm()
    env['Parameter'] = lambda name, type_, value: NS(name=name, type_=type_, value=value)
    return node


def set_live(node, *parameters):
    """What rclpy does: pre-set, validate, store, post-set."""
    parameters = node.follow_correction(list(parameters))
    result = node.check_parameters(parameters)
    if result.successful:
        node.commit_parameters(parameters)
    return result, parameters


def test_a_valid_correction_applies_and_is_published():
    node = live_node()
    result, sent = set_live(node, parameter('decoupling', CORRECTION))
    assert result.successful
    assert [p.name for p in sent] == ['decoupling', 'decoupling_negative']  # C- follows C
    assert node.decoupling == ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-0.1, 0.0, 1.0))
    assert node.decoupling_negative == node.decoupling
    assert node.config['decoupling'] == CORRECTION
    assert node.statuses[-1]['live']['decoupling'] == CORRECTION
    assert any('applied live' in line for line in node.events)


def test_a_split_negative_stays_put_when_only_c_changes():
    node = live_node()
    split = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.2, 0.0, 1.0]
    assert set_live(node, parameter('decoupling_negative', split))[0].successful
    result, sent = set_live(node, parameter('decoupling', CORRECTION))
    assert result.successful and [p.name for p in sent] == ['decoupling']
    assert node.config['decoupling_negative'] == split


@pytest.mark.parametrize('bad', [
    IDENTITY[:8], IDENTITY + [0.0], [math.nan] + IDENTITY[1:], [math.inf] + IDENTITY[1:],
    [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 2.5, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0],
    [0.0] + IDENTITY[1:]])
def test_a_bad_correction_is_refused_and_the_old_one_kept(bad):
    node = live_node()
    before = node.decoupling, dict(node.config)
    result, _ = set_live(node, parameter('decoupling', bad))
    assert not result.successful and 'decoupling' in result.reason
    assert (node.decoupling, node.config) == before and not node.statuses


def test_restart_keys_stay_refused_in_a_batch_with_c():
    node = live_node()
    result, _ = set_live(node, parameter('decoupling', CORRECTION), NS(name='channel_map', value=[2, 1, 3, 4]))
    assert not result.successful and 'restart required' in result.reason
    assert node.config['decoupling'] == IDENTITY


def test_own_mirroring_writes_skip_the_live_path():
    node = live_node()
    node.own_write = True
    sent = node.follow_correction([parameter('decoupling', CORRECTION)])
    assert len(sent) == 1 and node.check_parameters(sent).successful
    node.commit_parameters(sent)
    assert node.config['decoupling'] == IDENTITY
