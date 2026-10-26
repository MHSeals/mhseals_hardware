import pytest

from ros_isolation import isolate_ros_env


@pytest.mark.parametrize('raw', [None, '', '0', '42', '233', 'abc'])
def test_an_unusable_domain_is_replaced_by_a_private_one(raw):
    environ = {} if raw is None else {'ROS_DOMAIN_ID': raw}
    domain = isolate_ros_env(environ)
    assert 1 <= domain <= 232 and domain != 42
    assert environ['ROS_DOMAIN_ID'] == str(domain)


def test_the_domain_a_runner_picked_is_kept():
    assert isolate_ros_env({'ROS_DOMAIN_ID': '137'}) == 137


def test_static_peers_are_dropped_and_discovery_stays_on_this_host():
    environ = {'ROS_STATIC_PEERS': '192.168.0.10', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'}
    isolate_ros_env(environ)
    assert 'ROS_STATIC_PEERS' not in environ
    assert environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
