"""Keep ROS tests off the boat network: a private DDS domain, no static peers, localhost discovery."""

import os

BOAT_DOMAIN = 42
MAX_DOMAIN = 232
FIRST_PRIVATE = 110


def isolate_ros_env(environ=None):
    """Make environ safe for tests that start nodes, and return the domain id.

    An existing ROS_DOMAIN_ID (for example .handoff/tools/ctest's) is kept unless it is unset, not a
    number in 1..232, or the boat's 42; otherwise one derived from the pid (110..209) is set. Static
    peers are removed and discovery is limited to this host.
    """
    environ = os.environ if environ is None else environ
    raw = environ.get('ROS_DOMAIN_ID', '')
    if not (raw.isdigit() and 1 <= int(raw) <= MAX_DOMAIN and int(raw) != BOAT_DOMAIN):
        environ['ROS_DOMAIN_ID'] = str(FIRST_PRIVATE + os.getpid() % 100)
    environ.pop('ROS_STATIC_PEERS', None)
    environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    return int(environ['ROS_DOMAIN_ID'])
