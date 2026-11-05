"""Vessel facts from the vessel profile (astro_dock's profiles), ROS-free.

The vessel's directory is the one holding its network env (``env_path()``: mhseals_nav ``boat_network.path()``'s
contract, so hardware needs no mhseals_nav). Its profile.yaml may ``extends`` a fleet profile; ``section(name)`` lays
each profile's NAME.yaml and then its profile.yaml ``name:`` mapping over the last, fleet first (astro_dock's device
kinds), so a vessel overrides only what differs. Without a vessel profile every section reads as {} and callers keep
their package defaults.
"""

from functools import cache
import os
from pathlib import Path

import yaml

DEPTH = 8   # extends hops followed before giving up on a cycle


def _mapping(path):
    try:
        data = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError):
        return {}
    return data if isinstance(data, dict) else {}


def env_path():
    """``$ASTRO_NETWORK_ENV``, else ``$ROS_WS/config/boat-network.env``, else None."""
    explicit, workspace = os.environ.get('ASTRO_NETWORK_ENV'), os.environ.get('ROS_WS')
    if explicit:
        return Path(explicit).expanduser()
    return Path(workspace) / 'config' / 'boat-network.env' if workspace else None


@cache
def section(name, network_env=None):
    """The vessel profile chain's merged ``name:`` mapping ({} without a vessel profile or that key)."""
    env = Path(network_env) if network_env else env_path()
    if env is None or not env.is_file():
        return {}
    chain, here = [], env.parent
    while here is not None and len(chain) < DEPTH and here not in chain:
        chain.append(here)
        parent = _mapping(here / 'profile.yaml').get('extends')
        here = here.parent / parent if isinstance(parent, str) else None
    merged = {}
    for directory in reversed(chain):
        merged.update(_mapping(directory / f'{name}.yaml'))
        found = _mapping(directory / 'profile.yaml').get(name)
        if isinstance(found, dict):
            merged.update(found)
    return merged
