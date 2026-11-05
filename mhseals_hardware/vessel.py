"""Vessel facts from the vessel profile (astro_dock's profiles), ROS-free.

The vessel's directory is the one holding its network env (``env_path()``: mhseals_nav ``boat_network.path()``'s
contract, so hardware needs no mhseals_nav). Its profile.yaml may ``extends`` a fleet profile; ``section(name)`` lays
each profile's NAME.yaml and then its profile.yaml ``name:`` mapping over the last, fleet first (astro_dock's device
kinds), so a vessel overrides only what differs. Without a vessel profile every section reads as {} and callers keep
their package defaults. A named vessel that cannot be read refuses (ValueError) rather than falling back to the
defaults: a set ``$ASTRO_NETWORK_ENV`` (or ``network_env``) that is missing, or a profile.yaml in the chain that is
missing, and every file read with the operator file's strict loader (duplicate keys, anchors, syntax errors).
A ``$ROS_WS`` without the env file is a workspace with no vessel.
"""

import os
from pathlib import Path

import yaml

DEPTH = 8   # extends hops followed before giving up on a cycle


def _mapping(path, required=False):
    from mhseals_hardware.configuration import load_strict_yaml    # configuration imports this module
    if not path.is_file():
        if required:
            raise ValueError(f'vessel profile {path} is missing')
        return {}
    return load_strict_yaml(path)


def env_path():
    """``$ASTRO_NETWORK_ENV``, else ``$ROS_WS/config/boat-network.env``, else None."""
    explicit, workspace = os.environ.get('ASTRO_NETWORK_ENV'), os.environ.get('ROS_WS')
    if explicit:
        return Path(explicit).expanduser()
    return Path(workspace) / 'config' / 'boat-network.env' if workspace else None


def directory(network_env=None):
    """The vessel profile directory: the env file's resolved parent (a symlinked legacy ``$ROS_WS/config`` env
    names its target's vessel), or None without an env file."""
    env = Path(network_env).expanduser() if network_env else env_path()
    return env.resolve().parent if env is not None and env.is_file() else None


def section(name, network_env=None):
    """The vessel profile chain's merged ``name:`` mapping ({} without a vessel profile or that key)."""
    named = network_env or os.environ.get('ASTRO_NETWORK_ENV')
    env = Path(network_env).expanduser() if network_env else env_path()
    if env is None or not (named or env.is_file()):
        return {}
    if not env.is_file():
        raise ValueError(f'vessel network env {env} is missing')
    chain, here = [], directory(network_env)
    while here is not None and len(chain) < DEPTH and here not in chain:
        chain.append(here)
        parent = _mapping(here / 'profile.yaml', required=True).get('extends')
        here = here.parent / parent if isinstance(parent, str) else None
    merged = {}
    for profile in reversed(chain):
        merged.update(_mapping(profile / f'{name}.yaml'))
        found = _mapping(profile / 'profile.yaml').get(name, {})
        if not isinstance(found, dict):
            raise ValueError(f'{profile / "profile.yaml"}: {name}: must be a mapping')
        merged.update(found)
    return merged
