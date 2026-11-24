"""Check a hardware YAML the way the driver would load it.

    ros2 run mhseals_hardware config_check FILE

FILE is layered like the driver's config (default.yaml < the vessel profile's thrusters: < FILE)
and run through the same validators. Exit 0 prints ``ok FILE`` (plus a note when the velocity
controller would still refuse, e.g. no characterization yet); exit 1 prints the reason.
"""

import argparse
import sys

from mhseals_hardware.configuration import load_layers


def check(path):
    """``(ok, message)`` for one file; never raises on a bad file."""
    try:
        config = load_layers(path)[0]
    except (OSError, ValueError, TypeError, KeyError) as error:
        return False, f'{path}: {error}'
    try:
        from mhseals_hardware.velocity_law import Plant
        Plant(config)
        note = ''
    except ValueError as error:
        note = f' (the velocity controller would refuse: {error})'
    return True, f'ok {path}{note}'


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('file', help='hardware YAML to check')
    ok, message = check(parser.parse_args(args).file)
    print(message, file=sys.stdout if ok else sys.stderr)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
