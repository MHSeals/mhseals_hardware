"""Check a hardware YAML the way the driver would load it.

    ros2 run mhseals_hardware config_check [FILE]

FILE (default: the active one, $MHSEALS_HARDWARE_CONFIG or ~/.config/mhseals/hardware.yaml;
absent, the defaults and the profile alone) is layered like the driver's config (default.yaml < the vessel profile's thrusters: < FILE)
and run through the same validators. Exit 0 prints ``ok FILE`` (plus a note when the velocity
controller would still refuse, e.g. no characterization yet); exit 1 prints the reason.
"""

import argparse
import sys

from mhseals_hardware.configuration import config_path, load_layers


def check(path=None):
    """``(ok, message)`` for one file (None: the active one); never raises on a bad file."""
    shown = path or config_path()
    try:
        config = load_layers(path)[0]
    except (OSError, ValueError, TypeError, KeyError) as error:
        return False, f'{shown}: {error}'
    try:
        from mhseals_hardware.velocity_law import Plant
        Plant(config)
        note = ''
    except ValueError as error:
        note = f' (the velocity controller would refuse: {error})'
    return True, f'ok {shown}{note}'


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('file', nargs='?', help='hardware YAML to check (default: the active one)')
    ok, message = check(parser.parse_args(args).file)
    print(message, file=sys.stdout if ok else sys.stderr)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
