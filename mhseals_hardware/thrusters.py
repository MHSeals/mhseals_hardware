"""thrusters: the thruster bench tools.

    thrusters [test|config]          the thrusters screen: drive each output, set the wiring and pins
    thrusters wiring [--manual]      pulse each output in turn and say what moved; saves channel_map, reversed
    thrusters mosfet [--seconds N]   hold the MOSFET (ESC power) on; Ctrl-C releases it
    thrusters free [--check]         stop whatever holds the pins, neutral, release; report each output
"""

import sys

SUBCOMMANDS = ('test', 'config', 'wiring', 'mosfet', 'free')


def main(args=None):
    args = sys.argv[1:] if args is None else list(args)
    sub, rest = (args[0], args[1:]) if args else (None, [])
    if sub in (None, 'test', 'config'):
        if not sys.stdin.isatty():
            print(__doc__.rstrip())
            return 0 if sub is None else 1
        from mhseals_hardware.thruster_screen import run
        return run(rest)
    if sub == 'wiring':
        from mhseals_hardware.identification import main as wiring
        return wiring(rest)
    if sub == 'mosfet':
        from mhseals_hardware.mosfet import main as mosfet
        return mosfet(rest)
    if sub == 'free':
        from mhseals_hardware.free import main as free
        return free(rest)
    print(__doc__.rstrip(), file=sys.stdout if sub in ('-h', '--help') else sys.stderr)
    return 0 if sub in ('-h', '--help') else 2


if __name__ == '__main__':
    sys.exit(main())
