"""Hold the MOSFET enable on (ESC power) until Ctrl-C, SIGTERM/SIGHUP or --seconds; writes no PWM."""

import argparse
import signal
import time

from mhseals_hardware.configuration import configure_args
from mhseals_hardware.odroid_pwm import MosfetEnable, resolve_gpio_chip


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mosfet-chip', help='bank label or /dev/gpiochipN')
    parser.add_argument('--mosfet-line', type=int)
    parser.add_argument('--seconds', type=float, help='release after this long (default: hold until stopped)')
    parsed = configure_args(parser, args)
    if parsed.seconds is not None and parsed.seconds <= 0:
        parser.error('--seconds must be positive')
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)
    output = MosfetEnable(parsed.mosfet_chip, parsed.mosfet_line, parsed.mosfet_active_high)
    try:
        output.open()
    except OSError as error:
        output.close()
        raise SystemExit(f'MOSFET line busy ({error}); `thrusters free` shows and stops its holder')
    try:
        output.set_enabled(True)
        print(f'{parsed.mosfet_chip} -> {resolve_gpio_chip(parsed.mosfet_chip)} line {parsed.mosfet_line} '
              f'(active {"high" if parsed.mosfet_active_high else "low"}): MOSFET on. '
              'Ctrl-C or `thrusters free` releases it', flush=True)
        if parsed.seconds is None:
            while True:
                signal.pause()
        time.sleep(parsed.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        output.close()
        print('MOSFET released', flush=True)


if __name__ == '__main__':
    main()
