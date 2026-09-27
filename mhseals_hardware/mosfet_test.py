"""Bounded MOSFET enable check; does not configure or write PWM."""

import argparse
import time

from mhseals_hardware.configuration import configure_args
from mhseals_hardware.odroid_pwm import MosfetEnable, resolve_gpio_chip


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mosfet-chip', help='bank label or /dev/gpiochipN')
    parser.add_argument('--mosfet-line', type=int)
    parser.add_argument('--seconds', type=float, default=5)
    parsed = configure_args(parser, args)
    if not 0 < parsed.seconds <= 10:
        parser.error('--seconds must be within (0, 10]')
    chip = resolve_gpio_chip(parsed.mosfet_chip)
    print(f'{parsed.mosfet_chip} -> {chip}, offset {parsed.mosfet_line}, '
          f'active_high={parsed.mosfet_active_high}')
    print('Disconnect propulsion power and stop all PWM/controller programs. '
          'This toggles the enable only; existing PWM is NOT changed. '
          'Measure the trigger against signal ground.')
    if input('Type ENABLE to test, anything else cancels: ').strip() != 'ENABLE':
        return
    output = MosfetEnable(chip, parsed.mosfet_line, parsed.mosfet_active_high)
    try:
        output.open()
        print('Inactive for 2 seconds.', flush=True)
        time.sleep(2)
        output.set_enabled(True)
        print(f'ENABLED for {parsed.seconds:g} seconds.', flush=True)
        time.sleep(parsed.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        output.close()
        print('Released; use an external pull-down for a defined idle level.')


if __name__ == '__main__':
    main()
