"""Direct physical-output PWM test; command and module are thruster_test."""

import argparse
import time

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

from mhseals_hardware.configuration import configure_args, parse_csv
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.odroid_pwm import OdroidPWMOutputs
from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM, NEUTRAL_PWM


# Key -> (label, pulse width in us). F/B are a gentle +/-25% effort.
PRESETS = {'n': ('NEUTRAL', NEUTRAL_PWM), 'f': ('FORWARD', NEUTRAL_PWM + 100),
           'b': ('REVERSE', NEUTRAL_PWM - 100), 'z': ('MINIMUM', MIN_PWM),
           'x': ('MAXIMUM', MAX_PWM)}
# ESCs must see uninterrupted neutral after power-up before they accept throttle.
ARMING_SECONDS = 3.0


class ThrusterTestTUI:
    """Edit individual or all pulse widths live; frequency stays fixed at 50 Hz."""

    def __init__(self, outputs, console=None, pulse_step=10):
        self.outputs = outputs
        self.console = console or Console()
        self.selected = 0
        self.pulses = [outputs.neutral_us] * 4
        self.pulse_step = pulse_step
        self.state = 'NEUTRAL'

    def render(self):
        table = Table(title='Live ESC signal', expand=True)
        table.add_column('Select')
        table.add_column('Output')
        table.add_column('PWM device')
        table.add_column('Pulse', justify='right')
        names = ('ALL', 'Physical 1', 'Physical 2', 'Physical 3', 'Physical 4')
        for index, name in enumerate(names):
            marker = '[bold cyan]›[/]' if self.selected == index else ' '
            if index == 0:
                path, pulse = 'all four outputs', ', '.join(map(str, self.pulses))
            else:
                channel = self.outputs.channels[index - 1]
                path, pulse = str(channel.path), str(self.pulses[index - 1])
            table.add_row(marker, name, path, f'{pulse} µs')
        status = Panel(
            f'[bold]State:[/] [yellow]{self.state}[/]    '
            '[bold]Frequency:[/] 50 Hz (fixed)\n\n'
            '↑/↓ select  •  ←/→ pulse ±step  •  [/[] pulse ±1 µs\n'
            'N neutral  •  F forward  •  B reverse  •  Z/X min/max\n'
            '0/Space neutral  •  Q quit safely',
            title='Thruster PWM test (live)', border_style='yellow')
        return Group(status, table)

    def targets(self):
        # Row 0 is "all outputs"; rows 1-4 are the physical outputs.
        return range(4) if self.selected == 0 else (self.selected - 1,)

    def apply_pulses(self):
        self.outputs.set_pulse_widths(self.pulses)

    def set_preset(self, key):
        label, pulse = PRESETS[key]
        for index in self.targets():
            self.pulses[index] = pulse
        self.state = label
        self.apply_pulses()

    def adjust_pulse(self, amount):
        for index in self.targets():
            self.pulses[index] = max(MIN_PWM, min(MAX_PWM, self.pulses[index] + amount))
        self.state = 'CUSTOM'
        self.apply_pulses()

    def run(self):
        try:
            with KeyReader() as keys, Live(
                    self.render(), console=self.console,
                    refresh_per_second=10, screen=False) as live:
                while True:
                    key = keys.read(timeout=0.1)
                    if key in ('q', 'escape'):
                        return
                    if key == 'up':
                        self.selected = (self.selected - 1) % 5
                    elif key == 'down':
                        self.selected = (self.selected + 1) % 5
                    elif key == 'left':
                        self.adjust_pulse(-self.pulse_step)
                    elif key == 'right':
                        self.adjust_pulse(self.pulse_step)
                    elif key == '[':
                        self.adjust_pulse(-1)
                    elif key == ']':
                        self.adjust_pulse(1)
                    elif key in PRESETS:
                        self.set_preset(key)
                    elif key == '0' or key == 'space':
                        self.pulses = [self.outputs.neutral_us] * 4
                        self.state = 'NEUTRAL'
                        self.apply_pulses()
                    live.update(self.render())
        finally:
            # Any exit path, including exceptions, leaves the thrusters stopped.
            self.outputs.neutral()


def build_parser():
    parser = argparse.ArgumentParser(
        description='Interactive four-thruster Odroid PWM test')
    # Defaults for these come from the hardware config via configure_args.
    parser.add_argument('--pwm-chips', type=parse_csv,
                        help='four comma-separated pwmchip paths')
    parser.add_argument('--pwm-channels', type=lambda s: parse_csv(s, int),
                        help='channel within each pwmchip')
    parser.add_argument('--mosfet-chip')
    parser.add_argument('--mosfet-line', type=int)
    parser.add_argument('--pulse-step', type=int, default=10)
    return parser


def main(args=None):
    parsed = configure_args(build_parser(), args)
    console = Console()
    console.print(Panel(
        '[bold red]Propellers must be clear and submerged.[/]\n'
        'The program starts neutral and always returns neutral before exit.\n'
        'Press Enter to arm PWM, or Ctrl+C to cancel.',
        title='Physical thruster warning'))
    console.input()
    outputs = OdroidPWMOutputs.from_config(parsed.hardware_config).open()
    try:
        console.print(
            f'[yellow]Holding neutral for {ARMING_SECONDS:g} seconds '
            'while ESCs arm; controls are locked.[/]')
        time.sleep(ARMING_SECONDS)
        ThrusterTestTUI(outputs, console, parsed.pulse_step).run()
    except KeyboardInterrupt:
        console.print('\n[yellow]Interrupted; neutralizing outputs.[/]')
    finally:
        outputs.close()


if __name__ == '__main__':
    main()
