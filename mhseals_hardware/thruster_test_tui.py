"""Live native-PWM thruster bring-up tool, designed for nested SSH PTYs."""

import argparse

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.odroid_pwm import (
    DEFAULT_MOSFET_CHIP, DEFAULT_MOSFET_LINE, DEFAULT_PWM_CHIPS,
    OdroidPWMOutputs,
)


PRESETS = {'n': ('NEUTRAL', 1500), 'f': ('FORWARD', 1600),
           'b': ('REVERSE', 1400), 'z': ('MINIMUM', 1100),
           'x': ('MAXIMUM', 1900)}


def parse_csv(text, cast=str):
    return tuple(cast(value.strip()) for value in text.split(','))


class ThrusterTestTUI:
    """Edit frequency and individual/all pulse widths without restarting."""

    def __init__(self, outputs, console=None, pulse_step=10, frequency_step=1):
        self.outputs = outputs
        self.console = console or Console()
        self.selected = 0
        self.pulses = [outputs.neutral_us] * 4
        self.pulse_step = pulse_step
        self.frequency_step = frequency_step
        self.state = 'NEUTRAL'

    def render(self):
        table = Table(title='Live ESC signal', expand=True)
        table.add_column('Select')
        table.add_column('Output')
        table.add_column('PWM device')
        table.add_column('Pulse', justify='right')
        names = ('ALL', 'FL', 'FR', 'RR', 'RL')
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
            f'[bold]Frequency:[/] {self.outputs.frequency_hz:g} Hz\n\n'
            '↑/↓ select  •  ←/→ pulse ±step  •  [/[] pulse ±1 µs\n'
            'N neutral  •  F forward  •  B reverse  •  Z/X min/max\n'
            '-/+ frequency  •  0 stop/neutral  •  Q quit safely',
            title='Thruster PWM test (live)', border_style='yellow')
        return Group(status, table)

    def targets(self):
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
            self.pulses[index] = max(1000, min(2000,
                                             self.pulses[index] + amount))
        self.state = 'CUSTOM'
        self.apply_pulses()

    def adjust_frequency(self, amount):
        frequency = max(1.0, min(400.0,
                        self.outputs.frequency_hz + amount))
        self.outputs.set_frequency(frequency)
        self.pulses = [self.outputs.neutral_us] * 4
        self.state = 'NEUTRAL (frequency changed)'

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
                    elif key in ('-', '_'):
                        self.adjust_frequency(-self.frequency_step)
                    elif key in ('+', '='):
                        self.adjust_frequency(self.frequency_step)
                    live.update(self.render())
        finally:
            self.outputs.neutral()


def build_parser():
    parser = argparse.ArgumentParser(
        description='Interactive four-thruster Odroid PWM test')
    parser.add_argument('--pwm-chips', default=','.join(DEFAULT_PWM_CHIPS),
                        help='four comma-separated pwmchip paths')
    parser.add_argument('--pwm-channels', default='0,0,0,0',
                        help='channel within each pwmchip')
    parser.add_argument('--frequency', type=float, default=50.0)
    parser.add_argument('--mosfet-chip', default=DEFAULT_MOSFET_CHIP)
    parser.add_argument('--mosfet-line', type=int, default=DEFAULT_MOSFET_LINE)
    parser.add_argument('--pulse-step', type=int, default=10)
    parser.add_argument('--frequency-step', type=float, default=1.0)
    return parser


def main(args=None):
    parsed = build_parser().parse_args(args)
    chips = parse_csv(parsed.pwm_chips)
    channels = parse_csv(parsed.pwm_channels, int)
    console = Console()
    console.print(Panel(
        '[bold red]Propellers must be clear and submerged.[/]\n'
        'The program starts neutral and always returns neutral before exit.\n'
        'Press Enter to arm PWM, or Ctrl+C to cancel.',
        title='Physical thruster warning'))
    console.input()
    outputs = OdroidPWMOutputs(
        chips, channels, parsed.frequency,
        mosfet_chip=parsed.mosfet_chip,
        mosfet_line=parsed.mosfet_line).open()
    try:
        ThrusterTestTUI(outputs, console, parsed.pulse_step,
                        parsed.frequency_step).run()
    except KeyboardInterrupt:
        console.print('\n[yellow]Interrupted; neutralizing outputs.[/]')
    finally:
        outputs.close()


if __name__ == '__main__':
    main()
