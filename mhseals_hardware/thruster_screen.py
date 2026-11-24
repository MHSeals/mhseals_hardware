"""`thrusters` (also `thrusters test`, `thrusters config`): one screen to drive and configure the outputs.

One row per thruster (FL FR RR RL) shows its output, J2 pin, PWM chip, reversal, the commanded pulse
with its effort and a strength bar, and the pulse read back from sysfs. Pulsing opens the outputs (the
MOSFET on, every ESC held at neutral while it arms). The wiring and pins (channel_map, reversed,
pwm_chips) save to the vessel profile's ``thrusters:``, a diff first. Quitting leaves neutral, MOSFET off.
"""

import copy
import difflib
import glob
import os
from pathlib import Path
import re
import signal
import tempfile
import time

import yaml
from rich.console import Console, Group
from rich.live import Live
from rich.table import Table
from rich.text import Text

from mhseals_hardware import free, vessel
from mhseals_hardware.configuration import (
    WIRING, _layered, config_path, load_defaults, load_strict_yaml, plain,
)
from mhseals_hardware.odroid_pwm import (
    OVERLAY, OdroidPWMOutputs, chip_glob, check_pins, exposed_chips, pin_label, pwm_device,
)
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, NEUTRAL_PWM, POSITIONS, PWM_SCALE, effort_to_pulse,
)

ARMING_SECONDS = 3.0   # ESCs need uninterrupted neutral after power-up before they take throttle
PRESET = 0.25          # f/b effort
BAR_HALF = 10          # bar cells each side of neutral
REFRESH_HZ = 10
SLOW_SECONDS = 1.0     # the MOSFET line and the process list are read this often
KEYS = '↑↓ select  ←→ ±10µs  [] ±1µs  f/b ±25%  0 neutral  z/x min/max  t out  r rev  p pin  m MOSFET  k free  s save  q quit'


# Pure parts.

def effort_bar(pulse, half=BAR_HALF):
    """``-100 % … 0 … +100 %`` as ``half`` cells each side of a centre tick: reverse left, forward right."""
    effort = max(-1.0, min(1.0, (pulse - NEUTRAL_PWM) / PWM_SCALE))
    cells = round(abs(effort) * half)
    left = ' ' * (half - cells) + '█' * cells if effort < 0 else ' ' * half
    right = '█' * cells + ' ' * (half - cells) if effort > 0 else ' ' * half
    return left + '│' + right


def profile_mapping(own, start, current, base):
    """The vessel's own ``thrusters:`` after this session: each key changed from ``start`` set to its
    ``current`` value, or dropped when that equals ``base`` (defaults < what the vessel extends)."""
    mapping = dict(own)
    for key in WIRING:
        if plain(current[key]) == plain(start[key]):
            continue
        if plain(current[key]) == plain(base[key]):
            mapping.pop(key, None)
        else:
            mapping[key] = plain(current[key])
    return mapping


def profile_text(text, mapping):
    """``text`` (a profile.yaml) with its top-level ``thrusters:`` replaced by ``mapping``; comments and
    every other key kept. An empty mapping drops the key."""
    lines = text.splitlines(keepends=True)
    start = next((i for i, line in enumerate(lines) if re.match(r'thrusters\s*:', line)), None)
    if start is None:
        start = end = len(lines)
        if lines and not lines[-1].endswith('\n'):
            lines[-1] += '\n'
    else:
        end = start + 1
        while end < len(lines) and (not lines[end].strip() or lines[end][0] in ' \t'):
            end += 1
        while end > start + 1 and not lines[end - 1].strip():
            end -= 1    # trailing blank lines stay
    block = []
    if mapping:
        flow = 'thrusters: ' + yaml.safe_dump(dict(mapping), default_flow_style=True, sort_keys=False,
                                              width=float('inf')).strip()
        block = [flow + '\n'] if len(flow) <= 110 else yaml.safe_dump(
            {'thrusters': dict(mapping)}, default_flow_style=None, sort_keys=False,
            width=110).splitlines(keepends=True)
    new = ''.join(lines[:start] + block + lines[end:])
    before, after = yaml.safe_load(text) or {}, yaml.safe_load(new) or {}
    before.pop('thrusters', None)
    if after.pop('thrusters', {}) != (dict(mapping) or {}) or after != before:
        raise ValueError('could not rewrite thrusters: without touching the rest of the profile')
    return new


def diff(old, new, name):
    return ''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True), name, name))


def write_atomic(path, text):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix='.profile-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(text)
        os.chmod(temporary, path.stat().st_mode & 0o777 if path.exists() else 0o644)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


# The screen.

class Screen:
    """State and keys of the thrusters screen; ``render`` draws it, ``key`` acts on one key."""

    def __init__(self, console=None, clock=time.monotonic):
        self.console = console or Console()
        self.clock = clock
        defaults = load_defaults()
        self.base = dict(defaults, **vessel.section('thrusters', inherited=True))
        self.start = _layered({})            # defaults < the vessel profile (the operator file aside)
        for key in WIRING:                   # lists, edited in place
            if isinstance(self.start[key], (list, tuple)):
                self.start[key] = list(self.start[key])
        self.config = copy.deepcopy(self.start)
        directory = vessel.directory()
        self.profile = directory / 'profile.yaml' if directory else None
        self.pulses = [NEUTRAL_PWM] * 4      # by output
        self.selected = 0                    # 0-3 a thruster (POSITIONS), 4 all
        self.outputs = None
        self.armed_at = None
        self.picker = None                   # (title, [(label, value)], on_pick, index)
        self.pending = None                  # (new profile text, diff) awaiting y
        self.message = self.operator_note()
        self.slow_at = None
        self.mosfet_note, self.holders = '', []

    def operator_note(self):
        path = config_path()
        try:
            wins = sorted(set(load_strict_yaml(path)) & {'channel_map', 'reversed'}) if path.exists() else []
        except ValueError:
            wins = []
        return f'{path} sets {", ".join(wins)}: it wins over the profile' if wins else ''

    # What is on the pins.

    def refresh(self):
        now = self.clock()
        if self.slow_at is not None and now - self.slow_at < SLOW_SECONDS:
            return
        self.slow_at = now
        self.holders = free.processes()
        if self.outputs is not None and self.outputs.mosfet is not None:
            self.mosfet_note = ('on' if self.outputs.mosfet.enabled else 'off') + ' (this screen)'
            return
        try:
            used, consumer = free.line_info(self.config['mosfet_chip'], self.config['mosfet_line'])
        except Exception as error:
            self.mosfet_note = f'unknown ({error})'
            return
        owners = free.gpio_holders(self.holders)
        self.mosfet_note = ('held by ' + (', '.join(f'{n} {p}' for n, p in owners) or consumer)
                            if used else 'free (off)')

    def readback(self, output):
        """``enable duty`` of one output read from sysfs, or a dash."""
        path = free.channel_path(self.config['pwm_chips'][output - 1], self.config['pwm_channels'][output - 1])
        if path is None or not path.exists():
            return '-'
        enable, duty = free.read(path, 'enable'), free.read(path, 'duty_cycle')
        if enable != '1':
            return 'off'
        return f'{int(duty) // 1000} µs' if duty.isdigit() else duty

    # Outputs.

    def arming_left(self):
        return 0.0 if self.armed_at is None else max(0.0, self.armed_at - self.clock())

    def open_outputs(self):
        """True once the outputs are open (MOSFET on, neutral); the ESCs then arm for ARMING_SECONDS."""
        if self.outputs is not None:
            return True
        problems = check_pins(self.config)
        if problems:
            self.message = problems[0]
            return False
        try:
            self.outputs = OdroidPWMOutputs.from_config(self.config).open()
        except Exception as error:
            busy = any(name == 'thruster_pwm_node' for name, _ in self.holders)
            self.message = ('drive is running: astro down drive, or press k to free' if busy
                            else f'cannot open the outputs: {error}')
            self.outputs = None
            return False
        self.pulses = [NEUTRAL_PWM] * 4
        self.armed_at = self.clock() + ARMING_SECONDS
        self.slow_at = None
        return True

    def close_outputs(self):
        if self.outputs is not None:
            try:
                self.outputs.close()
            finally:
                self.outputs, self.armed_at, self.slow_at = None, None, None
        self.pulses = [NEUTRAL_PWM] * 4

    def targets(self):
        """The outputs the selection drives: one thruster's, or all four."""
        return range(1, 5) if self.selected == 4 else (self.config['channel_map'][self.selected],)

    def command(self, pulses):
        """Set pulses (clamped to the ESC range) on the selected outputs; opens the outputs first."""
        if not self.open_outputs():
            return
        if self.arming_left() > 0:
            self.message = f'ESCs arming at neutral, {self.arming_left():.1f} s'
            return
        for output in self.targets():
            self.pulses[output - 1] = max(MIN_PWM, min(MAX_PWM, int(pulses(output))))
        self.outputs.set_pulse_widths(self.pulses)
        self.message = ''

    def preset(self, effort):
        """±effort on each selected thruster, honouring its reversal."""
        def pulse(output):
            index = self.config['channel_map'].index(output)
            return effort_to_pulse(-effort if self.config['reversed'][index] else effort)
        self.command(pulse)

    def neutral_all(self):
        self.pulses = [NEUTRAL_PWM] * 4
        if self.outputs is not None:
            self.outputs.neutral()

    def toggle_mosfet(self):
        if self.outputs is None or self.outputs.mosfet is None:
            if self.open_outputs():
                self.message = 'MOSFET on, ESCs arming at neutral'
            return
        self.neutral_all()
        on = not self.outputs.mosfet.enabled
        self.outputs.mosfet.set_enabled(on)
        self.armed_at = self.clock() + ARMING_SECONDS if on else None
        self.message, self.slow_at = f'MOSFET {"on" if on else "off"}', None

    def free_pins(self):
        others = free.processes()
        if not others:
            self.message = 'nothing else holds the pins'
            return
        free.stop(others)
        self.message, self.slow_at = 'stopped ' + ', '.join(f'{n} {p}' for n, p in others), None

    # Config edits.

    def pick(self, title, options, on_pick):
        self.picker = [title, options, on_pick, 0]

    def assign_output(self):
        if self.selected == 4:
            return
        position = self.selected

        def chosen(output):
            mapping = self.config['channel_map']
            other = mapping.index(output)
            mapping[other], mapping[position] = mapping[position], output
        current = self.config['channel_map'][position]
        self.pick(f'{POSITIONS[position].upper()} is on output…',
                  [(f'output {o}  {pin_label(self.config["pwm_chips"][o - 1])}', o) for o in range(1, 5)],
                  chosen)
        self.picker[3] = current - 1

    def toggle_reversed(self):
        if self.selected != 4:
            self.config['reversed'][self.selected] = not self.config['reversed'][self.selected]

    def pick_chip(self):
        if self.selected == 4:
            return
        output = self.config['channel_map'][self.selected]
        devices = exposed_chips()
        if not devices:
            self.message = f'this host exposes no PWM chips; enable them in {OVERLAY}'
            return

        def chosen(device):
            chips = self.config['pwm_chips']
            new = chip_glob(device)
            if new in chips:            # another output had it: they swap
                chips[chips.index(new)] = chips[output - 1]
            chips[output - 1] = new
            self.close_outputs()
            self.message = 'pins changed; outputs closed, a pulse reopens them'
        in_use = {pwm_device(chip): index + 1 for index, chip in enumerate(self.config['pwm_chips'])}
        self.pick(f'PWM chip for output {output}',
                  [(f'{d}  {pin_label(chip_glob(d))}' + (f'  (output {in_use[d]})' if d in in_use else ''), d)
                   for d in devices], chosen)
        current = pwm_device(self.config['pwm_chips'][output - 1])
        self.picker[3] = devices.index(current) if current in devices else 0

    def save(self):
        if self.profile is None:
            self.message = 'no vessel profile here ($ASTRO_NETWORK_ENV or $ROS_WS): nothing to save to'
            return
        try:
            old = self.profile.read_text()
            own = (yaml.safe_load(old) or {}).get('thrusters') or {}
            mapping = profile_mapping(own, self.start, self.config, self.base)
            _layered({}, dict(vessel.section('thrusters', inherited=True), **mapping))   # validates
            if plain(self.config['pwm_chips']) != plain(self.start['pwm_chips']):
                problems = check_pins(self.config)
                if problems:
                    raise ValueError(problems[0])
            new = profile_text(old, mapping)
        except (OSError, ValueError) as error:
            self.message = f'not saved: {error}'
            return
        if new == old:
            self.message = 'nothing to save'
            return
        self.pending = (new, diff(old, new, str(self.profile)))

    def confirm(self, key):
        new, _ = self.pending
        self.pending = None
        if key != 'y':
            self.message = 'not saved'
            return
        write_atomic(self.profile, new)
        moved = any(plain(self.config[k]) != plain(self.start[k]) for k in WIRING[2:])
        self.start = copy.deepcopy(self.config)
        self.message = (f'saved {self.profile}; commit it in the profiles repo' +
                        ('; pins moved: re-run odroid_access on the ODROID host' if moved else ''))

    # Keys.

    def key(self, key):
        """Act on one key; False to quit."""
        if self.pending is not None:
            self.confirm(key)
            return True
        if self.picker is not None:
            title, options, on_pick, index = self.picker
            if key in ('up', 'down'):
                self.picker[3] = (index + (1 if key == 'down' else -1)) % len(options)
            elif key == 'enter':
                self.picker = None
                on_pick(options[index][1])
            elif key in ('escape', 'q'):
                self.picker = None
            return True
        if key in ('q', 'escape'):
            return False
        if key == 'up':
            self.selected = (self.selected - 1) % 5
        elif key == 'down':
            self.selected = (self.selected + 1) % 5
        elif key in ('left', 'right', '[', ']'):
            step = {'left': -10, 'right': 10, '[': -1, ']': 1}[key]
            self.command(lambda output: self.pulses[output - 1] + step)
        elif key in ('f', 'b'):
            self.preset(PRESET if key == 'f' else -PRESET)
        elif key in ('z', 'x'):
            self.command(lambda output: MIN_PWM if key == 'z' else MAX_PWM)
        elif key in ('0', 'space'):
            self.neutral_all()
        elif key == 't':
            self.assign_output()
        elif key == 'r':
            self.toggle_reversed()
        elif key == 'p':
            self.pick_chip()
        elif key == 'm':
            self.toggle_mosfet()
        elif key == 'k':
            self.free_pins()
        elif key == 's':
            self.save()
        return True

    # Drawing.

    def header(self):
        drive = [f'{n} {p}' for n, p in self.holders if n == 'thruster_pwm_node']
        left = self.arming_left()
        esc = ('closed (a pulse or m opens them)' if self.outputs is None else
               f'arming at neutral, {left:.1f} s' if left > 0 else
               'armed' if self.outputs.mosfet is None or self.outputs.mosfet.enabled else 'unpowered')
        lines = [f'MOSFET {self.mosfet_note}    ESCs {esc}    profile {self.profile or "none"}']
        if drive:
            lines.append(f'drive is running ({", ".join(drive)}): astro down drive, or press k to free')
        return Text('\n'.join(lines))

    def table(self):
        table = Table(box=None, padding=(0, 1), header_style='bold')
        for column in ('', 'thruster', 'out', 'pin', 'chip', 'rev', 'pulse', 'effort',
                       '-100%     0     +100%', 'readback'):
            table.add_column(column, justify='right' if column in ('pulse', 'effort') else 'left')
        changed = {key for key in WIRING if plain(self.config[key]) != plain(self.start[key])}
        for index, position in enumerate(POSITIONS):
            output = self.config['channel_map'][index]
            chip = self.config['pwm_chips'][output - 1]
            pulse = self.pulses[output - 1]
            found = len(glob.glob(chip))
            table.add_row(
                '›' if self.selected == index else ' ', position.upper(),
                Text(str(output), style='yellow' if 'channel_map' in changed else None),
                Text(pin_label(chip), style='yellow' if 'pwm_chips' in changed else None),
                '✓' if found == 1 else '✗',
                Text('yes' if self.config['reversed'][index] else '-',
                     style='yellow' if 'reversed' in changed else None),
                f'{pulse} µs', f'{(pulse - NEUTRAL_PWM) / PWM_SCALE:+.0%}',
                Text(effort_bar(pulse), style='bold' if pulse != NEUTRAL_PWM else 'dim'),
                self.readback(output))
        table.add_row('›' if self.selected == 4 else ' ', 'ALL', '', '', '', '', '', '', '', '')
        return table

    def footer(self):
        parts = []
        missing = [pwm_device(chip) for chip in self.config['pwm_chips'] if not glob.glob(chip)]
        if missing and any(missing):
            parts.append(f'not enabled on this host: {", ".join(m for m in missing if m)} (the overlay is {OVERLAY})')
        if self.picker is not None:
            title, options, _, index = self.picker
            parts.append(title + '  (↑/↓, Enter, Esc)')
            parts += [('› ' if i == index else '  ') + label for i, (label, _) in enumerate(options)]
        if self.pending is not None:
            parts += [self.pending[1].rstrip(), 'y saves this to the profile, any other key cancels']
        if self.message:
            parts.append(self.message)
        parts.append(KEYS)
        return Text('\n'.join(parts))

    def render(self):
        self.refresh()
        return Group(self.header(), self.table(), self.footer())

    def close(self):
        self.close_outputs()


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def run(args=None):
    from mhseals_hardware.keyboard import KeyReader
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)
    screen = Screen()
    try:
        with KeyReader() as keys, Live(screen.render(), console=screen.console,
                                       refresh_per_second=REFRESH_HZ, auto_refresh=False) as live:
            while True:
                key = keys.read(timeout=1 / REFRESH_HZ)
                key = key.lower() if isinstance(key, str) and len(key) == 1 else key
                if key is not None and not screen.key(key):
                    break
                live.update(screen.render(), refresh=True)
    except KeyboardInterrupt:
        pass
    finally:
        screen.close()
        screen.console.print('Outputs neutral, MOSFET off.')
