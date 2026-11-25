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
    J2_PINS, OVERLAY, OdroidPWMOutputs, chip_glob, check_pins, exposed_chips, pin_label, pwm_device,
)
from mhseals_hardware.thruster_mixer import (
    MAX_PWM, MIN_PWM, NEUTRAL_PWM, POSITIONS, PWM_SCALE, effort_to_pulse,
)

ARMING_SECONDS = 3.0   # ESCs need uninterrupted neutral after power-up before they take throttle
PRESET = 0.25          # f/b effort
BAR_HALF = 8           # bar cells each side of neutral
REFRESH_HZ = 10
SLOW_SECONDS = 1.0     # the MOSFET line and the process list are read this often
# The look, mirrored from astro_dock's tools/astro/style.py (this package cannot import it): the selected
# row is a marker plus bold, its name in the one accent colour; descriptions and key names' actions dim.
MARK, CURRENT, ACCENT = '\u203a', '(current)', 'cyan'
BAR = {'track': '\u00b7', 'fill': '\u2588', 'centre': '\u2502'}   # the strength bar's one glyph set
# The key help, one labelled line per group: (key, what it does).
KEYS = (('Drive', (('↑↓', 'select row'), ('←→', '±10µs'), ('[ ]', '±1µs'), ('f/b', '±25%'), ('0', 'neutral'),
                   ('z/x', 'min/max'))),
        ('Wiring', (('t', 'output'), ('r', 'reversed'), ('p', 'pin'), ('s', 'save to profile'))),
        ('Power', (('m', 'MOSFET on/off'), ('k', 'free pins'), ('q', 'quit (neutral, MOSFET off)'))))
PICKER_KEYS = (('↑↓', 'move'), ('Enter', 'choose'), ('Esc/q', 'cancel'))


# Pure parts.

def effort_bar(pulse, half=BAR_HALF):
    """``-100 % … 0 … +100 %`` as ``half`` cells each side of a centre tick: reverse fills left, forward
    right, the rest a dotted track (one glyph set, BAR)."""
    effort = max(-1.0, min(1.0, (pulse - NEUTRAL_PWM) / PWM_SCALE))
    cells = round(abs(effort) * half)
    track, fill = BAR['track'], BAR['fill']
    left = track * (half - cells) + fill * cells if effort < 0 else track * half
    right = fill * cells + track * (half - cells) if effort > 0 else track * half
    return left + BAR['centre'] + right


def effort_text(pulse):
    """The effort of a pulse, right-aligned in five cells: ``+50%``, ``-100%``, ``0%`` at neutral."""
    effort = round((pulse - NEUTRAL_PWM) / PWM_SCALE * 100)
    return f'{effort:+d}%'.rjust(5) if effort else '0%'.rjust(5)


def styled_bar(pulse, half=BAR_HALF):
    """effort_bar as rich Text: the track dim, the fill bold, the centre tick plain."""
    text = Text()
    for glyph in effort_bar(pulse, half):
        text.append(glyph, style='dim' if glyph == BAR['track'] else 'bold' if glyph == BAR['fill'] else None)
    return text


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


def key_line(pairs):
    """``key action`` pairs as rich Text: the key bold, its action dim, two spaces between."""
    text = Text()
    for i, (key, action) in enumerate(pairs):
        text.append('  ' if i else '')
        text.append(key, style='bold')
        text.append(' ' + action, style='dim')
    return text


def key_help():
    """The key help, one line per group, its label first (``Drive:  ↑↓ select row  …``), as rich Text."""
    width = max(len(label) for label, _ in KEYS) + 1
    lines = []
    for label, pairs in KEYS:
        line = Text()
        line.append(f'{label + ":":<{width}} ', style='bold')
        lines.append(line + key_line(pairs))
    return lines


def chip_rows(devices_exposed, chips):
    """[(label, device, enabled)] for the pin picker: every J2 PWM device of the header table plus any other
    the host exposes, with its J2 pin, whether the overlay enables it here, and the output using it."""
    known = [d for d in J2_PINS if d.endswith('.pwm')]
    devices = known + sorted(set(devices_exposed) - set(known))
    in_use = {pwm_device(chip): output for output, chip in enumerate(chips, 1)}
    rows = []
    for device in devices:
        pin = J2_PINS.get(device)
        enabled = device in devices_exposed
        rows.append((f'{"J2 " + str(pin[0]) if pin else "J2 ?":<6} {pin[2] if pin else "":<6} {device:<13} '
                     f'{"enabled" if enabled else "not in overlay":<16}'
                     + (f'output {in_use[device]}' if device in in_use else 'free'), device, enabled))
    return rows


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
        self.picker = None                   # dict: title, header, options [(label, value)], on_pick, index, current
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

    def pick(self, title, options, on_pick, current=0, header=''):
        self.picker = dict(title=title, header=header, options=options, on_pick=on_pick, index=current,
                           current=current)

    def assign_output(self):
        if self.selected == 4:
            return
        position = self.selected

        def chosen(output):
            mapping = self.config['channel_map']
            other = mapping.index(output)
            mapping[other], mapping[position] = mapping[position], output
        current = self.config['channel_map'][position]
        self.pick(f'Output for thruster {POSITIONS[position].upper()}:',
                  [(f'output {o}  {pin_label(self.config["pwm_chips"][o - 1])}', o) for o in range(1, 5)],
                  chosen, current - 1)

    def toggle_reversed(self):
        if self.selected != 4:
            self.config['reversed'][self.selected] = not self.config['reversed'][self.selected]

    def pick_chip(self):
        if self.selected == 4:
            return
        output = self.config['channel_map'][self.selected]
        rows = chip_rows(exposed_chips(), self.config['pwm_chips'])
        enabled = {device for _, device, on in rows if on}

        def chosen(device):
            if device not in enabled:
                self.message = f'{device} is not enabled on this host: add it to {OVERLAY}; pin unchanged'
                return
            chips = self.config['pwm_chips']
            new = chip_glob(device)
            if new in chips:            # another output had it: they swap
                chips[chips.index(new)] = chips[output - 1]
            chips[output - 1] = new
            self.close_outputs()
            self.message = 'pins changed; outputs closed, a pulse reopens them'
        devices = [device for _, device, _ in rows]
        current = pwm_device(self.config['pwm_chips'][output - 1])
        self.pick(f'Pin for output {output} (thruster {POSITIONS[self.selected].upper()}):',
                  [(label, device) for label, device, _ in rows], chosen,
                  devices.index(current) if current in devices else 0,
                  f'{"pin":<6} {"PWM":<6} {"device":<13} {"overlay":<16}used by')

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
            options, index = self.picker['options'], self.picker['index']
            if key in ('up', 'down'):
                self.picker['index'] = (index + (1 if key == 'down' else -1)) % len(options)
            elif key == 'enter':
                on_pick, self.picker = self.picker['on_pick'], None
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
        text = Text()
        text.append('thrusters', style=f'bold {ACCENT}')
        for label, value in (('MOSFET', self.mosfet_note), ('ESCs', esc)):
            text.append(f'   {label} ', style='bold')
            text.append(value)
        text.append('\n')
        text.append('profile ', style='bold')
        text.append(str(self.profile or 'none'), style='dim')
        if drive:
            text.append(f'\ndrive is running ({", ".join(drive)}): astro down drive, or press k to free',
                        style='bold')
        return text

    def table(self):
        """One row per thruster and ALL: the selected one marked (marker, bold, its name in the accent) with
        its editable cells in [brackets]; changed wiring yellow; the bar centred on neutral, the effort
        right-aligned after it."""
        table = Table(box=None, padding=(0, 1, 0, 0), header_style='bold', pad_edge=False)
        for column in ('thruster', 'out', 'pin', 'chip', 'rev', 'pulse', ' -100%   0   +100%', 'effort',
                       'readback'):
            table.add_column(column, justify='right' if column in ('pulse', 'effort') else 'left',
                             no_wrap=True)
        changed = {key for key in WIRING if plain(self.config[key]) != plain(self.start[key])}

        def cell(text, key, chosen):
            return Text(f'[{text}]' if chosen else text, style='yellow' if key in changed else None)
        for index, position in enumerate(POSITIONS):
            chosen = self.selected == index
            output = self.config['channel_map'][index]
            chip = self.config['pwm_chips'][output - 1]
            pulse = self.pulses[output - 1]
            found = len(glob.glob(chip))
            name = Text()
            name.append(f'{MARK} ' if chosen else '  ', style='bold')
            name.append(position.upper(), style=f'bold {ACCENT}' if chosen else None)
            reversed_ = 'yes' if self.config['reversed'][index] else ('no' if chosen else '-')
            table.add_row(
                name, cell(str(output), 'channel_map', chosen), cell(pin_label(chip), 'pwm_chips', chosen),
                '✓' if found == 1 else '✗', cell(reversed_, 'reversed', chosen),
                Text(f'{pulse} µs', style='bold' if chosen else None),
                Text(' ') + styled_bar(pulse), Text(effort_text(pulse), style='bold' if pulse != NEUTRAL_PWM else 'dim'),
                Text(self.readback(output), style='dim'))
        name = Text()
        name.append(f'{MARK} ' if self.selected == 4 else '  ', style='bold')
        name.append('ALL', style=f'bold {ACCENT}' if self.selected == 4 else 'dim')
        table.add_row(name, '', '', '', '', '', '', '', '')
        return table

    def fields(self):
        """The selected row's editable fields, each with its key: ``FR  t output [2]  r reversed [no]  p pin [...]``."""
        if self.selected == 4:
            return 'ALL selected: Drive keys move every output; select one thruster to edit its wiring'
        output = self.config['channel_map'][self.selected]
        return (f'{POSITIONS[self.selected].upper()} selected:  t output [{output}]  '
                f'r reversed [{"yes" if self.config["reversed"][self.selected] else "no"}]  '
                f'p pin [{pin_label(self.config["pwm_chips"][output - 1])}]')

    def picker_lines(self):
        """The open picker: its title (what is chosen), the column header, one row per option (the cursor
        marked and bold, the current one followed by (current)), then its keys."""
        p = self.picker
        lines = [Text(p['title'], style='bold')]
        if p['header']:
            lines.append(Text('  ' + p['header'], style='dim'))
        for i, (label, _) in enumerate(p['options']):
            chosen = i == p['index']
            row = Text()
            row.append(f'{MARK} ' if chosen else '  ', style='bold')
            row.append(label, style=f'bold {ACCENT}' if chosen else None)
            if i == p['current']:
                row.append(f'  {CURRENT}', style='dim')
            lines.append(row)
        lines.append(key_line(PICKER_KEYS))
        return lines

    def footer(self):
        parts = []
        missing = [pwm_device(chip) for chip in self.config['pwm_chips'] if not glob.glob(chip)]
        if missing and any(missing):
            parts.append(Text(f'not enabled on this host: {", ".join(m for m in missing if m)} '
                              f'(the overlay is {OVERLAY})', style='yellow'))
        parts.append(Text(self.fields(), style='bold'))
        if self.picker is not None:
            parts += [Text('')] + self.picker_lines()
        if self.pending is not None:
            parts += [Text(''), Text(self.pending[1].rstrip()),
                      Text('y saves this to the profile, any other key cancels', style='bold')]
        if self.message:
            parts += [Text(''), Text(self.message, style='bold')]
        if self.picker is None and self.pending is None:     # a picker shows its own keys
            parts += [Text('')] + key_help()
        return Group(*parts)

    def render(self):
        self.refresh()
        return Group(self.header(), Text(''), self.table(), Text(''), self.footer())

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
