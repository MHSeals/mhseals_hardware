"""Center-of-mass trim for keyboard driving (temporary, until a test measures it).

A CoM off the centreline turns surge thrust into yaw, and a CoM off the middle turns sway thrust into
yaw (docs/CONTROL.md L3). Both are a yaw entry of C: C[yaw, surge] and C[yaw, sway]. The trim moves a CoM
marker in steps, and each step sets C = identity + those two entries live on the driver, with C- the
same (a CoM offset needs the same entry forward and reverse). The first step resets every other entry to
identity. Nothing measures the offset, so a step is the yaw effort per unit of surge (or sway) effort
the boat is given against the drift, not metres.

Axes follow REP 103 (+y port, +yaw CCW). With the CoM to starboard, forward thrust yaws the boat to
starboard (CW), so a starboard marker adds +yaw per surge; with the CoM forward, port sway yaws it CW,
so a forward marker adds +yaw per sway.
"""

import datetime
import os
from pathlib import Path

import yaml

DRIVER = '/thruster_pwm_node'
STEP = 0.01     # yaw effort per unit surge/sway effort per key press: on the order of 1 cm of CoM offset
LIMIT = 15      # steps each way (0.15): a larger correction is not a CoM trim
YAW_SURGE, YAW_SWAY = 6, 7      # C[yaw, surge], C[yaw, sway] in the flat row-major 3x3
IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# key -> (starboard steps, forward steps); c centres the marker (C back to identity).
KEYS = {'l': (1, 0), 'j': (-1, 0), 'i': (0, 1), 'k': (0, -1), 'c': None}
GRID = (7, 3)   # picture half-width and half-height in cells; a marker past the edge sits on it


class Trim:
    """The marker in steps, starboard and forward of the centre, and whether the driver needs it."""

    def __init__(self, step=STEP, limit=LIMIT):
        self.step, self.limit = step, limit
        self.right = self.fwd = 0
        self.touched = self.dirty = False
        self.status = ''

    def key(self, name):
        """Move for a trim key; whether ``name`` was one."""
        if name not in KEYS:
            return False
        move = KEYS[name]
        if move is None:
            self.right = self.fwd = 0
        else:
            self.right = max(-self.limit, min(self.limit, self.right + move[0]))
            self.fwd = max(-self.limit, min(self.limit, self.fwd + move[1]))
        self.touched = self.dirty = True
        return True

    def matrix(self):
        """C (and C-) for the marker: identity plus the two yaw entries."""
        values = list(IDENTITY)
        values[YAW_SURGE] = round(self.right * self.step, 6)
        values[YAW_SWAY] = round(self.fwd * self.step, 6)
        return values

    def lines(self, before=None):
        """The panel's trim block: the boat with the marker, the entries and the driver's answer."""
        width, height = GRID
        rows = []
        for y in range(height, -height - 1, -1):
            cells = []
            for x in range(-width, width + 1):
                at = (max(-width, min(width, self.right)) == x and max(-height, min(height, self.fwd)) == y)
                cells.append('[bold cyan]●[/]' if at else '+' if x == y == 0 else '·')
            rows.append('   ' + ' '.join(cells) + ('   bow' if y == height else ''))
        side = 'stbd' if self.right > 0 else 'port' if self.right < 0 else ''
        end = 'fwd' if self.fwd > 0 else 'aft' if self.fwd < 0 else ''
        c = self.matrix()
        head = (f'[bold]CoM trim[/] {abs(self.right)} {side or "centre"}, {abs(self.fwd)} {end or "middle"}'
                f'   C\\[yaw,surge] {c[YAW_SURGE]:+.2f}  C\\[yaw,sway] {c[YAW_SWAY]:+.2f}  (C- the same)')
        keys = ('[dim]J/L port/stbd, I/K fwd/aft (one step a press, '
                f'{self.step:g} each, at most {self.limit}), C centre; '
                + ('the first press resets C and C- to identity' if not self.touched else self.status) + '[/]')
        was = f'\n[dim]C before: {fmt(before)}[/]' if before is not None else ''
        return '\n'.join([head, *rows, keys]) + was


class TrimKeys:
    """J/L/I/K/C move the trim marker (a press, or a plain key; repeats and releases are dropped, so a
    held key moves one step); every other key passes through untouched."""

    def __init__(self, keys, trim):
        self.keys, self.trim = keys, trim

    def __getattr__(self, name):
        return getattr(self.keys, name)

    def read(self, timeout=None):
        key = self.keys.read(timeout=timeout)
        while key is not None:
            kind, _, name = key.rpartition(':') if ':' in key else ('', '', key)
            name = name.lower() if len(name) == 1 else name
            if name not in KEYS:
                return key
            if kind in ('press', ''):
                self.trim.key(name)
            key = self.keys.read(timeout=0)
        return None


def fmt(values):
    return '[' + ', '.join(f'{v:g}' for v in values) + ']'


def result(trim, before=None, now=None):
    """A result `astro apply PATH` writes into the state file (backup, check, undo) and sets live."""
    now = now or datetime.datetime.now()
    c = trim.matrix()
    return {
        'test': 'com-trim', 'layer': 'L3 decoupling', 'status': 'done',
        'date': now.date().isoformat(), 'time': f'{now:%H:%M:%S}',
        'summary': f'keyboard CoM trim: {trim.right:+d} stbd, {trim.fwd:+d} fwd steps of {trim.step:g}',
        'before': before,
        'changes': [{'key': 'decoupling', 'value': c}, {'key': 'decoupling_negative', 'value': c}],
        'live': [{'node': DRIVER, 'param': 'decoupling', 'key': 'decoupling'},
                 {'node': DRIVER, 'param': 'decoupling_negative', 'key': 'decoupling_negative'}],
    }


def save(data, root=None, now=None):
    """Write ``data`` under <root>/<date>/results/ (root: $ASTRO_DATA, else astro's default); its path."""
    now = now or datetime.datetime.now()
    root = Path(root or os.environ.get('ASTRO_DATA') or Path.home() / '.local/state/astro')
    directory = root / now.date().isoformat() / 'results'
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f'{data["test"]}-{now:%H%M%S}.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False, width=110))
    return path
