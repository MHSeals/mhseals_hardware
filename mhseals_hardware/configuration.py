"""Shared, validated hardware configuration; loading never opens GPIO/PWM."""

import array
import contextlib
import copy
import fcntl
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import tempfile
import re

import yaml

from mhseals_hardware import vessel
from mhseals_hardware.resources import config_resource
from mhseals_hardware.command_arbitration import Arbitration, mux_section
from mhseals_hardware.control_validation import controller_section, topics_section
from mhseals_hardware.thruster_mixer import (
    validate_channel_map, validate_decoupling, validate_mixer, validate_reverse_ratio,
    validate_reversed,
)

# Renamed settings: an operator file still naming the old key loads it as the new one with a
# deprecation warning (an operator file on the boat may predate the rename); naming both refuses.
# The next write of the file (update_config) migrates it to the new name.
RENAMED = {'mosfet_heartbeat_hz': 'mosfet_heartbeat'}
# Removed settings an older operator file may still name: dropped on read with one warning, so
# the file still arms. The next write of the file drops them.
# The topic keys are now topics:, the two stage timeouts the one top-level command_timeout.
OBSOLETE = {'controller': ('input_topic', 'output_topic', 'mux_state_topic', 'command_timeout'),
            'command_mux': ('cmd_timeout',)}
SECTIONS = ('controller', 'command_mux', 'topics')  # mappings an operator file sets key by key
WIRING = ('channel_map', 'reversed')    # the only keys a vessel profile's thrusters: may set
log = logging.getLogger(__name__)


def config_path(path=None):
    # An empty variable means unset, not the current directory.
    return Path(path or os.environ.get('MHSEALS_HARDWARE_CONFIG')
                or Path.home() / '.config/mhseals/hardware.yaml').expanduser()


def validate_config(config):
    """Reject a config that could drive the wrong pins or produce unsafe output."""
    config['controller'] = controller_section(config['controller'])
    topics_section(config['topics'])
    # Thruster layout.
    validate_channel_map(config['channel_map'])
    validate_reversed(config['reversed'])
    validate_mixer(config['thruster_matrix'])
    validate_decoupling(config['decoupling'])
    validate_reverse_ratio(config['reverse_thrust_ratio'])
    if config.get('decoupling_negative') is not None:
        validate_decoupling(config['decoupling_negative'])
    epoch = config.get('hardware_epoch', 0)
    if type(epoch) is not int or epoch < 0:
        raise ValueError('hardware_epoch must be an integer >= 0')
    # Transmitter layout and mux timing; the section is built to prove it decodes.
    mux = config.get('command_mux', {})
    if not isinstance(mux, dict):
        raise ValueError('command_mux must be a mapping')
    try:
        Arbitration.from_config(mux_section(config))
    except TypeError as error:
        raise ValueError(f'invalid command_mux: {error}') from error
    # Optional calibration written by mhseals_testing's characterization.
    characterization = config.get('characterization', {})
    if not isinstance(characterization, dict):
        raise ValueError('characterization must be a mapping')
    limits = characterization.get('motion_limits', {})
    if limits and not isinstance(limits, dict):
        raise ValueError('characterization.motion_limits must be a mapping')
    for group in ('velocity', 'acceleration', 'deceleration', 'combined'):
        values = limits.get(group, {})
        if values and (not isinstance(values, dict) or any(
                type(value) not in (int, float) or not math.isfinite(value) or value < 0
                for value in values.values())):
            raise ValueError(f'characterization.motion_limits.{group} must contain finite nonnegative values')
    gain = characterization.get('velocity_gain')
    if gain:
        if not isinstance(gain, dict) or any(axis not in gain for axis in ('surge', 'sway', 'yaw')):
            raise ValueError('characterization.velocity_gain must contain surge, sway, yaw rows')
        for row in gain.values():
            if not isinstance(row, dict) or any(axis not in row for axis in ('surge', 'sway', 'yaw')):
                raise ValueError('characterization.velocity_gain rows must contain surge, sway, yaw')
            if any(type(value) not in (int, float) or not math.isfinite(value)
                   for value in row.values()):
                raise ValueError('characterization.velocity_gain values must be finite')
    curves = characterization.get('speed_curves')
    if curves:
        if not isinstance(curves, dict):
            raise ValueError('characterization.speed_curves must be a mapping')
        for key, curve in curves.items():
            if key not in ('surge+', 'surge-', 'sway+', 'sway-', 'yaw+', 'yaw-'):
                raise ValueError(f'characterization.speed_curves has unknown direction {key}')
            if not isinstance(curve, dict) or any(
                    type(curve.get(term)) not in (int, float) or
                    not math.isfinite(curve[term]) for term in ('d1', 'd2')):
                raise ValueError(f'characterization.speed_curves.{key} needs finite d1 and d2')
    bias = characterization.get('mechanical_bias')
    if bias and (not isinstance(bias, dict) or any(
            axis not in bias or type(bias[axis]) not in (int, float) or
            not math.isfinite(bias[axis]) for axis in ('surge', 'sway', 'yaw'))):
        raise ValueError('characterization.mechanical_bias must contain finite surge, sway, yaw values')
    # PWM outputs and the MOSFET enable line.
    if config['output_backend'] not in ('odroid', 'topic'):
        raise ValueError('output_backend must be odroid or topic')
    if not isinstance(config['pwm_topic'], str) or not config['pwm_topic'].startswith('/'):
        raise ValueError('pwm_topic must be an absolute ROS topic')
    if len(config['pwm_chips']) != 4 or not all(
            isinstance(p, str) and p.startswith('/sys/')
            for p in config['pwm_chips']):
        raise ValueError('pwm_chips must contain four absolute /sys paths')
    if len(set(config['pwm_chips'])) != 4:
        raise ValueError('PWM chip paths must be distinct')
    if len(config['pwm_channels']) != 4 or any(
            type(n) is not int or n < 0 for n in config['pwm_channels']):
        raise ValueError('pwm_channels must contain four nonnegative integers')
    # Restarting PWM at another frequency can look like throttle to the ESCs.
    if config['frequency'] != 50:
        raise ValueError('ESC frequency must be 50 Hz')
    if type(config['mosfet_line']) is not int or config['mosfet_line'] < 0:
        raise ValueError('mosfet_line must be a nonnegative integer')
    if type(config['mosfet_active_high']) is not bool:
        raise ValueError('mosfet_active_high must be a boolean')
    for key in ('estop_cuts_power', 'estop_on_rc_loss'):
        if type(config[key]) is not bool:
            raise ValueError(f'{key} must be a boolean')
    # Under 10 Hz a flip is more than the driver's 0.15 s stall threshold away.
    beat = config['mosfet_heartbeat']
    if type(beat) not in (int, float) or not math.isfinite(beat) or not (
            beat == 0 or 10 <= beat <= 100):
        raise ValueError('mosfet_heartbeat must be 0 (off) or within [10, 100]')
    if not isinstance(config['mosfet_chip'], str) or not re.fullmatch(
            r'(?:/dev/gpiochip\d+|gpio\d+)', config['mosfet_chip']):
        raise ValueError('mosfet_chip must be a bank label (gpio3) or /dev/gpiochipN')
    # Watchdog and keyboard deadman timings.
    for key in ('command_timeout', 'manual_repeat_timeout', 'manual_initial_timeout'):
        if type(config[key]) not in (int, float) or not math.isfinite(config[key]) or not 0 < config[key] <= 2:
            raise ValueError(f'{key} must be finite and within (0, 2] seconds')
    if type(config['manual_amplitude']) not in (int, float) or not math.isfinite(config['manual_amplitude']) or not 0 <= config['manual_amplitude'] <= 1:
        raise ValueError('manual_amplitude must be within [0, 1]')
    if type(config['manual_yaw_scale']) not in (int, float) or not math.isfinite(config['manual_yaw_scale']) or not 0 < config['manual_yaw_scale'] <= 1:
        raise ValueError('manual_yaw_scale must be within (0, 1]')
    return config


def load_defaults():
    """Shipped defaults only; never read per-user overrides at import time."""
    defaults = config_resource('default.yaml')
    return validate_config(load_strict_yaml(defaults))


def load_config(path=None):
    """Defaults < the vessel profile's ``thrusters:`` < the operator YAML (README "Configuration").

    The YAML path is ``path``, else $MHSEALS_HARDWARE_CONFIG, else
    ~/.config/mhseals/hardware.yaml. Unknown keys in either layer are rejected so
    typos fail loudly instead of being ignored.
    """
    selected = config_path(path)
    if not selected.exists() and (path or os.environ.get('MHSEALS_HARDWARE_CONFIG')):
        raise FileNotFoundError(selected)
    return _layered(_read_overrides(selected))


def _layered(overrides):
    """The operator overrides merged over the vessel profile's thrusters: wiring and the defaults."""
    wiring = vessel.section('thrusters')
    extra = sorted(set(wiring) - set(WIRING))
    if extra:
        raise ValueError(f'vessel profile thrusters: takes {", ".join(WIRING)}, not {extra}')
    return _merge_overrides({**copy.deepcopy(wiring), **overrides})


def config_sha(config):
    """Short hash of a loaded config, so the TUI can tell which file a node applied."""
    return hashlib.sha256(json.dumps(config, sort_keys=True, default=str).encode()).hexdigest()[:12]


def load_strict_yaml(path):
    """Parse a YAML file, refusing what could silently change meaning.

    Duplicate keys, anchors/aliases, merge keys, non-scalar keys and non-finite numbers
    raise ValueError naming ``file:line``. An empty file is an empty mapping.
    """
    text = Path(path).read_text()

    def refuse(mark, message):
        raise ValueError(f'{path}:{mark.line + 1}: {message}')

    def check(node):
        if isinstance(node, yaml.MappingNode):
            seen = {}
            for key, value in node.value:
                if not isinstance(key, yaml.ScalarNode) or key.tag == 'tag:yaml.org,2002:merge':
                    refuse(key.start_mark, 'merge and non-scalar keys are not allowed')
                if key.value in seen:
                    refuse(key.start_mark, f'duplicate key {key.value!r} (first at '
                           f'{path}:{seen[key.value].start_mark.line + 1})')
                seen[key.value] = key
                check(value)
        elif isinstance(node, yaml.SequenceNode):
            for item in node.value:
                check(item)
        elif node.tag == 'tag:yaml.org,2002:float' and not math.isfinite(
                yaml.SafeLoader('').construct_yaml_float(node)):
            refuse(node.start_mark, f'non-finite number {node.value}')

    try:
        for event in yaml.parse(text, Loader=yaml.SafeLoader):
            if getattr(event, 'anchor', None):  # anchors and the aliases that use them
                refuse(event.start_mark, 'anchors and aliases are not allowed')
        root = yaml.compose(text, Loader=yaml.SafeLoader)
        if root is None:
            return {}
        check(root)
        data = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise ValueError(f'invalid hardware YAML {path}: {error}') from error
    if not isinstance(data, dict):
        raise ValueError(f'{path}: hardware YAML must be a mapping')
    return data


def _read_overrides(path):
    """The operator file's overrides; every refusal is a ValueError naming the file."""
    if not path.exists():
        return {}
    overrides = load_strict_yaml(path)
    for old, new in RENAMED.items():
        if old not in overrides:
            continue
        if new in overrides:
            raise ValueError(f'{path}: {old} and {new} both set; {old} is the old name of {new}, '
                             f'keep {new}')
        log.warning('%s: %s is deprecated, read as %s (rename it; the next write does)',
                    path, old, new)
        overrides[new] = overrides.pop(old)
    gone = []
    for section, keys in OBSOLETE.items():
        values = overrides.get(section)
        if isinstance(values, dict) and any(key in values for key in keys):
            gone += [f'{section}.{key}' for key in keys if key in values]
            overrides[section] = {k: v for k, v in values.items() if k not in keys}
    if gone:
        log.warning('%s: %s no longer exist; ignored (the next write drops them)', path,
                    ', '.join(gone))
    unknown = sorted(set(overrides) - set(load_defaults()))
    if unknown:
        lines = {key.value: key.start_mark.line + 1 for key, _ in
                 yaml.compose(path.read_text(), Loader=yaml.SafeLoader).value}
        where = ', '.join(f'{path}:{lines[key]}' for key in unknown)
        raise ValueError(f'unknown hardware settings: {unknown} at {where}')
    return overrides


def declared(node, section, names=None):
    """``section`` with each of ``names`` (default: every non-mapping key) as a ROS parameter.

    Each is declared with its YAML value and read back, so a ``-p`` override wins and the file
    gives the rest. A node reads its arm snapshot this way once, at startup; none watches the YAML.
    """
    names = [k for k, v in section.items() if not isinstance(v, dict)] if names is None else names
    for name in names:
        node.declare_parameter(name, section[name])
    return dict(section, **{name: _plain(node.get_parameter(name).value) for name in names})


def _plain(value):
    """ROS array parameters arrive as sequences other than list; YAML and validators want lists."""
    return list(value) if isinstance(value, (tuple, array.array)) else value


def _merge_overrides(overrides):
    defaults = load_defaults()
    unknown = set(overrides) - set(defaults)
    if unknown:
        raise ValueError(f'unknown hardware settings: {sorted(unknown)}')
    combined = dict(defaults, **overrides)
    for name in SECTIONS:
        if name in overrides:
            if not isinstance(overrides[name], dict):
                raise ValueError(f'{name} must be a mapping')
            combined[name] = dict(defaults[name], **overrides[name])
    config = validate_config(combined)
    # ROS array parameters are typed by their elements: ints or a mix would declare the
    # wrong type (or crash), so every matrix is a plain list of floats.
    for key in ('thruster_matrix', 'decoupling', 'decoupling_negative'):
        if config.get(key) is not None:
            config[key] = [float(v) for v in config[key]]
    # A scalar written as 1 where the default is 1.0 would declare an integer parameter that
    # a later float edit cannot change; keep the default's float type.
    for key, default in defaults.items():
        if type(default) is float and type(config[key]) is int:
            config[key] = float(config[key])
    # Unset means "the same as decoupling". Filled in here so the driver always has a
    # concrete, declared parameter; written back out as null while it still matches.
    if config.get('decoupling_negative') is None:
        config['decoupling_negative'] = list(config['decoupling'])
    return config


def _follows(negative, decoupling):
    """True when ``negative`` is unset or equal to ``decoupling``: no real sign-split."""
    return negative is None or [float(v) for v in negative] == [float(v) for v in decoupling]


def save_config(config, path=None):
    """Atomically persist validated settings; never partially replace a file."""
    validate_config(config)
    target = config_path(path)
    normalized = {key: list(value) if isinstance(value, tuple) else value
                  for key, value in config.items()}
    if _follows(normalized.get('decoupling_negative'), normalized['decoupling']):
        normalized['decoupling_negative'] = None  # same as decoupling; don't freeze a copy
    return _write_config(normalized, target)


@contextlib.contextmanager
def _locked(target):
    """Serialize read-modify-write across processes.

    The lock is a sidecar file: the config itself is replaced atomically, so
    locking it would lock a stale inode.
    """
    target.parent.mkdir(parents=True, exist_ok=True)
    with open(target.parent / f'.{target.name}.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def update_config(changes, path=None):
    """Persist only explicit overrides, retaining existing operator settings."""
    target = config_path(path)
    with _locked(target):
        return _update_locked(changes, target)


def _update_locked(changes, target):
    overrides = _read_overrides(target)
    # A negative C equal to decoupling was following it: keep following, not split.
    if _follows(overrides.get('decoupling_negative'), _merge_overrides(overrides)['decoupling']):
        overrides.pop('decoupling_negative', None)
    overrides.update(changes)
    if _follows(overrides.get('decoupling_negative'), _merge_overrides(overrides)['decoupling']):
        overrides.pop('decoupling_negative', None)
    _layered(overrides)
    return _write_config(overrides, target)


class _NoAliasDumper(yaml.SafeDumper):
    """Never emit anchors or aliases, which the strict loader refuses."""

    def ignore_aliases(self, data):
        return True


def _config_yaml(normalized):
    """Keep flat mixer/correction sequences grouped by their three-column rows."""
    matrices = {'thruster_matrix': 12, 'decoupling': 9, 'decoupling_negative': 9}
    # Give matrices independent sequences so other fields cannot alias their anchors.
    serializable = {key: list(value) if key in matrices and value is not None else value
                    for key, value in normalized.items()}
    text = yaml.dump(serializable, Dumper=_NoAliasDumper, sort_keys=False)
    replacements = []
    for key, value in yaml.compose(text, Loader=yaml.SafeLoader).value:
        if key.value not in matrices or not isinstance(value, yaml.SequenceNode):
            continue
        if len(value.value) != matrices[key.value]:
            continue  # Validation remains the caller's responsibility.
        scalars = [text[item.start_mark.index:item.end_mark.index] for item in value.value]
        rows = ['  ' + ', '.join(scalars[i:i + 3]) + ','
                for i in range(0, len(scalars), 3)]
        replacement = key.value + ': [\n' + '\n'.join(rows) + '\n]\n'
        replacements.append((key.start_mark.index, value.end_mark.index, replacement))
    for start, end, replacement in reversed(replacements):
        text = text[:start] + replacement + text[end:]
    return text


def _write_config(normalized, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.hardware-', dir=target.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(_config_yaml(normalized))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return target


def parse_csv(text, cast=str):
    """Parse a comma-separated CLI value such as '0,0,0,0' into a tuple."""
    return tuple(cast(value.strip()) for value in text.split(','))


def _plain(value):
    return list(value) if isinstance(value, tuple) else value


def configure_args(parser, args=None):
    """Apply shared defaults before parsing explicit CLI overrides.

    Precedence is shipped defaults, then the hardware YAML, then CLI flags whose
    names match config keys (for example --pwm-chips sets pwm_chips). The merged
    result is validated and returned as ``parsed.hardware_config``.
    """
    parser.add_argument('--config', help='hardware YAML (or MHSEALS_HARDWARE_CONFIG)')
    parser.add_argument('--save-config', action='store_true',
                        help='persist pin/mixer overrides before opening hardware')
    preliminary, _ = parser.parse_known_args(args)
    config = load_config(preliminary.config)
    parser.set_defaults(**config)
    parsed = parser.parse_args(args)
    changes = {}
    for key in config:
        if hasattr(parsed, key):
            value = getattr(parsed, key)
            if _plain(value) != _plain(config[key]):
                changes[key] = _plain(value)
            config[key] = value
    validate_config(config)
    if parsed.save_config:
        update_config(changes, parsed.config)  # only the flags given, not every default
    parsed.hardware_config = config
    return parsed
