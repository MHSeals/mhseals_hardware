"""Shared, validated hardware configuration; loading never opens GPIO/PWM."""

import contextlib
import fcntl
import math
import os
from pathlib import Path
import tempfile
import re

import yaml

from mhseals_hardware.thruster_mixer import (
    validate_channel_map, validate_decoupling, validate_mixer, validate_reversed,
)


def config_path(path=None):
    # An empty variable means unset, not the current directory.
    return Path(path or os.environ.get('MHSEALS_HARDWARE_CONFIG')
                or Path.home() / '.config/mhseals/hardware.yaml').expanduser()


def validate_config(config):
    """Reject a config that could drive the wrong pins or produce unsafe output."""
    # Thruster layout.
    validate_channel_map(config['channel_map'])
    validate_reversed(config['reversed'])
    validate_mixer(config['thruster_matrix'])
    validate_decoupling(config['decoupling'])
    if config.get('decoupling_negative') is not None:
        validate_decoupling(config['decoupling_negative'])
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
                not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0
                for value in values.values())):
            raise ValueError(f'characterization.motion_limits.{group} must contain finite nonnegative values')
    gain = characterization.get('velocity_gain')
    if gain:
        if not isinstance(gain, dict) or any(axis not in gain for axis in ('surge', 'sway', 'yaw')):
            raise ValueError('characterization.velocity_gain must contain surge, sway, yaw rows')
        for row in gain.values():
            if not isinstance(row, dict) or any(axis not in row for axis in ('surge', 'sway', 'yaw')):
                raise ValueError('characterization.velocity_gain rows must contain surge, sway, yaw')
            if any(not isinstance(value, (int, float)) or not math.isfinite(value)
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
                    not isinstance(curve.get(term), (int, float)) or
                    not math.isfinite(curve[term]) for term in ('d1', 'd2')):
                raise ValueError(f'characterization.speed_curves.{key} needs finite d1 and d2')
    bias = characterization.get('mechanical_bias')
    if bias and (not isinstance(bias, dict) or any(
            axis not in bias or not isinstance(bias[axis], (int, float)) or
            not math.isfinite(bias[axis]) for axis in ('surge', 'sway', 'yaw'))):
        raise ValueError('characterization.mechanical_bias must contain finite surge, sway, yaw values')
    for key in ('fitted_with_decoupling', 'fitted_with_decoupling_negative'):
        if characterization.get(key) is not None:
            validate_decoupling(characterization[key])
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
    if not isinstance(config['mosfet_chip'], str) or not re.fullmatch(
            r'(?:/dev/gpiochip\d+|gpio\d+)', config['mosfet_chip']):
        raise ValueError('mosfet_chip must be a bank label (gpio3) or /dev/gpiochipN')
    # Watchdog and keyboard deadman timings.
    for key in ('command_timeout', 'manual_repeat_timeout', 'manual_initial_timeout'):
        if not math.isfinite(config[key]) or not 0 < config[key] <= 2:
            raise ValueError(f'{key} must be finite and within (0, 2] seconds')
    if not math.isfinite(config['manual_amplitude']) or not 0 <= config['manual_amplitude'] <= 1:
        raise ValueError('manual_amplitude must be within [0, 1]')
    if not math.isfinite(config['manual_yaw_scale']) or not 0 < config['manual_yaw_scale'] <= 1:
        raise ValueError('manual_yaw_scale must be within (0, 1]')
    return config


def load_defaults():
    """Shipped defaults only; never read per-user overrides at import time."""
    defaults = Path(__file__).with_name('config') / 'default.yaml'
    return validate_config(yaml.safe_load(defaults.read_text()))


def load_config(path=None):
    """Shipped defaults overlaid with the user's hardware YAML, if present.

    The YAML path is ``path``, else $MHSEALS_HARDWARE_CONFIG, else
    ~/.config/mhseals/hardware.yaml. Unknown keys are rejected so typos fail
    loudly instead of being ignored.
    """
    selected = config_path(path)
    if not selected.exists() and (path or os.environ.get('MHSEALS_HARDWARE_CONFIG')):
        raise FileNotFoundError(selected)
    return _merge_overrides(_read_overrides(selected))


class ConfigWatcher:
    """Reload the hardware YAML once an edit has settled.

    A change is parsed only when the file's (mtime, size, inode) is the same on two
    consecutive polls, so an in-place save caught mid-write is never applied as a
    partial file (whose missing keys would fall back to the defaults).
    """

    def __init__(self, path):
        self.path = path
        self.mtime = self.seen = self._stat()  # applied key, and last tick's key

    def _stat(self):
        try:
            st = os.stat(config_path(self.path))
        except OSError:
            return None
        return st.st_mtime_ns, st.st_size, st.st_ino

    def poll(self):
        """The new config after a settled edit, else None; raises OSError/ValueError/KeyError on a bad one.

        YAML syntax errors and wrong-typed values surface as ValueError, so a typo in a
        live edit is rejected by the callers instead of killing the node.
        """
        key, previous, self.seen = self._stat(), self.seen, self._stat()
        if key is None or key == self.mtime or key != previous:
            return None
        self.mtime = key
        try:
            return load_config(self.path)
        except (yaml.YAMLError, TypeError) as error:
            raise ValueError(f'invalid hardware YAML: {error}') from error


def _read_overrides(path):
    overrides = yaml.safe_load(path.read_text()) if path.exists() else {}
    if not isinstance(overrides, dict):
        raise ValueError('hardware YAML must be a mapping')
    return overrides


def _merge_overrides(overrides):
    defaults = load_defaults()
    unknown = set(overrides) - set(defaults)
    if unknown:
        raise ValueError(f'unknown hardware settings: {sorted(unknown)}')
    config = validate_config(dict(defaults, **overrides))
    # ROS array parameters are typed by their elements: ints or a mix would declare the
    # wrong type (or crash), so every matrix is a plain list of floats.
    for key in ('thruster_matrix', 'decoupling', 'decoupling_negative'):
        if config.get(key) is not None:
            config[key] = [float(v) for v in config[key]]
    # Unset means "the same as decoupling". Filled in here so the driver always has a
    # concrete, declared parameter; written back out as null while it still matches.
    if config.get('decoupling_negative') is None:
        config['decoupling_negative'] = list(config['decoupling'])
    return config


def _follows(negative, decoupling):
    """True when ``negative`` is unset or equal to ``decoupling``: no real sign-split."""
    return negative is None or [float(v) for v in negative] == [float(v) for v in decoupling]


def single_decoupling(config):
    """The one linear C, for code that models C (fits, the velocity law).

    Refuses a sign-split decoupling: inverting only the positive columns would
    silently mislabel a direction-dependent correction as one linear gain.
    """
    if not _follows(config.get('decoupling_negative'), config['decoupling']):
        raise ValueError('sign-split decoupling is active (decoupling_negative differs); linear fits '
                         'and the velocity law model one C. Clear it before characterizing.')
    return config['decoupling']


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
    _merge_overrides(overrides)
    return _write_config(overrides, target)


def _write_config(normalized, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.hardware-', dir=target.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            yaml.safe_dump(normalized, stream, sort_keys=False)
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
    for key in config:
        if hasattr(parsed, key):
            config[key] = getattr(parsed, key)
    validate_config(config)
    if parsed.save_config:
        save_config(config, parsed.config)
    parsed.hardware_config = config
    return parsed
