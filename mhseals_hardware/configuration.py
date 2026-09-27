"""Shared, validated hardware configuration; loading never opens GPIO/PWM."""

import math
import os
from pathlib import Path
import tempfile

import yaml

from mhseals_hardware.thruster_mixer import validate_channel_map, validate_mixer


def config_path(path=None):
    return Path(path or os.environ.get(
        'MHSEALS_HARDWARE_CONFIG',
        str(Path.home() / '.config/mhseals/hardware.yaml'))).expanduser()


def validate_config(config):
    validate_channel_map(config['channel_map'])
    validate_mixer(config['thruster_matrix'])
    if len(config['pwm_chips']) != 4 or not all(
            isinstance(p, str) and p.startswith('/sys/')
            for p in config['pwm_chips']):
        raise ValueError('pwm_chips must contain four absolute /sys paths')
    if len(set(config['pwm_chips'])) != 4:
        raise ValueError('PWM chip paths must be distinct')
    if len(config['pwm_channels']) != 4 or any(
            type(n) is not int or n < 0 for n in config['pwm_channels']):
        raise ValueError('pwm_channels must contain four nonnegative integers')
    if config['frequency'] != 50:
        raise ValueError('ESC frequency must be 50 Hz')
    if type(config['mosfet_line']) is not int or config['mosfet_line'] < 0:
        raise ValueError('mosfet_line must be a nonnegative integer')
    if type(config['mosfet_active_high']) is not bool:
        raise ValueError('mosfet_active_high must be a boolean')
    if not isinstance(config['mosfet_chip'], str) or not config['mosfet_chip'].startswith('/dev/gpiochip'):
        raise ValueError('mosfet_chip must be an absolute gpiochip device path')
    for key in ('command_timeout', 'manual_repeat_timeout', 'manual_initial_timeout'):
        if not math.isfinite(config[key]) or not 0 < config[key] <= 2:
            raise ValueError(f'{key} must be finite and within (0, 2] seconds')
    if not math.isfinite(config['manual_amplitude']) or not 0 <= config['manual_amplitude'] <= 1:
        raise ValueError('manual_amplitude must be within [0, 1]')
    return config


def load_config(path=None):
    defaults = Path(__file__).with_name('config') / 'default.yaml'
    config = yaml.safe_load(defaults.read_text())
    selected = config_path(path)
    if selected.exists():
        overrides = yaml.safe_load(selected.read_text())
        if not isinstance(overrides, dict):
            raise ValueError('hardware YAML must be a mapping')
        unknown = set(overrides) - set(config)
        if unknown:
            raise ValueError(f'unknown hardware settings: {sorted(unknown)}')
        config.update(overrides)
    elif path or os.environ.get('MHSEALS_HARDWARE_CONFIG'):
        raise FileNotFoundError(selected)
    return validate_config(config)


def save_config(config, path=None):
    """Atomically persist validated settings; never partially replace a file."""
    validate_config(config)
    target = config_path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    normalized = {key: list(value) if isinstance(value, tuple) else value
                  for key, value in config.items()}
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


def configure_args(parser, args=None):
    """Apply shared defaults before parsing explicit CLI overrides."""
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
