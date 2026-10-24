"""Pure hardware startup compilation and non-actuating observation recipes.

The recipe proves Python mixer/model consumption only, never powered outputs.
"""
import math
from dataclasses import asdict
from pathlib import Path
from collections.abc import Mapping

from mhseals_config.resolver import Field, parse, resolve
from mhseals_config._evidence import Recipe
from mhseals_hardware.configuration import _merge_overrides, load_defaults
from mhseals_hardware.thruster_mixer import mix_thrusters, validate_mixer
from mhseals_hardware.velocity_law import Plant, VelocityLaw

RECIPE_ID = 'hardware-pure-startup/v1'


def plain(value):
    """Copy immutable resolver values to domain-owned JSON documents."""
    if isinstance(value, Mapping):
        return {key: plain(child) for key, child in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(child) for child in value]
    return value


def compile_inputs(operator=None, *, profile=None, session=None):
    """Compile B separately from S with located package/profile/operator origins."""
    defaults_path = Path(__file__).with_name('config') / 'default.yaml'
    defaults = load_defaults()
    fields = {key: Field(type(value), nullable=value is None)
              for key, value in defaults.items()}
    # The nullable negative matrix is a sequence when explicitly supplied.
    fields['decoupling_negative'] = Field(list, nullable=True)
    layers = [parse(defaults_path.read_text(), str(defaults_path), 'defaults', fields)]
    for layer, selected in (('profile', profile), ('operator', operator)):
        if selected is not None:
            path = Path(selected)
            layers.append(parse(path.read_text(), str(path), layer, fields))
    if session is not None:
        text, source = session
        layers.append(parse(text, source, 'session', fields))
    resolved = resolve(layers)
    return dict(baseline=_merge_overrides(plain(resolved.baseline)),
                effective=_merge_overrides(plain(resolved.effective)),
                origins={key: [asdict(origin) for origin in origins]
                         for key, origins in resolved.origins.items()})


def observe(document):
    """Evaluate the actual pure mixer and, when calibrated, velocity law."""
    config = document['effective']
    mixer = validate_mixer(config['thruster_matrix'])
    evidence = {'mixer': [list(mix_thrusters(*command, mixer=mixer))
                          for command in ((0.25, 0, 0), (0, 0.25, 0), (0, 0, 0.25))],
                'timeout': config['command_timeout']}
    if config.get('characterization'):
        law = VelocityLaw(Plant(config), kp=(0, 0, 0), ki=(0, 0, 0),
                          drift_time_constant=0)
        law.observe((0, 0, 0), 0.0, 0.0)
        evidence['law'] = list(law.step((0.1, 0, 0), 0.0))
    return evidence


def evaluate(document, evidence):
    """Independently check bounded probe expectations from the compiled inputs."""
    config = document['effective']
    expected = {'mixer': [[0.25 * config['thruster_matrix'][row * 3 + axis]
                           for row in range(4)] for axis in range(3)],
                'timeout': config['command_timeout']}
    # Model evidence is optional only for configurations without calibration.
    if config.get('characterization'):
        characterization = config['characterization']
        expected['law'] = []
        for axis, target in zip(('surge', 'sway', 'yaw'), (0.1, 0.0, 0.0)):
            limit = characterization['motion_limits']['velocity'][axis]
            speed = min(target, limit) - characterization.get('mechanical_bias', {}).get(axis, 0)
            direction = axis + ('-' if speed < 0 else '+')
            curve = characterization.get('speed_curves', {}).get(direction)
            if curve is None:
                gain = characterization['velocity_gain'][axis][axis]
                wrench = speed * abs(speed) / gain ** 2
            else:
                wrench = curve['d1'] * speed + curve['d2'] * speed * abs(speed)
            effort = math.copysign(math.sqrt(abs(wrench)), wrench)
            expected['law'].append(max(-1.0, min(1.0, effort)))
    return evidence == expected


EVALUATORS = {RECIPE_ID: Recipe(RECIPE_ID, evaluate)}
