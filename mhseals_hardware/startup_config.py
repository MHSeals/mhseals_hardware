"""
Pure hardware startup compilation and non-actuating observation recipes.

The recipe proves Python mixer/model consumption only, never powered outputs.
"""
from collections.abc import Mapping
from dataclasses import asdict
import math
from pathlib import Path

from mhseals_config._evidence import Recipe
from mhseals_config.resolver import Field, parse, resolve
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
    return dict(baseline=_merge_overrides(plain(resolved.baseline)),  # noqa: C408
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
    for row in expected['mixer']:
        peak = max(1.0, *(abs(value) for value in row))
        row[:] = [value / peak for value in row]
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


def startup_inputs(path=None, *, root=None, scope=None, profile=None, session=None):
    """
    Select an immutable scope, or bootstrap a new pure pre-output snapshot.

    A pure receipt authorizes no output operation; node lifecycle remains owner.
    """
    import os
    import uuid
    from mhseals_config import Artifact, BaselineRevision, digest
    from mhseals_config._bootstrap import (
        NoActiveSelection, begin_bootstrap, finish_bootstrap,
        read_bootstrap_inputs, read_selected_inputs,
    )
    from mhseals_hardware.configuration import config_path

    root = root or os.environ.get('MHSEALS_CONFIG_ROOT') or (
        Path(os.environ.get('MHSEALS_DATA', str(Path.home() / '.local/share/mhseals')))
        / 'config-startup')
    scope = scope or os.environ.get('MHSEALS_CONFIG_SCOPE') or uuid.uuid4().hex
    try:
        selected = read_selected_inputs(root, scope, evaluators=EVALUATORS)
    except NoActiveSelection:
        selected_path = config_path(path)
        explicit = path or os.environ.get('MHSEALS_HARDWARE_CONFIG')
        operator = selected_path if explicit or selected_path.exists() else None
        document = compile_inputs(operator, profile=profile, session=session)
        baseline_document = dict(baseline=document['baseline'],  # noqa: C408
                                 effective=document['baseline'],
                                 origins={k: [o for o in origins if o['layer'] != 'session']
                                          for k, origins in document['origins'].items()})
        baseline_hash = digest(baseline_document)
        baseline = BaselineRevision(baseline_hash, (
            Artifact('hardware-baseline.json', baseline_hash),))
        source = digest(document['origins'])
        documents = {'hardware-pure': document, 'hardware-baseline': baseline_document}
        identities = {participant: RECIPE_ID for participant in documents}
        ticket = begin_bootstrap(
            root, scope, baseline, documents, identities, source_identity=source,
            participant_identities=identities, closure='inert', evaluators=EVALUATORS)
        pending = read_bootstrap_inputs(root, ticket)
        observations = {
            participant: dict(scope=scope, candidate=ticket.candidate, source=source,  # noqa: C408
                              identity=RECIPE_ID, recipe=RECIPE_ID,
                              evidence=observe(plain(captured)))
            for participant, captured in pending.documents.items()}
        receipt = finish_bootstrap(root, ticket, observations,
                                   evaluators=EVALUATORS)
        if receipt['status'] != 'verified_startup':
            raise ValueError('hardware pure startup observation failed')
        selected = read_selected_inputs(root, scope, evaluators=EVALUATORS)
    document = plain(selected.documents['hardware-pure'])
    # Revalidate captured bytes, never reread mutable authoring inputs here.
    _merge_overrides(document['baseline'])
    _merge_overrides(document['effective'])
    return document, selected


def node_startup_inputs(node, path, *, hardware_parameters=True):
    """Resolve the existing node startup before its output lifecycle begins."""
    import yaml

    for name in ('config_root', 'config_scope', 'config_profile'):
        node.declare_parameter(name, '')
    invocation = {}
    if hardware_parameters:
        for key in load_defaults():
            parameter = node._parameter_overrides.get(key)
            if parameter is not None:
                invocation[key] = parameter.value
    value = lambda name: node.get_parameter(name).value  # noqa: E731
    document, selected = startup_inputs(
        path, root=value('config_root') or None, scope=value('config_scope') or None,
        profile=value('config_profile') or None,
        session=(yaml.safe_dump(invocation), 'ROS invocation') if invocation else None)
    return document['effective'], selected
