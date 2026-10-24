"""
Pure hardware startup compilation and non-actuating observation recipes.

The recipe proves Python mixer/model consumption only, never powered outputs.
"""
from collections.abc import Mapping
from dataclasses import asdict, replace
import math
from pathlib import Path

from mhseals_config._evidence import Recipe
from mhseals_config.resolver import Field, parse, resolve
from mhseals_hardware.resources import config_resource
from mhseals_hardware.configuration import _merge_overrides, load_defaults
from mhseals_hardware.control_validation import controller_section
from mhseals_hardware.thruster_mixer import mix_thrusters, validate_mixer
from mhseals_hardware.velocity_law import Plant, VelocityLaw

RECIPE_ID = 'hardware-pure-startup/v2'
PARTICIPANTS = frozenset(('hardware-baseline', 'hardware-pure'))
SELECTION_PARAMETERS = ('config_root', 'config_scope', 'config_profile', 'config_candidate')


def plain(value):
    """Copy immutable resolver values to domain-owned JSON documents."""
    if isinstance(value, Mapping):
        return {key: plain(child) for key, child in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(child) for child in value]
    return value


def parse_layer(text, source, layer, fields):
    """Accept integer YAML spelling for float settings without accepting booleans."""
    import yaml

    numeric_fields = dict(fields)
    root = yaml.compose(text)
    def walk(node, prefix='', ancestors=frozenset()):
        # Preprocessing must leave malformed graphs to the located strict parser.
        if id(node) in ancestors:
            return
        ancestors = ancestors | {id(node)}
        if isinstance(node, yaml.MappingNode):
            for key, value in node.value:
                if key.tag != 'tag:yaml.org,2002:str':
                    continue
                name = prefix + key.value
                if (name in fields and fields[name].kind is float
                        and value.tag == 'tag:yaml.org,2002:int'):
                    numeric_fields[name] = replace(fields[name], kind=int)
                elif name not in fields and any(
                        field.startswith(name + '.') for field in fields):
                    walk(value, name + '.', ancestors)
    walk(root)
    assignments = parse(text, source, layer, numeric_fields)
    return tuple(replace(item, value=float(item.value))
                 if fields[item.key].kind is float else item for item in assignments)


def hardware_fields(defaults=None):
    """Return the closed hardware schema: one Field per default leaf (controller flattened)."""
    defaults = defaults or load_defaults()
    fields = {key: Field(type(value), nullable=value is None)
              for key, value in defaults.items()}
    fields.pop('controller')
    fields.update({'controller.' + key: Field(type(value))
                   for key, value in defaults['controller'].items()})
    # The nullable negative matrix is a sequence when explicitly supplied.
    fields['decoupling_negative'] = Field(list, nullable=True)
    return fields


def compile_inputs(operator=None, *, profile=None, session=None):
    """Compile B separately from S with located package/profile/operator origins."""
    defaults_path = config_resource('default.yaml')
    defaults = load_defaults()
    fields = hardware_fields(defaults)
    layers = [parse_layer(defaults_path.read_text(), str(defaults_path), 'defaults', fields)]
    for layer, selected in (('profile', profile), ('operator', operator)):
        if selected is not None:
            path = Path(selected)
            layers.append(parse_layer(path.read_text(), str(path), layer, fields))
    if session is not None:
        text, source = session
        layers.append(parse_layer(text, source, 'session', fields))
    resolved = resolve(layers)
    def nested(values):
        values = plain(values)
        values['controller'] = {key: values.pop('controller.' + key)
                                for key in defaults['controller']}
        return _merge_overrides(values)
    return dict(baseline=nested(resolved.baseline),  # noqa: C408
                effective=nested(resolved.effective),
                origins={key: [asdict(origin) for origin in origins]
                         for key, origins in resolved.origins.items()})


def observe(document):
    """Evaluate the actual pure mixer and, when calibrated, velocity law."""
    config = document['effective']
    controller = controller_section(config.get('controller'))  # incomplete legacy documents must refuse
    mixer = validate_mixer(config['thruster_matrix'])
    evidence = {'mixer': [list(mix_thrusters(*command, mixer=mixer))
                          for command in ((0.25, 0, 0), (0, 0.25, 0), (0, 0, 0.25))],
                'timeout': config['command_timeout']}
    if config.get('characterization'):
        law = VelocityLaw(Plant(config), **{key: value for key, value in controller.items()
                          if key not in ('feedback', 'command_timeout', 'input_topic',
                                         'output_topic', 'odom_topic')})
        law.observe((0, 0, 0), 0.0, 0.0)
        evidence['law'] = list(law.step((0.1, 0, 0), 0.0, feedback=controller['feedback']))
    return evidence


def evaluate(document, evidence):
    """Independently check bounded probe expectations from the compiled inputs."""
    config = document['effective']
    controller = controller_section(config.get('controller'))
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
            if controller['feedback']:
                effort += controller['kp'][len(expected['law'])] * min(target, limit)
            expected['law'].append(max(-1.0, min(1.0, effort)))

    def matches(actual, wanted):
        if isinstance(wanted, dict):
            return (isinstance(actual, dict) and actual.keys() == wanted.keys()
                    and all(matches(actual[key], value) for key, value in wanted.items()))
        if isinstance(wanted, list):
            return (isinstance(actual, (list, tuple)) and len(actual) == len(wanted)
                    and all(matches(a, b) for a, b in zip(actual, wanted)))
        return (type(actual) in (int, float) and math.isfinite(actual)
                and math.isclose(actual, wanted, rel_tol=1e-12, abs_tol=1e-14))

    return matches(evidence, expected)


EVALUATORS = {RECIPE_ID: Recipe(RECIPE_ID, evaluate)}


def prepare_inputs(operator=None, *, profile=None, session=None, document=None):
    """Prepare the fixed hardware closure without reserving or writing a scope."""
    from mhseals_config import Artifact, BaselineRevision, digest

    document = document or compile_inputs(operator, profile=profile, session=session)
    baseline_document = {'baseline': document['baseline'], 'effective': document['baseline'],
                         'origins': {k: [o for o in origins if o['layer'] != 'session']
                                     for k, origins in document['origins'].items()}}
    baseline_hash = digest(baseline_document)
    artifacts = (Artifact('hardware-baseline.json', baseline_hash),)
    return {'documents': {'hardware-pure': document, 'hardware-baseline': baseline_document},
            'recipes': {p: RECIPE_ID for p in PARTICIPANTS},
            'identities': {p: RECIPE_ID for p in PARTICIPANTS},
            'baseline_artifacts': artifacts,
            'baseline': BaselineRevision(baseline_hash, artifacts),
            'source': digest(document['origins'])}


def observe_inputs(documents, ticket, source):
    """Bind actual pure observations to the owner's captured aggregate identity."""
    return {p: {'scope': ticket.scope, 'candidate': ticket.candidate, 'source': source,
                'identity': RECIPE_ID, 'recipe': RECIPE_ID,
                'evidence': observe(plain(documents[p]))} for p in PARTICIPANTS}


def startup_inputs(path=None, *, root=None, scope=None, profile=None, session=None,
                   config_candidate=None):
    """
    Select an immutable scope, or bootstrap a new pure pre-output snapshot.

    A pure receipt authorizes no output operation; node lifecycle remains owner.
    """
    import os
    import uuid
    from mhseals_config._bootstrap import (
        NoActiveSelection, begin_bootstrap, finish_bootstrap,
        read_bootstrap_inputs, read_selected_inputs, read_selected_subset,
    )
    from mhseals_hardware.configuration import config_path

    root = root or os.environ.get('MHSEALS_CONFIG_ROOT') or (
        Path(os.environ.get('MHSEALS_DATA', str(Path.home() / '.local/share/mhseals')))
        / 'config-startup')
    if config_candidate and not (scope or os.environ.get('MHSEALS_CONFIG_SCOPE')):
        raise ValueError('managed startup requires explicit scope')
    scope = scope or os.environ.get('MHSEALS_CONFIG_SCOPE') or uuid.uuid4().hex
    try:
        selected = read_selected_subset(
            root, scope, PARTICIPANTS, evaluators=EVALUATORS,
            expected_candidate=config_candidate)
    except NoActiveSelection:
        if config_candidate:
            raise
        selected_path = config_path(path)
        explicit = path or os.environ.get('MHSEALS_HARDWARE_CONFIG')
        operator = selected_path if explicit or selected_path.exists() else None
        document = compile_inputs(operator, profile=profile, session=session)
        prepared = prepare_inputs(document=document)
        ticket = begin_bootstrap(
            root, scope, prepared['baseline'], prepared['documents'], prepared['recipes'],
            source_identity=prepared['source'], participant_identities=prepared['identities'],
            closure='inert', evaluators=EVALUATORS)
        pending = read_bootstrap_inputs(root, ticket)
        observations = observe_inputs(pending.documents, ticket, pending.candidate['source'])
        receipt = finish_bootstrap(root, ticket, observations,
                                   evaluators=EVALUATORS)
        if receipt['status'] != 'verified_startup':
            raise ValueError('hardware pure startup observation failed')
        selected = read_selected_inputs(root, scope, evaluators=EVALUATORS)
    document = plain(selected.documents['hardware-pure'])
    # Revalidate captured bytes, never reread mutable authoring inputs here.
    for version in ('baseline', 'effective'):
        controller_section(document[version].get('controller'))
    _merge_overrides(document['baseline'])
    _merge_overrides(document['effective'])
    return document, selected


def node_startup_inputs(node, path, *, hardware_parameters=True):
    """Resolve the existing node startup before its output lifecycle begins."""
    import yaml

    for name in SELECTION_PARAMETERS:
        node.declare_parameter(name, '')
    invocation = {}
    if hardware_parameters:
        for key in load_defaults():
            parameter = node._parameter_overrides.get(key)
            if parameter is not None:
                invocation[key] = parameter.value
    else:
        for key in load_defaults()['controller']:
            parameter = node._parameter_overrides.get(key)
            if parameter is not None:
                value = parameter.value
                invocation.setdefault('controller', {})[key] = (
                    list(value) if key in ('kp', 'ki', 'integral_limit')
                    and not isinstance(value, (str, bytes)) else value)
    value = lambda name: node.get_parameter(name).value  # noqa: E731
    document, selected = startup_inputs(
        path, root=value('config_root') or None, scope=value('config_scope') or None,
        profile=value('config_profile') or None,
        config_candidate=value('config_candidate') or None,
        session=(yaml.safe_dump(invocation), 'ROS invocation') if invocation else None)
    if not hardware_parameters and invocation:
        captured = document['effective']['controller']
        requested = controller_section(dict(captured, **invocation['controller']))
        if requested != captured:
            raise ValueError('controller invocation conflicts with immutable startup selection')
    return document['effective'], selected
