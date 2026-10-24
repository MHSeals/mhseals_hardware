"""Pure tests against real package-owned default and descriptor resources."""
import copy
import hashlib
from importlib.resources import files

import pytest
import yaml
from mhseals_config.catalog import DescriptorBundle
from mhseals_hardware.catalog import REGISTRY, load_catalog
from mhseals_hardware.configuration import load_defaults, _merge_overrides
from mhseals_hardware.resources import config_resource


def test_installed_resources_and_inventory():
    bundle = load_catalog()
    defaults = load_defaults()
    assert len(bundle.rows) == len(defaults) - 1 + len(defaults['controller'])
    for row in bundle.rows:
        key = row.descriptor['canonical_key'].removeprefix('hardware.')
        value = row.default
        assert (list(value) if isinstance(value, tuple) else value) == (defaults['controller'][key.split('.')[1]]
                if key.startswith('controller.') else defaults[key])
        assert not row.writable
    timeout = bundle.field('hardware.command_timeout')
    assert timeout.default == defaults['command_timeout']
    assert 'unresolved-consumer-version:thruster_pwm_node' in timeout.blockers
    with pytest.raises(ValueError):
        bundle.field('hardware.command_mux.unknown')


def test_nondefault_resource_reference_and_unknown_consumers():
    data = yaml.safe_load(config_resource('catalog.yaml').read_text())
    defaults = yaml.safe_load(config_resource('default.yaml').read_text())
    defaults['command_timeout'] = 0.73
    raw = yaml.safe_dump(defaults).encode()
    digest = hashlib.sha256(raw).hexdigest()
    data['default_sources'][0]['sha256'] = digest
    for leaf in data['leaves']:
        leaf['default_ref']['source_hash'] = digest
    bundle = DescriptorBundle.read(data, {'hardware-default': raw}, REGISTRY, {})
    assert bundle.field('hardware.command_timeout').default == 0.73
    assert all(not row.writable for row in bundle.rows)
    assert bundle.field('hardware.characterization').blockers
    timeout = bundle.field('hardware.command_timeout').descriptor
    assert [c['id'] for c in timeout['consumers']] == ['thruster_pwm_node']


@pytest.mark.parametrize('key,value', [
    ('command_timeout', True), ('command_timeout', float('nan')),
    ('pwm_channels', [True, 0, 0, 0]), ('channel_map', [1, 1, 3, 4]),
    ('reversed', [0, False, False, False]), ('decoupling', [1.0]),
])
def test_actual_validator_rejects_unsafe_closure(key, value):
    config = copy.deepcopy(load_defaults())
    config[key] = value
    with pytest.raises(ValueError):
        REGISTRY['hardware.validate_config'](config)


def test_null_negative_tracks_nondefault_positive():
    positive = [2.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    normalized = _merge_overrides({'decoupling': positive, 'decoupling_negative': None})
    assert normalized['decoupling_negative'] == positive
    assert load_catalog().field('hardware.decoupling_negative').descriptor[
        'authoring']['null'] == 'follows-controlling-key'


def test_candidate_tiers_follow_actual_restart_policy():
    # Read constant declarations without importing ROS nodes or opening outputs.
    import ast

    def constants(module, names, bindings=None):
        found = dict(bindings or {})

        def value(node):
            if isinstance(node, ast.Name):
                return found[node.id]
            if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
                return value(node.left) + value(node.right)
            return ast.literal_eval(node)

        source = files('mhseals_hardware').joinpath(module + '.py').read_text()
        for node in ast.parse(source).body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id in names:
                        found[target.id] = value(node.value)
        return found

    outputs = constants('outputs', {'OUTPUT_KEYS'})
    driver = constants('thruster_pwm_node', {'RESTART_KEYS', 'LIVE_KEYS'}, outputs)
    controller = constants('velocity_controller', {'PLANT_KEYS'})
    bundle = load_catalog()
    for row in bundle.rows:
        key = row.descriptor['canonical_key'].removeprefix('hardware.')
        for consumer in row.descriptor.get('consumers', ()):
            if consumer['id'] == 'thruster_pwm_node':
                assert key in driver['RESTART_KEYS'] + driver['LIVE_KEYS']
                expected = 'unsupported' if key in driver['RESTART_KEYS'] else 1
            else:
                assert consumer['id'] == 'velocity_controller'
                assert key in controller['PLANT_KEYS'] or key.startswith('controller.')
                expected = 'unsupported'
            assert consumer['candidate_tier'] == expected
            assert not row.writable


def test_controller_catalog_is_complete_independent_and_read_only():
    defaults = load_defaults()['controller']
    bundle = load_catalog()
    rows = [row for row in bundle.rows if row.descriptor['canonical_key'].startswith('hardware.controller.')]
    assert len(rows) == len(defaults) == 11
    for row in rows:
        descriptor = row.descriptor
        key = descriptor['canonical_key'].split('.')[-1]
        assert descriptor['default_ref']['yaml_pointer'] == '/controller/' + key
        assert descriptor['aliases'] == ()
        assert descriptor['coverage'][0]['status'] == 'startup-supported'
        assert descriptor['consumers'][0]['candidate_tier'] == 'unsupported'
        assert not row.writable
    assert bundle.field('hardware.command_timeout').descriptor['consumers'][0]['id'] == 'thruster_pwm_node'
    assert bundle.field('hardware.controller.command_timeout').descriptor['consumers'][0]['id'] == 'velocity_controller'
    with pytest.raises(ValueError):
        REGISTRY['hardware.controller_section'](dict(defaults, feedback=1))


def test_default_pointers_include_current_source_lines_and_flat_matrices():
    from mhseals_hardware.resources import config_resource
    import re
    text = config_resource('default.yaml').read_text()
    locations = {}

    def walk(node, prefix=''):
        if isinstance(node, yaml.MappingNode):
            for key, value in node.value:
                pointer = prefix + '/' + key.value
                locations[pointer] = key.start_mark.line + 1
                walk(value, pointer)
    walk(yaml.compose(text))
    for row in load_catalog().rows:
        ref = row.descriptor['default_ref']
        assert ref['line'] == locations[ref['yaml_pointer']]
    for key, rows in (('thruster_matrix', 4), ('decoupling', 3)):
        block = re.search(key + r': \[\n(.*?)\n\]', text, re.S).group(1)
        assert len(block.splitlines()) == rows
        assert all(len(line.split('#')[0].strip().rstrip(',').split(',')) == 3
                   for line in block.splitlines())
        value = yaml.safe_load(text)[key]
        assert len(value) == rows * 3 and all(type(v) is float for v in value)
