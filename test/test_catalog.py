"""Pure tests against real package-owned default and descriptor resources."""
import copy
import hashlib
from importlib.resources import files

import pytest
import yaml
from mhseals_config.catalog import DescriptorBundle
from mhseals_hardware.catalog import REGISTRY, load_catalog
from mhseals_hardware.configuration import load_defaults, _merge_overrides


def test_installed_resources_and_inventory():
    bundle = load_catalog()
    defaults = load_defaults()
    assert len(bundle.rows) == len(defaults)
    for row in bundle.rows:
        key = row.descriptor['canonical_key'].removeprefix('hardware.')
        value = row.default
        assert (list(value) if isinstance(value, tuple) else value) == defaults[key]
        assert not row.writable
    timeout = bundle.field('hardware.command_timeout')
    assert timeout.default == defaults['command_timeout']
    assert 'unresolved-consumer-version:thruster_pwm_node' in timeout.blockers
    with pytest.raises(ValueError):
        bundle.field('hardware.command_mux.unknown')


def test_nondefault_resource_reference_and_unknown_consumers():
    config = files('mhseals_hardware').joinpath('config')
    data = yaml.safe_load(config.joinpath('catalog.yaml').read_text())
    defaults = yaml.safe_load(config.joinpath('default.yaml').read_text())
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
                assert key in controller['PLANT_KEYS']
                expected = 'unsupported'
            assert consumer['candidate_tier'] == expected
            assert not row.writable
