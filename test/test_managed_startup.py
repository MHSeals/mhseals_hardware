"""Owner union adoption uses hardware evidence and checks all foreign artifacts."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

from mhseals_config import Artifact, BaselineRevision, digest
from mhseals_config._bootstrap import begin_bootstrap, finish_bootstrap, read_bootstrap_inputs
from mhseals_config._evidence import Recipe
from mhseals_hardware.startup_config import (
    EVALUATORS, node_startup_inputs, observe_inputs, prepare_inputs, startup_inputs,
)
import pytest


def union(root):
    prepared = prepare_inputs(session=('output_backend: topic\n', 'profile outputs'))
    foreign = {'nav2-baseline': {'value': 1}, 'nav2-compiler': {'value': 2}}
    documents = dict(prepared['documents'], **foreign)
    recipes = dict(prepared['recipes'], **{p: 'foreign/v1' for p in foreign})
    identities = dict(prepared['identities'], **{p: 'foreign-code/v1' for p in foreign})
    artifacts = prepared['baseline_artifacts'] + (
        Artifact('nav2-baseline.json', digest(foreign['nav2-baseline'])),)
    baseline = BaselineRevision(digest([a.sha256 for a in artifacts]), artifacts)
    foreign_recipe = Recipe('foreign/v1', lambda document, evidence: evidence == document)
    evaluators = dict(EVALUATORS, **{'foreign/v1': foreign_recipe})
    ticket = begin_bootstrap(root, 'union', baseline, documents, recipes,
                             source_identity='aggregate', participant_identities=identities,
                             evaluators=evaluators, closure='inert')
    captured = read_bootstrap_inputs(root, ticket)
    observations = observe_inputs(captured.documents, ticket, 'aggregate')
    observations.update({p: {'scope': 'union', 'candidate': ticket.candidate,
                             'source': 'aggregate', 'identity': identities[p],
                             'recipe': recipes[p], 'evidence': document}
                         for p, document in foreign.items()})
    assert finish_bootstrap(root, ticket, observations,
                            evaluators=evaluators)['status'] == 'verified_startup'
    return ticket


def constructor(root, candidate, calls):
    class Node:
        def __init__(self, name):
            self._parameter_overrides = {}
            self.parameters = {'config_root': str(root), 'config_scope': 'union',
                               'config_candidate': candidate}

        def declare_parameter(self, name, value):
            self.parameters.setdefault(name, value)

        def get_parameter(self, name):
            return NS(value=self.parameters[name])

        def create_publisher(self, *args):
            return NS()

        def apply(self, config):
            calls.append(('config', config['output_backend']))

    path = Path(__file__).parents[1] / 'mhseals_hardware/thruster_pwm_node.py'
    cls = next(n for n in ast.parse(path.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'ThrusterPWMNode')
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__']
    # Stop at the output boundary after recording the actual constructor request.

    def outputs(config, **kwargs):
        calls.append(('outputs', config['output_backend']))
        raise RuntimeError('inert output boundary')
    from mhseals_hardware.configuration import validate_config
    import uuid
    namespace = {'Node': Node, 'node_startup_inputs': node_startup_inputs,
                 'validate_config': validate_config, 'uuid': uuid,
                 'String': NS, 'STATUS_QOS': 1, 'make_outputs': outputs}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                 str(path), 'exec'), namespace)
    return namespace['ThrusterPWMNode']


@pytest.mark.parametrize('fault', [None, 'foreign', 'identity', 'missing'])
def test_actual_managed_constructor(tmp_path, fault):
    root = tmp_path / 'snapshots'
    ticket = union(root)
    if fault == 'foreign':
        (root / 'union/initializing/nav2-compiler.json').write_text('{}')
    if fault == 'missing':
        (root / 'union/active.json').unlink()
    before = sorted(str(p) for p in root.rglob('*'))
    calls = []
    candidate = 'wrong' if fault == 'identity' else ticket.candidate
    with pytest.raises((RuntimeError, ValueError, FileNotFoundError)):
        constructor(root, candidate, calls)()
    assert calls == ([('config', 'topic'), ('outputs', 'topic')] if fault is None else [])
    assert sorted(str(p) for p in root.rglob('*')) == before


def test_managed_missing_scope_never_bootstraps(tmp_path):
    with pytest.raises(FileNotFoundError):
        startup_inputs(root=tmp_path, scope='missing', config_candidate='required')
    assert not (tmp_path / 'missing/initializing').exists()
