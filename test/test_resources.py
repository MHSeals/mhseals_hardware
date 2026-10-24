"""Source and installed data bindings are explicit, ROS-free and fail closed."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from mhseals_hardware import resources


def test_source_resources_ignore_cwd_and_refuse_missing(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.chdir(tmp_path)
    assert resources.config_resource('default.yaml') == root / 'config/default.yaml'
    linked = tmp_path / 'linked-package'
    linked.symlink_to(root / 'mhseals_hardware', target_is_directory=True)
    monkeypatch.setattr(resources, '__file__', str(linked / 'resources.py'))
    assert resources.config_resource('default.yaml') == root / 'config/default.yaml'
    fake = tmp_path / 'source'
    (fake / 'mhseals_hardware').mkdir(parents=True)
    (fake / 'resource').mkdir()
    (fake / 'package.xml').touch()
    (fake / 'resource/mhseals_hardware').touch()
    monkeypatch.setattr(resources, '__file__', str(fake / 'mhseals_hardware/resources.py'))
    with pytest.raises(FileNotFoundError, match='required hardware configuration'):
        resources.config_resource('default.yaml')


def test_copied_colcon_resources_bind_code_prefix(tmp_path, monkeypatch):
    prefix = tmp_path / 'install/mhseals_hardware'
    package = prefix / 'lib/python3.14/site-packages/mhseals_hardware'
    package.mkdir(parents=True)
    shared = prefix / 'share/mhseals_hardware/config'
    shared.mkdir(parents=True)
    expected = shared / 'default.yaml'
    expected.write_text('owned: true')
    monkeypatch.setattr(resources, '__file__', str(package / 'resources.py'))
    assert resources.config_resource('default.yaml') == expected
    with pytest.raises(FileNotFoundError, match='catalog.yaml'):
        resources.config_resource('catalog.yaml')


def test_installed_wheel_loads_own_data_from_unrelated_cwd(tmp_path):
    root = Path(__file__).resolve().parents[1]
    source = tmp_path / 'build-source'
    source.mkdir()
    for name in ('mhseals_hardware', 'config', 'resource'):
        shutil.copytree(root / name, source / name)
    for name in ('setup.py', 'setup.cfg', 'package.xml'):
        shutil.copy2(root / name, source / name)
    # Build/install only this wheel; dependencies are already source-pinned by the gate.
    subprocess.run([sys.executable, 'setup.py', 'bdist_wheel'], cwd=source,
                   check=True, capture_output=True)
    wheel = next((source / 'dist').glob('*.whl'))
    target = tmp_path / 'installed'
    subprocess.run([sys.executable, '-m', 'pip', 'install', '--no-deps', '--no-index',
                    '--target', str(target), str(wheel)], check=True, capture_output=True)
    for name in ('default.yaml', 'catalog.yaml'):
        assert (target / 'share/mhseals_hardware/config' / name).read_bytes() == (root / 'config' / name).read_bytes()
    env = dict(os.environ)
    # Exclude the checkout from subprocess imports, keeping explicit core dependency roots.
    dependency_paths = [str(Path(p).resolve()) for p in env.get('PYTHONPATH', '').split(os.pathsep)
                        if p and Path(p).resolve() != root]
    env['PYTHONPATH'] = os.pathsep.join([str(target), *dependency_paths])
    code = '''
from pathlib import Path
import sys
from mhseals_hardware.resources import config_resource
from mhseals_hardware.configuration import load_defaults
from mhseals_hardware.catalog import load_catalog
from mhseals_hardware.startup_config import compile_inputs, observe, evaluate
assert config_resource('default.yaml') == Path(sys.argv[1]) / 'share/mhseals_hardware/config/default.yaml'
assert not (Path(sys.argv[1]) / 'mhseals_hardware/config').exists()
assert load_defaults()['controller']['kp'] == [0.8, 0.8, 0.6]
assert len(load_catalog().rows) == 32
compiled = compile_inputs()
evidence = observe(compiled)
assert evidence['mixer'][0] == [-.25, -.25, .25, .25]
assert evaluate(compiled, evidence)
assert 'rclpy' not in sys.modules
'''
    subprocess.run([sys.executable, '-c', code, str(target)], cwd=tmp_path,
                   env=env, check=True, capture_output=True)
    (target / 'share/mhseals_hardware/config/default.yaml').unlink()
    refusal = subprocess.run([sys.executable, '-c',
        'from mhseals_hardware.configuration import load_defaults; load_defaults()'],
        cwd=tmp_path, env=env, capture_output=True, text=True)
    assert refusal.returncode != 0
    assert 'required hardware configuration resource missing' in refusal.stderr
