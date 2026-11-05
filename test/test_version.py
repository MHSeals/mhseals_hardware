"""The driver's code version is a digest of its sources, computed without a subprocess (#119)."""
import subprocess

import pytest

from mhseals_hardware.version import code_version

pytestmark = pytest.mark.smoke


def test_the_version_digest_follows_the_sources(tmp_path):
    (tmp_path / 'a.py').write_text('x = 1\n')
    first = code_version(tmp_path)['digest']
    code_version.cache_clear()
    (tmp_path / 'a.py').write_text('x = 2\n')
    assert code_version(tmp_path)['digest'] != first


def test_the_version_runs_no_subprocess(tmp_path, monkeypatch):
    def refuse(*args, **kwargs):
        raise AssertionError('code_version must not run a subprocess')
    for name in ('run', 'Popen', 'check_output'):
        monkeypatch.setattr(subprocess, name, refuse)
    (tmp_path / 'a.py').write_text('x = 3\n')
    assert set(code_version(tmp_path)) == {'digest'}
