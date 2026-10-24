"""Host installer checks use a logging sudo stub, never privileged operations."""
import os
from pathlib import Path
import subprocess


def test_host_installer_sources_and_syntax(tmp_path):
    root = Path(__file__).resolve().parents[1]
    script = root / 'scripts/install_odroid_access.sh'
    subprocess.run(['bash', '-n', str(script)], check=True)
    subprocess.run(['bash', '-n', str(root / 'host/mhseals-thruster-access')], check=True)
    stub = tmp_path / 'sudo'
    stub.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$INSTALL_LOG"\n')
    stub.chmod(0o755)
    log = tmp_path / 'calls'
    env = dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.environ['PATH'],
               INSTALL_LOG=str(log))
    subprocess.run(['bash', str(script)], cwd=tmp_path, env=env, check=True,
                   capture_output=True)
    calls = log.read_text()
    for name in ('mhseals-thruster-access', 'mhseals-thruster-access.service'):
        assert str(root / 'scripts/../host' / name) in calls
        assert (root / 'host' / name).is_file()
    assert not (root / 'config/mhseals-thruster-access').exists()
