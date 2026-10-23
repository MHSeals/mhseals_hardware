import pytest


@pytest.fixture(autouse=True)
def _private_runtime_dir(tmp_path, monkeypatch):
    """Keep tests off the real command_mux hold file of a live mux."""
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path / 'xdg'))
