import pytest

from ros_isolation import isolate_ros_env


def pytest_configure(config):
    """Tests that start nodes never reach the live boat: private domain, no static peers, localhost only."""
    isolate_ros_env()


@pytest.fixture(autouse=True)
def _private_runtime_dir(tmp_path, monkeypatch):
    """Keep tests off the real command_mux hold file of a live mux."""
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path / 'xdg'))
