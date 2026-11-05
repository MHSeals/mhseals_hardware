import pytest

from mhseals_hardware import vessel
from ros_isolation import isolate_ros_env


def pytest_configure(config):
    """Tests that start nodes never reach the live boat: private domain, no static peers, localhost only."""
    isolate_ros_env()


@pytest.fixture(autouse=True)
def _private_runtime_dir(tmp_path, monkeypatch):
    """Keep tests off the real command_mux hold file of a live mux."""
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path / 'xdg'))


@pytest.fixture(autouse=True)
def _no_vessel(monkeypatch):
    """No vessel profile unless a test names one, so a developer's vessel env never flips load_config."""
    monkeypatch.delenv('ASTRO_NETWORK_ENV', raising=False)
    monkeypatch.delenv('ROS_WS', raising=False)
    vessel.section.cache_clear()
    yield
    vessel.section.cache_clear()
