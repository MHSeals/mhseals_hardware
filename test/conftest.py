import pytest


@pytest.fixture(autouse=True)
def _no_vessel(monkeypatch):
    """No vessel profile unless a test names one, so a developer's vessel env never flips load_config."""
    monkeypatch.delenv('ASTRO_NETWORK_ENV', raising=False)
    monkeypatch.delenv('ROS_WS', raising=False)
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
