"""run_node exits quietly on the spin-versus-shutdown race but not on real errors."""

import pytest

pytest.importorskip('rclpy')
from mhseals_hardware import ros_util  # noqa: E402


class FakeNode:
    def __init__(self):
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


def run(monkeypatch, error, context_ok):
    node = FakeNode()

    def spin(_node):
        raise error

    monkeypatch.setattr(ros_util.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(ros_util.rclpy, 'spin', spin)
    monkeypatch.setattr(ros_util.rclpy, 'ok', lambda: context_ok)
    monkeypatch.setattr(ros_util.rclpy, 'try_shutdown', lambda: None)
    try:
        ros_util.run_node(lambda: node)
    finally:
        assert node.destroyed


def test_shutdown_race_is_quiet(monkeypatch):
    run(monkeypatch, RuntimeError('Unable to convert call argument 0 to Python object'), False)


def test_runtime_error_with_live_context_propagates(monkeypatch):
    with pytest.raises(RuntimeError):
        run(monkeypatch, RuntimeError('boom'), True)
