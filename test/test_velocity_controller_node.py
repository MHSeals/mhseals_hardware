"""The controller reads its arm snapshot once; a YAML edit waits for the next arm, gains change live."""

import pytest

rclpy = pytest.importorskip('rclpy')

from mhseals_hardware.configuration import save_config
from mhseals_hardware.velocity_controller import VelocityController
from test_velocity_law import config

STRONGER = ((0.8, 0.0, 0.0), (0.0, 0.4, 0.0), (0.05, 0.0, 0.3))


def test_startup_refuses_a_drift_limit_at_the_sway_limit(tmp_path):
    path = tmp_path / 'hardware.yaml'
    base = config()
    save_config(dict(base, controller=dict(base['controller'], drift_limit=0.4)), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=velocity_drift_limit_test',
                     '-p', f'config_file:={path}'])
    try:
        with pytest.raises(ValueError, match='drift_limit'):
            VelocityController()
    finally:
        rclpy.shutdown()


def test_a_yaml_edit_keeps_the_running_law_and_gains_apply_live(tmp_path):
    path = tmp_path / 'hardware.yaml'
    save_config(config(), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=velocity_snapshot_test',
                     '-p', f'config_file:={path}', '-p', 'odom_timeout:=0.7'])
    node = None
    try:
        node = VelocityController()
        assert node._law.odom_timeout == 0.7  # a -p override wins over the YAML
        law, curves = node._law, dict(node._law.plant.curves)
        node._law.drift[:] = (0.1, -0.2)
        save_config(config(gain=STRONGER), path)  # the next arm's model
        assert not hasattr(node, '_reload') and node._law is law
        assert node._law.plant.curves == curves
        from rclpy.parameter import Parameter
        assert node.set_parameters([Parameter('kp', value=[1., 1., 1.])])[0].successful
        assert list(node._law.kp) == [1., 1., 1.] and list(node._law.drift) == [0.1, -0.2]
        assert not node.set_parameters([Parameter('kp', value=[1., 2.])])[0].successful
        assert list(node._law.kp) == [1., 1., 1.]
        result = node.set_parameters([Parameter('command_timeout', value=1.0)])[0]
        assert not result.successful and 'next arm' in result.reason
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
