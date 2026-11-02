"""C set over the real SetParameters service reaches the pulses and reads back through GetParameters."""
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from mhseals_hardware.configuration import load_defaults, save_config
from mhseals_hardware import thruster_pwm_node
from mhseals_hardware.thruster_mixer import thruster_pulses, validate_decoupling

pytestmark = pytest.mark.smoke

CORRECTION = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.1, 0.0, 1.0]


class Outputs:
    def open(self):
        return self

    def close(self):
        pass

    def set_pulse_widths(self, pulses):
        self.pulses = pulses


def call(nodes, client, request):
    assert client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(request)
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    executor.spin_until_future_complete(future, timeout_sec=5.0)
    executor.shutdown()
    return future.result()


def test_c_over_the_service_applies_reads_back_and_refuses_bad(tmp_path, monkeypatch):
    outputs = Outputs()
    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda *a, **k: outputs)
    path = tmp_path / 'hardware.yaml'
    save_config(load_defaults(), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=live_c_test', '-p', f'config_file:={path}'])
    node = caller = None
    try:
        node = thruster_pwm_node.ThrusterPWMNode()
        caller = rclpy.create_node('live_c_caller')
        setter = caller.create_client(SetParameters, '/live_c_test/set_parameters')
        getter = caller.create_client(GetParameters, '/live_c_test/get_parameters')

        def set_c(values):
            request = SetParameters.Request(parameters=[
                Parameter('decoupling', Parameter.Type.DOUBLE_ARRAY, values).to_parameter_msg()])
            return call((node, caller), setter, request).results[0]

        def read_back():
            reply = call((node, caller), getter, GetParameters.Request(names=['decoupling', 'decoupling_negative']))
            return [list(value.double_array_value) for value in reply.values]

        assert set_c(CORRECTION).successful
        assert read_back() == [CORRECTION, CORRECTION]
        message = Twist()
        message.linear.x = .5
        node.cmd_vel_callback(message)
        assert outputs.pulses == thruster_pulses((.5, 0., 0.), node.mixer, (1, 2, 3, 4),
                                                 decoupling=validate_decoupling(CORRECTION))
        bad = set_c([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 9.0, 0.0, 1.0])
        assert not bad.successful and 'within' in bad.reason
        assert read_back() == [CORRECTION, CORRECTION]
        assert node.config['decoupling'] == CORRECTION
    finally:
        if caller is not None:
            caller.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
