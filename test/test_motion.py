"""motion.children: the one arm topology, with no remaps."""
import pytest

from mhseals_hardware.motion import children

pytestmark = pytest.mark.smoke


@pytest.mark.parametrize('velocity,names', [(True, ['hardware', 'velocity', 'mux']),
                                            (False, ['hardware', 'mux'])])
def test_order_and_no_remaps(velocity, names):
    started = children('/s/hardware.yaml', velocity, True, 'topic')
    assert [name for name, _ in started] == names
    for _, argv in started:
        assert '-r' not in argv and argv[:3] == ['ros2', 'run', 'mhseals_hardware']
        assert 'config_file:=/s/hardware.yaml' in argv and 'use_sim_time:=true' in argv
    executables = [argv[3] for _, argv in started]
    assert executables[0] == 'thruster_pwm_node' and executables[-1] == 'command_mux'
    assert ('calibrated_velocity_controller' in executables) is velocity
    assert dict(started)['hardware'][-2:] == ['-p', 'output_backend:=topic']
