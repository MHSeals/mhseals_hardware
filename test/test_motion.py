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


def test_readme_starts_motion_nodes_by_hand_only_in_sim():
    """Arming is the only starter: the README hand-starts no motion node except the sim driver."""
    from pathlib import Path
    readme = (Path(__file__).resolve().parents[1] / 'README.md').read_text()
    for line in readme.splitlines():
        if any(f'ros2 run mhseals_hardware {name}' in line for name in
               ('thruster_pwm_node', 'command_mux', 'calibrated_velocity_controller')):
            assert 'output_backend:=topic' in line, line
