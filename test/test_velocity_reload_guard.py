"""Actual watcher/law behavior with no ROS transport or output construction."""
import copy
import threading
from types import SimpleNamespace as NS

import pytest

from mhseals_hardware.velocity_law import Plant, VelocityLaw
from test_pwm_reload_guard import extract, pwm, request
from test_velocity_law import config


def controller(fault=''):
    accepted = config()
    accepted['decoupling_negative'] = list(accepted['decoupling'])
    cls, _ = extract('velocity_controller.py', 'VelocityController',
                     Plant=lambda *a: pytest.fail('must not rebuild'),
                     time=NS(monotonic=lambda: 1.0), twist_values=lambda msg: msg,
                     twist_from=lambda values: tuple(values))
    node = cls()
    node._law_lock = threading.RLock()
    node._law = VelocityLaw(Plant(accepted))
    node._plant_inputs = node._inputs(accepted)
    node._requested_plant_inputs, node._reload_error = node._plant_inputs, ''
    node._fault, node.logs, node.sent = fault, [], []
    node.get_logger = lambda: NS(error=node.logs.append)
    node.get_parameter = lambda name: NS(value=False)
    node._pub = NS(publish=node.sent.append)
    return node, accepted


def reload(node, candidate):
    node._watch = NS(poll=lambda: candidate)
    node._reload()


@pytest.mark.parametrize('change', ['decoupling', 'decoupling_negative', 'thruster_matrix',
                                   'velocity_gain', 'speed_curves', 'mechanical_bias',
                                   'motion_limits', 'metadata'])
@pytest.mark.parametrize('fault', ['', 'existing fault'])
def test_plant_edits_preserve_model_and_state(change, fault):
    node, accepted = controller(fault)
    candidate = copy.deepcopy(accepted)
    if change in ('decoupling', 'decoupling_negative', 'thruster_matrix'):
        candidate[change] = [v * 0.8 for v in candidate[change]]
    elif change == 'velocity_gain':
        candidate['characterization'][change]['surge']['surge'] *= 2
    else:
        candidate['characterization'][change] = {
            'speed_curves': {'surge+': {'d1': 0.1, 'd2': 2.0}},
            'mechanical_bias': {'surge': 0.1, 'sway': 0, 'yaw': 0},
            'motion_limits': {'velocity': {'surge': 0.7, 'sway': 0.4, 'yaw': 0.3}},
            'metadata': {'requested': 1}}[change]
    law, plant, inputs = node._law, node._law.plant, node._plant_inputs
    law.integral[:] = 0.2
    law.drift[:] = (0.1, -0.2)
    law.reset = lambda: pytest.fail('must not reset')
    reload(node, candidate)
    assert node._law is law and node._law.plant is plant and node._plant_inputs is inputs
    assert list(law.integral) == [0.2] * 3 and list(law.drift) == [0.1, -0.2]
    assert node._fault == fault and not node.sent
    assert node._requested_plant_inputs == node._inputs(candidate)
    assert node._reload_error.startswith('restart required:') and node.logs
    reload(node, None)
    assert node._reload_error.startswith('restart required:')
    reload(node, dict(accepted, pwm_channels=[1, 0, 0, 0], command_timeout=1))
    assert node._reload_error == '' and node._fault == fault
    assert node._law is law and node._law.plant is plant
    assert node._requested_plant_inputs == inputs


def test_coupled_request_is_refused_by_both_consumers_and_effort_stays_old():
    node, accepted = controller()
    driver, _ = pwm()
    driver.config = driver.file_config = driver.requested_config = accepted
    driver.apply(accepted)
    driver.apply = driver.sync_parameters = lambda *a: pytest.fail('PWM must refuse')
    candidate = copy.deepcopy(accepted)
    candidate['characterization']['velocity_gain']['surge']['surge'] *= 2
    candidate['decoupling'][0] = 0.8
    node._law.observe((0, 0, 0), 0, 1.0)
    node._command((0.12, 0, 0))
    before = node.sent[-1]
    assert before == pytest.approx((0.2, 0, 0))
    assert Plant(candidate).effort(0, 0.12) == pytest.approx(0.1)  # real challenge differs
    request(driver, candidate)
    reload(node, candidate)
    node._command((0.12, 0, 0))
    assert node.sent[-1] == pytest.approx(before)
    assert driver.pending_restart_keys == ['characterization', 'decoupling']
    assert node._reload_error == 'restart required: characterization, decoupling'
    reload(node, accepted)
    assert not node._reload_error and node._plant_inputs == node._requested_plant_inputs


def test_startup_selection_parameters_are_read_only():
    from mhseals_hardware.startup_config import SELECTION_PARAMETERS
    node, _ = controller()
    for name in SELECTION_PARAMETERS:
        assert not node._validate_gains([NS(name=name, value='changed')]).successful
