"""The mux state contract: each consumer reads only ``input`` (command_mux's docstring).

The controller sits before the mux, on the auto lane: it drives only while the mux input is
auto, and any other input, or an unknown state, resets it to neutral. The driver takes test
pulses only while the input is idle.
"""
import json
import time
from types import SimpleNamespace as NS

import pytest

from mhseals_hardware.velocity_law import Plant, VelocityLaw
from test_controller_parameters import node
from test_velocity_law import config

STATES = {
    'idle': {'schema': 'mhseals.mux/1', 'input': 'idle'},
    'auto': {'schema': 'mhseals.mux/1', 'input': 'auto'},
    'rc': {'schema': 'mhseals.mux/1', 'input': 'rc'},
    'manual': {'schema': 'mhseals.mux/1', 'input': 'manual'},
    'hold': {'schema': 'mhseals.mux/1', 'input': 'hold'},
    'bad schema': {'schema': 'mhseals.mux/2', 'input': 'idle'},
    'missing': {'schema': 'mhseals.mux/1'},
}
# state: (driver takes pulses, controller drives)
TABLE = {'idle': (True, False), 'auto': (False, True), 'rc': (False, False),
         'manual': (False, False), 'hold': (False, False), 'bad schema': (False, False),
         'missing': (False, False)}


def message(name):
    return NS(data=json.dumps(STATES[name]))


def pushing():
    """A driving controller holding still against a current, with the integral in use."""
    n = node()
    n._mux_state(message('auto'))
    n._law.drift[:] = (0.0, 0.1)
    n._law.integral[:] = (0.1, -0.2, 0.05)
    n._law.observe((0., 0., 0.), 0., 1.)
    n._command((0., 0., 0.))
    assert any(n.sent[-1])  # station-keeping pushes
    return n


@pytest.mark.parametrize('name', TABLE)
def test_the_controller_drives_only_while_the_mux_is_auto(name):
    n, plain = pushing(), pushing()
    sent = len(n.sent)
    n._mux_state(message(name))
    assert len(n.sent) == sent  # a zero here would refresh a stale auto lane
    n._command((.12, 0., 0.))
    plain._command((.12, 0., 0.))
    if TABLE[name][1]:
        assert n.sent[-1] == pytest.approx(plain.sent[-1]) and any(n.sent[-1])
    else:
        assert n.sent[-1] == (0., 0., 0.)  # neutral on each frame
        assert not n._law.integral.any() and not n._law.effort.any()
        assert n._law.drift == pytest.approx((0.0, 0.1))  # the current estimate is kept


@pytest.mark.parametrize('name', TABLE)
def test_the_driver_takes_pulses_only_while_the_mux_is_idle(name):
    pytest.importorskip('rclpy')
    from mhseals_hardware.thruster_pwm_node import ThrusterPWMNode
    driver = NS(test=None, guard_topic='/mux', mux_time=0.0)
    driver.pulse_refusal = lambda: ThrusterPWMNode.pulse_refusal(driver)
    ThrusterPWMNode.guard_callback(driver, message(name))
    assert (driver.pulse_refusal() is None) is TABLE[name][0]
    driver.mux_time = time.monotonic() - 10  # a dead mux is unknown too
    assert driver.pulse_refusal() is not None


def test_a_controller_started_before_the_mux_is_neutral():
    n = node()
    n._held = True  # the constructor's start: no mux state yet
    n._command((.12, 0., 0.))
    assert n.sent[-1] == (0., 0., 0.)


def test_release_to_auto_resumes_with_no_stale_integral():
    n = pushing()
    n._mux_state(message('hold'))
    n._mux_state(message('auto'))
    n._command((.12, 0., 0.))
    fresh = VelocityLaw(Plant(config()))
    fresh.drift[:] = (0.0, 0.1)
    fresh.observe((0., 0., 0.), 0., 1.)
    assert n.sent[-1] == pytest.approx(fresh.step((.12, 0., 0.), 1.))


def test_the_command_timeout_does_not_keep_the_auto_lane_fresh():
    n = pushing()
    sent = len(n.sent)
    n._last_command = -10.0
    n._watchdog()
    assert len(n.sent) == sent and n._law.last_step is None  # reset, nothing published
