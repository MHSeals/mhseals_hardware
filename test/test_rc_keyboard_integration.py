"""RC (Cube Orange /mavros/rc/in) and the keyboard (/cmd_vel/manual) at once, end to end.

The real command_mux and thruster_pwm_node run in one executor; the driver drives fake
sysfs PWM and a fake gpiod MOSFET line (test_estop). The velocity controller is not in
this chain (profiles without it remap the driver to /cmd_vel/applied, as here).
"""

import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist  # noqa: E402
from mavros_msgs.msg import RCIn  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from mhseals_hardware import command_mux, thruster_pwm_node  # noqa: E402
from mhseals_hardware.configuration import load_defaults, save_config  # noqa: E402
from mhseals_hardware.odroid_pwm import OdroidPWMOutputs  # noqa: E402
from test_estop import GUIDED, MANUAL, OFF, ON, fake_chip, frame, line  # noqa: E402,F401

NEUTRAL = ['1500000'] * 4
STICK = 1800             # surge stick past the 200 us takeover: 0.6 * rc_scale 0.5 = 0.3
KEYS = 0.8               # keyboard surge


class Rig:
    def __init__(self, tmp_path, monkeypatch, levels):
        self.levels = levels
        path = tmp_path / 'hardware.yaml'
        save_config(load_defaults(), path)
        chips = [fake_chip(tmp_path / 'pwm', n) for n in range(4)]
        monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda cfg, node=None:
                            OdroidPWMOutputs(chips, mosfet_chip='/dev/gpiochip5', mosfet_line=29))
        monkeypatch.setenv('ROS_DOMAIN_ID', '196')
        rclpy.init(args=['--ros-args', '-p', f'config_file:={path}',
                         '-r', 'thruster_pwm_node:cmd_vel:=/cmd_vel/applied'])
        self.mux = command_mux.CommandMux()
        self.driver = thruster_pwm_node.ThrusterPWMNode()
        probe = self.probe = rclpy.create_node('probe')
        self.rc_pub = probe.create_publisher(RCIn, '/mavros/rc/in', 10)
        self.key_pub = probe.create_publisher(Twist, '/cmd_vel/manual', 10)
        self.states, self.applied, self.status = [], [], []
        probe.create_subscription(String, '/command_mux/state',
                                  lambda m: self.states.append(json.loads(m.data)),
                                  command_mux.STATE_QOS)
        probe.create_subscription(Twist, '/cmd_vel/applied',
                                  lambda m: self.applied.append(m.linear.x), 10)
        probe.create_subscription(String, '/thruster_pwm_node/status',
                                  lambda m: self.status.append(json.loads(m.data)),
                                  thruster_pwm_node.STATUS_QOS)
        self.client = probe.create_client(Trigger, '/thruster_pwm_node/estop')
        self.executor = SingleThreadedExecutor()
        for node in (self.mux, self.driver, probe):
            self.executor.add_node(node)

    def run(self, seconds, rc=None, keys=None):
        """Stream RC at 50 Hz (a frame, or frame(i)) and keys at 20 Hz; return the
        mux inputs published meanwhile."""
        mark, now = len(self.states), time.monotonic()
        end, next_rc, next_key, i = now + seconds, now, now, 0
        while now < end:
            if rc is not None and now >= next_rc:
                channels = rc(i) if callable(rc) else rc
                self.rc_pub.publish(RCIn(channels=channels))
                next_rc, i = now + 0.02, i + 1
            if keys is not None and now >= next_key:
                message = Twist()
                message.linear.x = keys
                self.key_pub.publish(message)
                next_key = now + 0.05
            self.executor.spin_once(timeout_sec=0.005)
            now = time.monotonic()
        return [state['input'] for state in self.states[mark:]]

    def estop(self):
        future = self.client.call_async(Trigger.Request())
        self.executor.spin_until_future_complete(future, timeout_sec=3)
        return future.result()

    @property
    def input(self):
        return self.states[-1]['input']

    @property
    def duties(self):
        return [(c.path / 'duty_cycle').read_text() for c in self.driver.outputs.channels]

    @property
    def powered(self):
        return self.driver.outputs.mosfet.enabled

    def close(self):
        self.executor.shutdown()
        for node in (self.probe, self.mux, self.driver):
            node.destroy_node()
        rclpy.try_shutdown()


@pytest.fixture
def rig(tmp_path, monkeypatch, line):  # noqa: F811
    rig = Rig(tmp_path, monkeypatch, line)
    rig.run(0.3)                      # discovery
    yield rig
    rig.close()


def changes(inputs):
    return [x for i, x in enumerate(inputs) if i == 0 or inputs[i - 1] != x]


def settled(inputs, source):
    """Ends on ``source`` and never left it once there: no flapping."""
    return bool(inputs) and inputs[-1] == source and set(inputs[inputs.index(source):]) == {source}


def dither(i):
    """A stick shaking across the takeover threshold, frame by frame."""
    return frame(mode=GUIDED, surge=1695 if i % 2 else 1705)


# A1: RC sticks take over from the keyboard, cleanly, and the keyboard does not linger.

def test_rc_takeover_beats_the_keyboard_without_flapping(rig):
    rig.run(0.5, rc=frame(mode=GUIDED), keys=KEYS)
    assert rig.input == 'manual' and rig.applied[-1] == pytest.approx(KEYS)
    assert changes(rig.run(0.6, rc=dither, keys=KEYS)) in ([], ['manual'])
    seen = rig.run(0.6, rc=frame(mode=GUIDED, surge=STICK), keys=KEYS)
    assert settled(seen, 'rc') and rig.applied[-1] == pytest.approx(0.3)
    assert changes(rig.run(0.6, rc=dither, keys=KEYS)) in ([], ['rc'])  # in-band: RC keeps it
    rig.run(0.7, rc=frame(mode=GUIDED, surge=STICK))   # keys let go while RC drives
    seen = rig.run(1.0, rc=frame(mode=GUIDED))         # sticks released
    assert 'manual' not in seen and rig.input == 'hold' and rig.applied[-1] == 0.0
    assert rig.powered and not rig.driver.estop.latched


def test_rc_release_hands_back_only_to_a_live_keyboard(rig):
    rig.run(0.5, rc=frame(mode=GUIDED, surge=STICK), keys=KEYS)
    assert rig.input == 'rc'
    seen = rig.run(1.0, rc=frame(mode=GUIDED), keys=KEYS)  # still typing: manual resumes
    assert settled(seen, 'manual') and rig.applied[-1] == pytest.approx(KEYS)


# A2: a centred, connected RC leaves the keyboard alone until a stick takes over.

def test_keyboard_drives_while_rc_is_centred(rig):
    assert set(rig.run(1.0, rc=frame(mode=GUIDED), keys=KEYS)) == {'manual'}
    assert rig.applied[-1] == pytest.approx(KEYS) and rig.duties != NEUTRAL
    # Published on change (plus 1 Hz): nothing new means still manual.
    assert set(rig.run(0.5, rc=frame(mode=GUIDED, surge=1650), keys=KEYS)) <= {'manual'}
    assert set(rig.run(0.1, rc=frame(mode=GUIDED, surge=STICK), keys=KEYS)) <= {'manual'}
    assert rig.input == 'manual' and rig.applied[-1] == pytest.approx(KEYS)
    rig.run(0.4, rc=frame(mode=GUIDED, surge=STICK), keys=KEYS)  # held past stick_time
    assert rig.input == 'rc'


# A3: RC loss cuts power only when RC was in control.

def test_rc_loss_while_the_keyboard_drives_keeps_power(rig):
    rig.run(0.5, rc=frame(mode=GUIDED), keys=KEYS)
    seen = rig.run(1.0, keys=KEYS)                     # receiver gone
    assert set(seen) <= {'manual'} and rig.applied[-1] == pytest.approx(KEYS)
    assert rig.powered and not rig.driver.estop.latched and rig.duties != NEUTRAL


def test_rc_loss_while_rc_drives_cuts_and_both_displays_agree(rig):
    rig.run(0.5, rc=frame(mode=MANUAL, surge=STICK))
    assert rig.input == 'rc' and rig.duties != NEUTRAL
    rig.run(0.8)                                       # receiver gone
    assert not rig.powered and rig.duties == NEUTRAL
    assert rig.status[-1]['estop']['cause'] == 'rc-lost' and rig.status[-1]['mosfet'] == 'off'
    assert rig.input == 'hold' and rig.states[-1]['hold']['cause'] == 'rc_lost'


# B: both sources can e-stop; the cut is the same latch and neither undoes the other.

def test_operator_estop_while_rc_drives(rig):
    rig.run(0.5, rc=frame(mode=MANUAL, surge=STICK))
    response = rig.estop()
    assert response.success and response.message == 'latched (operator)'
    assert not rig.powered and rig.duties == NEUTRAL
    rig.run(0.3, rc=frame(ON, mode=MANUAL, surge=STICK))
    rig.run(0.3, rc=frame(OFF, mode=MANUAL, surge=STICK))   # the RC e-stop on top
    rig.run(0.3, rc=frame(ON, mode=MANUAL, surge=STICK))
    assert rig.duties == NEUTRAL and rig.levels.count(1) == 1
    estop = rig.status[-1]['estop']
    assert estop['latched'] and estop['cause'] == 'operator'


def test_rc_estop_while_the_keyboard_drives(rig):
    rig.run(0.5, rc=frame(ON, mode=GUIDED), keys=KEYS)
    assert rig.input == 'manual' and rig.duties != NEUTRAL
    rig.run(0.2, rc=frame(OFF, mode=GUIDED), keys=KEYS)
    assert not rig.powered and rig.duties == NEUTRAL
    assert rig.status[-1]['estop']['cause'] == 'rc-stop'
    response = rig.estop()                             # operator too: already off
    assert response.success and response.message == 'latched (rc-stop)'
    rig.run(0.5, rc=frame(ON, mode=GUIDED), keys=KEYS)  # switch back, keep typing
    assert rig.duties == NEUTRAL and rig.levels.count(1) == 1
