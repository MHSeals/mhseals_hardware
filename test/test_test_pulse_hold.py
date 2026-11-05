"""A test pulse reaches the outputs for its whole duration, under the mux's 25 Hz cmd_vel.

Since the command_mux, /cmd_vel/applied carries zeros at 25 Hz even while idle; the driver
used to obey the newest message, so each pulse was overwritten within 40 ms (11-04 boat test:
48 single-thruster pulses, no motion). The pulse now owns the outputs while it lives, and the
mux guard, its hold and the e-stop still end it.
"""

import json

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt16MultiArray

from mhseals_hardware import thruster_pwm_node
from mhseals_hardware.configuration import load_defaults, save_config

PULSE = [1500, 1740, 1500, 1500]
NEUTRAL = [1500] * 4


class Outputs:
    """Fake outputs that hold what was last written and log every write."""

    neutral_us = 1500

    def __init__(self):
        self.pulses, self.writes = list(NEUTRAL), []

    def open(self):
        return self

    def close(self, unexport=False):
        pass

    def set_pulse_widths(self, pulses):
        self.pulses = [int(pulse) for pulse in pulses]
        self.writes.append(list(self.pulses))

    def neutral(self):
        self.set_pulse_widths(NEUTRAL)

    def readback(self):
        return list(self.pulses)


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def start_driver(tmp_path, monkeypatch, **config):
    outputs, clock = Outputs(), Clock()
    monkeypatch.setattr(thruster_pwm_node, 'make_outputs', lambda *args, **kwargs: outputs)
    monkeypatch.setattr(thruster_pwm_node.time, 'monotonic', clock)
    path = tmp_path / 'hardware.yaml'
    save_config(dict(load_defaults(), **config), path)
    rclpy.init(args=['--ros-args', '-r', '__node:=pulse_hold_test', '-p', f'config_file:={path}'])
    node = thruster_pwm_node.ThrusterPWMNode()
    published = []
    if hasattr(node, 'output_pub'):   # the golden recorder runs this on the driver before it
        node.output_pub.publish = lambda message: published.append(json.loads(message.data))
    node.guard_callback(String(data='{"input": "idle"}'))
    try:
        yield node, outputs, clock, published
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def driver(tmp_path, monkeypatch):
    yield from start_driver(tmp_path, monkeypatch)


# Non-identity wiring for the golden manual sequence: every output moved, two reversed.
MAPPED = dict(channel_map=[2, 4, 1, 3], reversed=[True, False, False, True])


@pytest.fixture
def mapped_driver(tmp_path, monkeypatch):
    yield from start_driver(tmp_path, monkeypatch, **MAPPED)


def held(pulses, hold_s, seq=1):
    return String(data=json.dumps({'pulses': pulses, 'hold_s': hold_s, 'seq': seq}))


def run(node, clock, seconds, pulse=None, pulse_every=0.1, stall=None):
    """Step time in 10 ms ticks: mux zeros every 40 ms, the pulse sender every ``pulse_every``
    (silent within ``stall``), the watchdog every 100 ms, the mux state every second."""
    start, tick = clock.now, 0
    while clock.now - start < seconds - 1e-9:
        t = clock.now - start
        if pulse is not None and tick % round(pulse_every / 0.01) == 0 and not (
                stall and stall[0] <= t < stall[1]):
            pulse()
        if tick % 4 == 0:
            node.cmd_vel_callback(Twist())
        if tick % 10 == 0:
            node.watchdog_callback()
        if tick % 100 == 0:
            node.guard_callback(String(data='{"input": "idle"}'))
        clock.now += 0.01
        tick += 1


def test_a_pulse_holds_through_the_mux_zero_stream(driver):
    node, outputs, clock, published = driver
    node.test_pulse(PULSE, 3.0, 7)
    outputs.writes.clear()
    run(node, clock, 2.9)
    assert outputs.pulses == PULSE and outputs.writes in ([], [PULSE] * len(outputs.writes))
    assert published[-1]['source'] == 'test' and published[-1]['pulses'] == PULSE
    assert published[-1]['test']['seq'] == 7
    run(node, clock, 0.3)                       # its hold runs out: neutral, cmd_vel again
    assert outputs.pulses == NEUTRAL and published[-1]['source'] != 'test'


def test_the_tui_republish_survives_a_stall_with_the_held_topic(driver):
    node, outputs, clock, _ = driver
    send = lambda: node.held_pulses_callback(held(PULSE, 1.5))
    send()
    run(node, clock, 3.0, pulse=send, stall=(1.0, 2.2))   # a 1.2 s TUI stall
    assert all(write == PULSE for write in outputs.writes) and outputs.pulses == PULSE
    node.held_pulses_callback(held(NEUTRAL, 1.5))        # 0 or Tab: ends at once
    assert outputs.pulses == NEUTRAL and node.test is None


def test_the_legacy_topic_holds_while_republished(driver):
    node, outputs, clock, _ = driver
    send = lambda: node.pulses_callback(UInt16MultiArray(data=PULSE))
    send()
    run(node, clock, 3.0, pulse=send)
    assert all(write == PULSE for write in outputs.writes) and outputs.pulses == PULSE
    run(node, clock, 1.0)                                # sender gone: command_timeout
    assert outputs.pulses == NEUTRAL


@pytest.mark.parametrize('mux_input', ['manual', 'rc', 'hold', 'auto'])
def test_a_human_or_a_hold_ends_the_pulse_at_once(driver, mux_input):
    node, outputs, clock, published = driver
    node.test_pulse(PULSE, 3.0, 1)
    node.guard_callback(String(data=json.dumps({'input': mux_input})))
    assert outputs.pulses == NEUTRAL and node.test is None
    assert 'test' not in (published[-1]['source'],)
    node.test_pulse(PULSE, 3.0, 2)                       # refused while they drive
    assert outputs.pulses == NEUTRAL
    drive = Twist()
    drive.linear.x = 0.5
    node.cmd_vel_callback(drive)                          # and their commands go through
    assert outputs.pulses != NEUTRAL


def test_a_stale_mux_state_ends_the_pulse(driver):
    node, outputs, clock, _ = driver
    node.test_pulse(PULSE, 5.0, 1)
    clock.now += thruster_pwm_node.GUARD_MAX_AGE + 0.1
    node.watchdog_callback()
    assert outputs.pulses == NEUTRAL and node.test is None


def test_the_estop_ends_the_pulse_and_refuses_more(driver):
    node, outputs, clock, published = driver
    node.test_pulse(PULSE, 3.0, 1)
    node.estop_step(node.estop.trip, 'operator')
    assert outputs.pulses == NEUTRAL
    node.watchdog_callback()
    node.test_pulse(PULSE, 3.0, 2)
    assert outputs.pulses == NEUTRAL and node.test is None
    assert published[-1]['source'] == 'estop'


def test_a_held_pulse_is_capped_and_validated(driver):
    node, outputs, clock, _ = driver
    node.held_pulses_callback(held(PULSE, 60.0))
    assert node.test['until'] - clock.now == pytest.approx(thruster_pwm_node.TEST_HOLD_MAX_S)
    node.end_test(None)
    for bad in (held([1500, 2500, 1500, 1500], 1.0), held(PULSE, -1.0), held(PULSE[:3], 1.0),
                String(data='not json'), held([1500.0, 1740.0, 1500.0, 1500.0], 1.0)):
        node.held_pulses_callback(bad)
        assert node.test is None and outputs.pulses == NEUTRAL


def manual_sequence(node, clock):
    """A recorded manual drive: 25 Hz frames with watchdog ticks, a NaN frame, a stall into the
    command_timeout neutral, and frames again; every outputs write in order."""
    frames = [(0.0, 0.0, 0.0), (0.75, 0.0, 0.0), (0.75, 0.0, 0.13), (0.0, -0.75, 0.0),
              (-0.75, 0.0, 0.0), (0.0, 0.0, -0.13), (0.4, 0.3, 0.1), (1.0, 1.0, 1.0),
              (-1.0, -1.0, -1.0), (float('nan'), 0.0, 0.0), (0.2, -0.6, 0.05), (0.0, 0.0, 0.0)]
    for index, (surge, sway, yaw) in enumerate(frames):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = surge, sway, yaw
        node.cmd_vel_callback(message)
        for _ in range(4):           # 40 ms between frames, the watchdog every 10 ms here
            clock.now += 0.01
            node.watchdog_callback()
        if index == 8:               # the stream stalls: the watchdog's timeout neutral
            for _ in range(int(node.command_timeout / 0.1) + 3):
                clock.now += 0.1
                node.watchdog_callback()


# Recorded from the driver before the test-pulse change (mhseals_hardware 3d79a7e) with MAPPED.
GOLDEN = [
    [1500, 1500, 1500, 1500],
    [1800, 1800, 1200, 1200],
    [1804, 1795, 1205, 1196],
    [1800, 1800, 1800, 1800],
    [1200, 1200, 1800, 1800],
    [1448, 1552, 1448, 1552],
    [1613, 1598, 1304, 1296],
    [1731, 1269, 1269, 1100],
    [1269, 1731, 1731, 1900],
    [1500, 1500, 1500, 1500],
    [1500, 1500, 1500, 1500],
    [1754, 1752, 1727, 1725],
    [1500, 1500, 1500, 1500],
]


def test_manual_and_cmd_vel_writes_are_unchanged(mapped_driver):
    """The recorded manual sequence writes exactly what the driver before this change wrote."""
    node, outputs, clock, _ = mapped_driver
    outputs.writes.clear()
    manual_sequence(node, clock)
    assert outputs.writes == GOLDEN


def test_a_command_frame_takes_over_before_its_mux_state_arrives(driver):
    """The mux sends a takeover's first frame before its state: that frame ends the pulse and drives."""
    node, outputs, clock, _ = driver
    node.test_pulse(PULSE, 3.0, 1)
    drive = Twist()
    drive.linear.x = 0.5
    node.cmd_vel_callback(drive)
    assert node.test is None and outputs.pulses not in (PULSE, NEUTRAL)
    assert outputs.writes[-2:][0] == NEUTRAL      # the pulse ended at neutral, then the frame


def test_pulses_are_refused_while_autonomy_drives(driver):
    node, outputs, clock, _ = driver
    node.guard_callback(String(data='{"input": "auto"}'))
    node.test_pulse(PULSE, 3.0, 1)
    assert node.test is None and outputs.pulses == NEUTRAL
