import math
import ast
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from mhseals_hardware.keyboard import KeyReader, decode_key
from mhseals_hardware.manual_state import ManualState


def test_event_type_survives_decode():
    assert decode_key(b'\x1b[119;1:1u', True) == 'press:w'
    assert decode_key(b'\x1b[119;1:2u', True) == 'repeat:w'
    assert decode_key(b'\x1b[119;1:3u', True) == 'release:w'


def test_diagonal_survives_initial_gap_and_one_key_repeating():
    state = ManualState()
    state.update('press:w', 0)
    expected = (.25 / math.sqrt(2), -.25 / math.sqrt(2), 0)
    assert state.update('press:d', .1) == expected
    assert state.update(None, .5) == expected
    for tick in range(60, 200, 5):
        assert state.update('repeat:d', tick / 100) == expected
    assert state.update('release:d', 2.0) == (.25, 0, 0)
    assert state.update('release:w', 2.01) == (0, 0, 0)


def test_opposites_aliases_yaw_speed_and_releases():
    state = ManualState()
    state.update('press:w', 0)
    assert state.update('press:up', .01) == (.25, 0, 0)
    assert state.update('release:w', .02) == (.25, 0, 0)
    assert state.update('press:s', .03) == (0, 0, 0)
    assert state.update('release:s', .04) == (.25, 0, 0)
    assert state.update('press:left', .05) == (.25, 0, .25)
    assert state.update('press:+', .06) == (.3, 0, .3)
    assert state.update('release:+', .07) == (.3, 0, .3)


@pytest.mark.parametrize('stop', ['press:space', 'focus-out', None])
def test_stop_and_timeout_cannot_rearm_from_repeat(stop):
    state = ManualState()
    state.update('press:w', 0)
    state.update('press:d', .1)
    assert state.update(stop, 1) == (0, 0, 0)
    assert state.update('repeat:d', 1.01) == (0, 0, 0)
    state.update('focus-in', 1.02)
    assert state.update('press:w', 1.03) == (.25, 0, 0)


def test_legacy_remains_single_key_not_phantom_chords():
    state = ManualState()
    state.update('w', 0)
    assert state.update('d', .1) == (0, -.25, 0)
    assert state.update(None, .5) == (0, -.25, 0)
    assert state.update(None, .76) == (0, 0, 0)


def test_real_pty_fragmented_and_queued_events():
    master, slave = os.openpty()
    import termios
    attributes = termios.tcgetattr(slave)
    attributes[3] &= ~(termios.ICANON | termios.ECHO)
    termios.tcsetattr(slave, termios.TCSANOW, attributes)
    reader = KeyReader(report_events=True)
    reader.fd = slave
    payload = b'\x1b[119;1:1u\x1b[100;1:1u\x1b[100;1:3u\x1b[O'

    def write():
        for byte in payload:
            os.write(master, bytes([byte]))
            time.sleep(.001)

    writer = threading.Thread(target=write)
    try:
        writer.start()
        assert [reader.read(.5) for _ in range(4)] == [
            'press:w', 'press:d', 'release:d', 'focus-out']
        writer.join()
    finally:
        writer.join()
        os.close(master)
        os.close(slave)


def test_reader_eof_is_not_an_empty_key():
    read_fd, write_fd = os.pipe()
    os.close(write_fd)
    reader = KeyReader(report_events=True)
    reader.fd = read_fd
    try:
        with pytest.raises(EOFError):
            reader.read(0)
    finally:
        os.close(read_fd)


def test_split_sequence_survives_poll_timeouts():
    read_fd, write_fd = os.pipe()
    reader = KeyReader(report_events=True)
    reader.fd = read_fd
    try:
        os.write(write_fd, b'\x1b[119;')
        assert reader.read(0) is None
        os.write(write_fd, b'1:1u')
        assert reader.read(0) == 'press:w'
    finally:
        os.close(read_fd)
        os.close(write_fd)


@pytest.mark.parametrize('delay', [.1, .25, .4, .5, .6])
def test_no_neutral_at_any_tick_before_initial_repeat(delay):
    state = ManualState()
    state.update('press:w', 0)
    state.update('press:d', .01)
    for tick in range(2, round(delay * 100)):
        vector = state.update(None, tick / 100)
        assert vector[0] > 0 and vector[1] < 0
    vector = state.update('repeat:d', delay)
    assert vector[0] > 0 and vector[1] < 0


def test_kitty_control_c_still_interrupts():
    with pytest.raises(KeyboardInterrupt):
        decode_key(b'\x1b[99;5u', True)


@pytest.mark.parametrize('disconnect', [False, True])
def test_actual_publish_loop_has_no_initial_neutral_and_finally_stops(disconnect):
    # Execute production UI/publisher functions without ROS or a physical motor.
    source = Path(__file__).parents[1] / 'mhseals_hardware/keyboard_control.py'
    tree = ast.parse(source.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name in ('run_manual', 'publish_manual', 'manual_panel')]
    clock = SimpleNamespace(now=0.0)
    events = [(0, 'press:w'), (.1, 'press:d')]
    events += [(t / 100, 'repeat:d') for t in range(60, 120, 5)]
    events += [(1.2, 'release:d'), (1.3, 'release:w'), (1.4, 'press:x')]

    class Reader:
        def __init__(self, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def read(self, timeout):
            deadline = clock.now + timeout
            if events and events[0][0] <= deadline:
                clock.now, key = events.pop(0)
                if disconnect and key == 'release:d':
                    raise EOFError('test disconnect')
                return key
            clock.now = deadline
            return None

    class Live(Reader):
        def __init__(self, *args, **kwargs):
            pass

        def update(self, panel):
            pass

    def twist():
        return SimpleNamespace(linear=SimpleNamespace(x=0., y=0.),
                               angular=SimpleNamespace(z=0.))

    messages = []
    publisher = SimpleNamespace(publish=lambda msg: messages.append(
        (clock.now, (msg.linear.x, msg.linear.y, msg.angular.z))))
    scope = dict(ManualState=ManualState, KeyReader=Reader, Live=Live,
                 Console=lambda: object(), Panel=lambda *a, **kw: a[0],
                 Twist=twist, time=SimpleNamespace(monotonic=lambda: clock.now))
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), scope)
    if disconnect:
        with pytest.raises(EOFError):
            scope['run_manual'](publisher)
    else:
        scope['run_manual'](publisher)
    assert messages[-1][1] == (0, 0, 0)
    moving = [(t, vector) for t, vector in messages[:-1] if t < 1.2]
    assert len(moving) > 15
    assert all(vector[0] > 0 for _, vector in moving)
    assert all(vector[1] < 0 for t, vector in moving if t >= .1)
    if not disconnect:
        assert all(vector == (.25, 0, 0) for t, vector in messages if 1.2 <= t < 1.3)
