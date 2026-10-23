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
    # Yaw runs at half the linear amplitude: it is the more aggressive axis.
    assert state.update('press:left', .05) == (.25, 0, .125)
    assert state.update('press:+', .06) == (.3, 0, .15)
    assert state.update('release:+', .07) == (.3, 0, .15)


@pytest.mark.parametrize('stop', ['press:space', 'focus-out', None])
def test_stop_and_timeout_cannot_rearm_from_repeat(stop):
    state = ManualState()
    state.update('press:w', 0)
    state.update('press:d', .1)
    assert state.update(stop, 1) == (0, 0, 0)
    assert state.update('repeat:d', 1.01) == (0, 0, 0)
    state.update('focus-in', 1.02)
    assert state.update('press:w', 1.03) == (.25, 0, 0)


def test_plain_keys_replace_each_other_because_only_the_newest_repeats():
    state = ManualState()
    state.update('w', 0)
    assert state.update('d', .1) == (0, -.25, 0)
    assert state.update('d', .5) == (0, -.25, 0)
    assert state.update('left', .6) == (0, 0, .125)
    assert state.update('s', .65) == (-.25, 0, 0)
    assert state.update(None, 1.31) == (0, 0, 0)  # repeats stopped
    assert not state.chords


def test_tmux_yaw_stops_when_another_key_takes_over():
    # tmux forwards no Kitty events: arrows arrive bare, so every repeat
    # decodes as a press, letters arrive plain, and no release ever comes.
    state = ManualState()
    left = decode_key(b'\x1b[D', True)
    for tick in range(0, 50, 5):
        assert state.update(left, tick / 100) == (0, 0, .125)
    assert state.update(decode_key(b'w', True), .5) == (.25, 0, 0)
    for tick in range(55, 150, 5):  # the left arrow was let go; only w repeats
        assert state.update(decode_key(b'w', True), tick / 100) == (.25, 0, 0)
    assert not state.chords


@pytest.mark.parametrize('key,expected', [
    (b'w', (.25, 0, 0)), (b's', (-.25, 0, 0)),
    (b'a', (0, .25, 0)), (b'd', (0, -.25, 0)),
])
def test_plain_wasd_replaces_an_unconfirmed_arrow(key, expected):
    # Before any event, a bare arrow may be tmux's plain key with no release.
    state = ManualState()
    assert state.update(decode_key(b'\x1b[D', True), 0) == (0, 0, .125)
    assert state.update(decode_key(key, True), .05) == expected
    assert state.update('release:left', .1) == expected
    assert state.update(None, .76) == (0, 0, 0)


def test_yaw_and_plain_translation_combine_and_release_separately():
    state = ManualState()
    state.update('press:left', 0)
    state.update('repeat:left', .05)  # events confirmed: the arrow will report its release
    assert state.update('w', .1) == (.25, 0, .125)
    assert state.update('release:left', .15) == (.25, 0, 0)
    assert state.update(None, .81) == (0, 0, 0)


def test_kitty_letter_press_chords_from_the_first_key():
    state = ManualState()
    state.update(decode_key(b'\x1b[119u', True), 0)
    assert state.chords
    diagonal = (.25 / math.sqrt(2), -.25 / math.sqrt(2), 0)
    assert state.update(decode_key(b'\x1b[100u', True), .05) == diagonal
    assert state.command == state.update(None, .1)


def test_bare_arrow_is_promoted_once_its_repeat_proves_events():
    state = ManualState()
    state.update('press:left', 0)
    state.update('repeat:left', .3)
    state.update('press:w', .35)
    assert state.command == (.25, 0, .125)
    assert state.update('release:w', .4) == (0, 0, .125)


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


def test_shift_tab_is_backtab():
    from mhseals_hardware.keyboard import decode_key
    assert decode_key(b'\x1b[Z') == 'backtab' and decode_key(b'\t') == '\t'


def test_ctrl_hjkl_and_ctrl_arrows_are_focus_keys_and_enter_stays_enter():
    from mhseals_hardware.keyboard import decode_key
    assert [decode_key(b) for b in (b'\x08', b'\n', b'\x0b', b'\x0c')] == [
        'ctrl-h', 'ctrl-j', 'ctrl-k', 'ctrl-l']
    assert [decode_key(b'\x1b[1;5' + c) for c in (b'A', b'B', b'C', b'D')] == [
        'ctrl-up', 'ctrl-down', 'ctrl-right', 'ctrl-left']
    assert decode_key(b'\r') == 'enter' and decode_key(b'\x1b[A') == 'up'


def test_arrow_presses_join_held_chords_so_the_boat_can_turn_while_moving():
    from mhseals_hardware.keyboard import decode_key
    from mhseals_hardware.manual_state import ManualState
    keys = [decode_key(raw, report_events=True) for raw in (
        b'\x1b[119u', b'\x1b[D', b'\x1b[1;1:2D', b'\x1b[119;1:2u')]
    assert keys == ['press:w', 'press:left', 'repeat:left', 'repeat:w']
    state = ManualState(0.25, 0.15, 0.65)
    for now, key in enumerate(keys):
        command = state.update(key, now * 0.05)
    assert command[0] > 0 and command[2] > 0  # forward and turning left together
    assert state.update(decode_key(b'\x1b[1;1:3D', report_events=True), 0.3)[2] == 0
    assert decode_key(b'\x1b[D') == 'left'  # plain terminals keep plain names


def test_held_key_survives_another_keys_release_because_it_gets_no_more_repeats():
    state = ManualState()
    state.update('press:w', 0)
    for tick in range(50, 100, 10):
        state.update('repeat:w', tick / 100)
    state.update('press:d', 1.0)  # from here only d repeats
    for tick in range(110, 200, 10):
        state.update('repeat:d', tick / 100)
    state.update('release:d', 2.0)
    assert state.update(None, 3.0) == (.25, 0, 0)  # w is still held
    assert state.update('release:w', 3.1) == (0, 0, 0)


def test_chord_docs_state_the_held_key_deadman():
    root = Path(__file__).parent.parent
    assert 'another key' in (root / 'README.md').read_text().replace('\n', ' ')
    assert 're-press' in (root / 'mhseals_hardware/keyboard_control.py').read_text()


def test_held_key_stops_two_seconds_after_other_release():
    state = ManualState()
    state.update('press:w', 0)
    state.update('press:d', .1)
    for tick in range(60, 200, 5):
        state.update('repeat:d', tick / 100)
    state.update('release:d', 2.0)
    assert state.update(None, 3.9) != (0, 0, 0)
    assert state.update(None, 4.1) == (0, 0, 0)
