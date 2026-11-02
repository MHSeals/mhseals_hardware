"""KeyReader's input layer: bulk reads, one reader across event mode, mouse, terminal kinds, holds."""

import os

import pytest

from mhseals_hardware import keyboard
from mhseals_hardware.keyboard import Hold, KeyReader, Terminal, decode_mouse


@pytest.fixture
def feed():
    read_fd, write_fd = os.pipe()
    reader = KeyReader()
    reader.fd = read_fd
    yield reader, lambda data: os.write(write_fd, data)
    os.close(read_fd)
    os.close(write_fd)


@pytest.mark.smoke
def test_one_bulk_read_yields_every_queued_key_in_order(feed):
    reader, send = feed
    send(b'ab\x1b[A\x1b[?1u\x1b[<0;3;4M' + 'é'.encode() + b'\x1b[Z')
    keys = [reader.read(0.2) for _ in range(6)]
    assert keys == ['a', 'b', 'up', 'press:mouse-left', 'é', 'backtab']
    assert (keys[3].col, keys[3].row) == (3, 4)
    assert reader.read(0.05) is None


@pytest.mark.smoke
def test_events_keep_the_reader_and_its_queue(feed, monkeypatch):
    reader, send = feed
    written = []
    monkeypatch.setattr(keyboard, 'terminal_write', written.append)
    send(b'q\x1b[119;1:1u')
    with reader.events() as keys:
        assert keys is reader and reader.report_events
        assert [reader.read(0.2), reader.read(0.2)] == ['q', 'press:w']
        with reader.events():      # nested: no second request
            pass
    assert not reader.report_events
    assert written == [keyboard.EVENTS_ON, keyboard.EVENTS_OFF]


def test_lone_escape_still_waits_for_its_sequence(feed):
    reader, send = feed
    send(b'\x1b')
    assert reader.read(0.01) is None
    send(b'[B')
    assert reader.read(0.2) == 'down'
    send(b'\x1b')
    assert reader.read(0.2) == 'escape'


@pytest.mark.parametrize('report, name, action, button', [
    (b'\x1b[<0;1;1M', 'press:mouse-left', 'press', 'left'),
    (b'\x1b[<2;1;1m', 'release:mouse-right', 'release', 'right'),
    (b'\x1b[<33;1;1M', 'drag:mouse-middle', 'drag', 'middle'),
    (b'\x1b[<35;1;1M', 'move:mouse', 'move', None),
    (b'\x1b[<64;1;1M', 'wheel-up', 'wheel', 'up'),
    (b'\x1b[<81;1;1M', 'wheel-down', 'wheel', 'down'),
])
def test_sgr_mouse(report, name, action, button):
    mouse = decode_mouse(report)
    assert (mouse, mouse.action, mouse.button) == (name, action, button)
    assert decode_mouse(b'\x1b[A') is None
    assert decode_mouse(b'\x1b[<20;1;1M').mods == {'shift', 'ctrl'}


@pytest.mark.parametrize('caps, env, kind', [
    ({'keyboard': 1, 'done': True}, {}, 'kitty'),
    ({'keyboard': 1, 'done': True}, {'TMUX': '/tmp/t'}, 'legacy'),
    ({'xtversion': 'tmux 3.5a', 'done': True}, {}, 'legacy'),
    ({'done': True}, {}, 'plain'),
    ({}, {}, 'unknown'),
])
def test_terminal_kind(caps, env, kind):
    terminal = Terminal.detect(caps, env)
    assert terminal.kind == kind
    assert terminal.releases == (kind == 'kitty')
    assert bool(terminal.chords_unavailable) == (kind in ('legacy', 'plain'))


def test_hold_with_releases_ends_at_the_release_only():
    hold = Hold('y', 1.0, releases=True)
    assert not hold.feed('press:y', 0.0)
    assert not hold.tick(0.9)
    assert hold.tick(1.0) and hold.progress(1.0) == 1.0
    assert not hold.feed('release:y', 1.1)
    assert not hold.feed('repeat:y', 1.2)      # a repeat never starts a hold
    hold.feed('press:mouse-left', 0.0)
    assert not hold.feed('x', 0.5)             # another key cancels


def test_hold_from_a_repeat_stream_ends_at_a_gap():
    hold = Hold('y', 1.0, releases=False, first_gap_s=0.65, gap_s=0.15)
    times = [0.0, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    assert [hold.feed('y', t) for t in times][-1]
    hold.reset()
    hold.feed('y', 0.0)
    hold.feed('y', 0.5)
    assert not hold.tick(0.9)                  # 0.4 s without a repeat
    assert hold.progress(0.9) == 0.0
