"""Byte-fed terminal probe replies: they set reader state and never become keys."""

import os
import time

import pytest

from mhseals_hardware.keyboard import KeyReader

APC_OK = b'\x1b_Gi=31;OK\x1b\\'
XTVERSION = b'\x1bP>|kitty(0.35.2)\x1b\\'
CELL = b'\x1b[6;38;19t'
DA1 = b'\x1b[?62;c'


@pytest.fixture
def feed():
    read_fd, write_fd = os.pipe()
    reader = KeyReader(report_events=True)
    reader.fd = read_fd
    yield reader, lambda data: os.write(write_fd, data)
    os.close(read_fd)
    os.close(write_fd)


def test_kitty_replies_set_graphics_and_cell_size(feed):
    reader, send = feed
    send(APC_OK + XTVERSION + CELL + DA1 + b'w')
    assert reader.read(1) == 'w'
    assert reader.graphics is True
    assert reader.cell_px == (19, 38)
    assert reader.xtversion == 'kitty(0.35.2)'
    assert reader.probed


def test_tmux_answers_only_da1(feed):
    reader, send = feed
    send(DA1 + b'a')
    assert reader.read(1) == 'a'
    assert reader.graphics is False and reader.cell_px is None and reader.probed


def test_graphics_ok_after_da1_does_not_count(feed):
    reader, send = feed
    send(DA1 + APC_OK + b'a')
    assert reader.read(1) == 'a'
    assert reader.graphics is False


def test_apc_mid_keystroke_never_becomes_keys(feed):
    reader, send = feed
    send(b'\x1b[119;1:1u' + APC_OK + b'\x1b[119;1:3u')
    assert reader.read(1) == 'press:w'
    assert reader.read(1) == 'release:w'
    assert reader.read(0.05) is None
    # An APC split across reads is still framed up to ESC \.
    send(b'\x1b_Gi=31;O')
    assert reader.read(0.05) is None
    send(b'K\x1b\\q')
    assert reader.read(1) == 'q'


@pytest.mark.parametrize('intro', [b']', b'P', b'_', b'X', b'^'])
def test_unterminated_string_introducer_is_dropped(feed, intro):
    reader, send = feed
    send(b'\x1b' + intro)
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        assert reader.read(0.1) is None
    send(b'q')
    assert reader.read(1) == 'q'
