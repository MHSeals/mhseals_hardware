"""Unit tests for native Linux PWM conversion and output behavior."""

from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

from mhseals_hardware.odroid_pwm import (
    OdroidPWMOutputs, SysfsPWMChannel, period_ns, pulse_ns, resolve_pwm_chip,
)


def test_gpio_bank_resolves_independently_of_device_number(tmp_path):
    from mhseals_hardware.odroid_pwm import resolve_gpio_chip
    for number, label in ((3, 'gpio1'), (5, 'gpio3')):
        chip = tmp_path / f'gpiochip{number}'
        chip.mkdir()
        (chip / 'label').write_text(label + '\n')
    assert resolve_gpio_chip('gpio3', tmp_path) == '/dev/gpiochip5'
    with pytest.raises(FileNotFoundError):
        resolve_gpio_chip('gpio4', tmp_path)
    assert resolve_gpio_chip('/dev/gpiochip5', tmp_path) == '/dev/gpiochip5'


def test_gpio_bank_resolves_with_gpiod_when_sysfs_has_no_labels(tmp_path,
                                                                monkeypatch):
    from mhseals_hardware import odroid_pwm

    class Chip:
        def __init__(self, path):
            self.path = path

        def label(self):
            return {'/dev/gpiochip3': 'gpio3'}[self.path]

        def close(self):
            pass

    monkeypatch.setitem(sys.modules, 'gpiod', SimpleNamespace(Chip=Chip))
    monkeypatch.setattr(odroid_pwm.glob, 'glob',
                        lambda _pattern: ['/dev/gpiochip3'])
    assert odroid_pwm.resolve_gpio_chip('gpio3', tmp_path) == \
        '/dev/gpiochip3'


@pytest.mark.parametrize('version', [1, 2])
@pytest.mark.parametrize('active_high', [True, False])
def test_mosfet_polarity_and_release(monkeypatch, version, active_high):
    from mhseals_hardware.odroid_pwm import MosfetEnable
    events = []

    class Request:
        def request(self, **kwargs):
            events.append(kwargs['default_vals'][0])

        def set_value(self, *args):
            if len(args) != version:
                raise TypeError('wrong API signature')
            events.append(args[-1])

        def release(self):
            events.append('release')

    class Chip:
        def __init__(self, path):
            assert path == '/dev/gpiochip5'

        def close(self):
            events.append('close')

        def get_line(self, offset):
            assert offset == 28
            return Request()

    if version == 2:
        def request_lines(self, **kwargs):
            events.append(kwargs['config'][28].output_value)
            return Request()
        Chip.request_lines = request_lines
    module = SimpleNamespace(Chip=Chip, LINE_REQ_DIR_OUT=3)
    if version == 2:
        module.LineSettings = SimpleNamespace
        module.line = SimpleNamespace(
            Value=SimpleNamespace(ACTIVE=1, INACTIVE=0),
            Direction=SimpleNamespace(OUTPUT=3))
    monkeypatch.setitem(sys.modules, 'gpiod', module)
    enable = MosfetEnable('/dev/gpiochip5', 28, active_high).open()
    enable.set_enabled(True)
    enable.close()
    enable.close()
    assert events == [int(not active_high), int(active_high),
                      int(not active_high), 'release', 'close']


def fake_chip(root, number):
    chip = root / f'pwmchip{number}'
    pwm = chip / 'pwm0'
    pwm.mkdir(parents=True)
    for name in ('enable', 'duty_cycle', 'period', 'polarity'):
        (pwm / name).write_text('0', encoding='ascii')
    return chip


def test_esc_timing_conversion():
    assert period_ns(50) == 20_000_000
    assert pulse_ns(1500, 50) == 1_500_000


@pytest.mark.parametrize(('frequency', 'pulse'), ((0, 1500), (50, 21000)))
def test_invalid_timing_is_rejected(frequency, pulse):
    with pytest.raises(ValueError):
        pulse_ns(pulse, frequency)


def test_channel_configures_period_duty_and_enable(tmp_path):
    chip = fake_chip(tmp_path, 0)
    channel = SysfsPWMChannel(chip).open()
    channel.configure(50, 1500)
    assert (channel.path / 'period').read_text() == '20000000'
    assert (channel.path / 'duty_cycle').read_text() == '1500000'
    assert (channel.path / 'polarity').read_text() == 'normal'
    assert (channel.path / 'enable').read_text() == '1'


def test_stable_platform_glob_resolves_dynamic_chip_number(tmp_path):
    chip = fake_chip(tmp_path / 'febd0030.pwm' / 'pwm', 17)
    assert resolve_pwm_chip(tmp_path / 'febd0030.pwm' / 'pwm' / 'pwmchip*') == chip


def test_four_outputs_neutralize_on_close(tmp_path):
    chips = [fake_chip(tmp_path, number) for number in range(4)]
    outputs = OdroidPWMOutputs(chips, mosfet_chip=None).open()
    outputs.set_pulse_widths([1600, 1400, 1550, 1450])
    outputs.close()
    for channel in outputs.channels:
        assert (channel.path / 'duty_cycle').read_text() == '1500000'
        assert (channel.path / 'enable').read_text() == '0'


def test_close_disables_every_channel_after_one_neutral_failure(tmp_path,
                                                                monkeypatch):
    chips = [fake_chip(tmp_path, number) for number in range(4)]
    outputs = OdroidPWMOutputs(chips, mosfet_chip=None).open()

    def fail_neutral(_pulse_width):
        raise OSError('simulated duty-cycle failure')

    monkeypatch.setattr(outputs.channels[0], 'set_pulse_width', fail_neutral)
    with pytest.raises(OSError, match='simulated duty-cycle failure'):
        outputs.close()
    for channel in outputs.channels:
        assert (channel.path / 'enable').read_text() == '0'
