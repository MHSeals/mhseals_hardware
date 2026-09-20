"""Unit tests for native Linux PWM conversion and output behavior."""

from pathlib import Path

import pytest

from mhseals_hardware.odroid_pwm import (
    OdroidPWMOutputs, SysfsPWMChannel, period_ns, pulse_ns, resolve_pwm_chip,
)


def fake_chip(root, number):
    chip = root / f'pwmchip{number}'
    pwm = chip / 'pwm0'
    pwm.mkdir(parents=True)
    for name in ('enable', 'duty_cycle', 'period'):
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
