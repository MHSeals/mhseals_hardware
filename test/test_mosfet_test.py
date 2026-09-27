import pytest

from mhseals_hardware import mosfet_test


@pytest.mark.parametrize('interrupt', [False, True])
def test_enable_only_always_closes(tmp_path, monkeypatch, interrupt):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    events = []

    class Output:
        def __init__(self, chip, line, active_high):
            assert (chip, line, active_high) == ('/dev/gpiochip5', 28, True)

        def open(self):
            events.append('open')

        def set_enabled(self, value):
            events.append(value)

        def close(self):
            events.append('close')

    def sleep(seconds):
        events.append(seconds)
        if seconds == 5 and interrupt:
            raise KeyboardInterrupt

    monkeypatch.setattr(mosfet_test, 'MosfetEnable', Output)
    monkeypatch.setattr(mosfet_test, 'resolve_gpio_chip', lambda _: '/dev/gpiochip5')
    monkeypatch.setattr(mosfet_test.time, 'sleep', sleep)
    monkeypatch.setattr('builtins.input', lambda _: 'ENABLE')
    mosfet_test.main([])
    assert events == ['open', 2, True, 5, 'close']
