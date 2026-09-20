"""Linux sysfs PWM output for four directly-connected ESCs."""

from pathlib import Path
import glob
import time


# pwmchip numbers are allocated dynamically. Platform-address globs remain
# stable and correspond to physical J2 pins 7, 12, 15, and 33 respectively.
DEFAULT_PWM_CHIPS = (
    '/sys/devices/platform/febd0030.pwm/pwm/pwmchip*',
    '/sys/devices/platform/fd8b0030.pwm/pwm/pwmchip*',
    '/sys/devices/platform/febf0030.pwm/pwm/pwmchip*',
    '/sys/devices/platform/febe0000.pwm/pwm/pwmchip*',
)


def resolve_pwm_chip(value):
    """Resolve one stable platform glob to exactly one pwmchip directory."""
    matches = tuple(glob.glob(str(value)))
    if not matches and not any(char in str(value) for char in '*?['):
        return Path(value)
    if len(matches) != 1:
        raise FileNotFoundError(
            f'expected exactly one PWM controller matching {value}, '
            f'found {len(matches)}')
    return Path(matches[0])


def period_ns(frequency_hz):
    """Convert frequency to the integer period accepted by Linux PWM."""
    frequency_hz = float(frequency_hz)
    if frequency_hz <= 0:
        raise ValueError('frequency must be positive')
    return round(1_000_000_000 / frequency_hz)


def pulse_ns(pulse_width_us, frequency_hz):
    """Validate and convert an ESC pulse width."""
    value = round(float(pulse_width_us) * 1000)
    if value < 0 or value > period_ns(frequency_hz):
        raise ValueError('pulse width must fit within one PWM period')
    return value


class SysfsPWMChannel:
    """One channel of the Linux PWM sysfs ABI."""

    def __init__(self, chip, channel=0, wait_seconds=1.0):
        self.chip_spec = str(chip)
        self.chip = Path(chip)
        self.channel_number = int(channel)
        self.path = self.chip / f'pwm{self.channel_number}'
        self.wait_seconds = wait_seconds
        self.exported_here = False

    @staticmethod
    def _write(path, value):
        Path(path).write_text(str(value), encoding='ascii')

    def open(self):
        self.chip = resolve_pwm_chip(self.chip_spec)
        self.path = self.chip / f'pwm{self.channel_number}'
        if not self.chip.is_dir():
            raise FileNotFoundError(
                f'{self.chip} is unavailable; enable the Odroid PWM overlay '
                'and pass the PWM devices into the container')
        if not self.path.exists():
            self._write(self.chip / 'export', self.channel_number)
            self.exported_here = True
            deadline = time.monotonic() + self.wait_seconds
            while not self.path.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
        if not self.path.exists():
            raise RuntimeError(f'kernel did not export {self.path}')
        return self

    def configure(self, frequency_hz, pulse_width_us):
        period = period_ns(frequency_hz)
        duty = pulse_ns(pulse_width_us, frequency_hz)
        self._write(self.path / 'enable', 0)
        # A new, shorter period cannot be installed while duty exceeds it.
        self._write(self.path / 'duty_cycle', 0)
        self._write(self.path / 'period', period)
        self._write(self.path / 'duty_cycle', duty)
        self._write(self.path / 'enable', 1)

    def set_pulse_width(self, pulse_width_us):
        self._write(self.path / 'duty_cycle',
                    round(float(pulse_width_us) * 1000))

    def close(self, unexport=False):
        if self.path.exists():
            self._write(self.path / 'enable', 0)
        if unexport and self.exported_here:
            self._write(self.chip / 'unexport', self.channel_number)


class OdroidPWMOutputs:
    """Four ESC outputs with neutral-on-open and neutral-on-close safety."""

    def __init__(self, chips=DEFAULT_PWM_CHIPS, channels=None,
                 frequency_hz=50.0, neutral_us=1500):
        if len(chips) != 4:
            raise ValueError('exactly four PWM chip paths are required')
        channels = channels or (0, 0, 0, 0)
        if len(channels) != 4:
            raise ValueError('exactly four PWM channel numbers are required')
        self.channels = [SysfsPWMChannel(chip, channel)
                         for chip, channel in zip(chips, channels)]
        self.frequency_hz = float(frequency_hz)
        self.neutral_us = int(neutral_us)

    def open(self):
        opened = []
        try:
            for channel in self.channels:
                channel.open().configure(self.frequency_hz, self.neutral_us)
                opened.append(channel)
        except Exception:
            for channel in opened:
                channel.close()
            raise
        return self

    def set_pulse_widths(self, values):
        values = tuple(values)
        if len(values) != 4:
            raise ValueError('exactly four pulse widths are required')
        for value in values:
            pulse_ns(value, self.frequency_hz)
        for channel, value in zip(self.channels, values):
            channel.set_pulse_width(value)

    def set_frequency(self, frequency_hz):
        frequency_hz = float(frequency_hz)
        for channel in self.channels:
            channel.configure(frequency_hz, self.neutral_us)
        self.frequency_hz = frequency_hz

    def neutral(self):
        self.set_pulse_widths([self.neutral_us] * 4)

    def close(self):
        for channel in self.channels:
            try:
                channel.set_pulse_width(self.neutral_us)
            finally:
                channel.close()
