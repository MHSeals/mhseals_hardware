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
DEFAULT_MOSFET_CHIP = '/dev/gpiochip3'
DEFAULT_MOSFET_LINE = 28  # GPIO3_D4, physical header pin 11


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


class MosfetEnable:
    """Hold the M2 pin-11 MOSFET enable line active while outputs are armed."""

    def __init__(self, chip=DEFAULT_MOSFET_CHIP,
                 line=DEFAULT_MOSFET_LINE, active_high=True):
        self.chip_path = str(chip)
        self.offset = int(line)
        self.active_high = bool(active_high)
        self.chip = None
        self.request = None

    def open(self):
        try:
            import gpiod
        except ImportError as error:
            raise RuntimeError(
                'python3-gpiod is required to control MOSFET enable pin 11') \
                from error
        self.gpiod = gpiod
        self.chip = gpiod.Chip(self.chip_path)
        if hasattr(self.chip, 'request_lines'):
            inactive = (gpiod.line.Value.INACTIVE if self.active_high else
                        gpiod.line.Value.ACTIVE)
            settings = gpiod.LineSettings(
                direction=gpiod.line.Direction.OUTPUT,
                output_value=inactive)
            self.request = self.chip.request_lines(
                consumer='mhseals-thruster-mosfet',
                config={self.offset: settings})
        else:
            self.request = self.chip.get_line(self.offset)
            self.request.request(
                consumer='mhseals-thruster-mosfet',
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0 if self.active_high else 1])
        return self

    def set_enabled(self, enabled):
        physical_value = bool(enabled) == self.active_high
        if hasattr(self.request, 'set_value'):
            try:
                value = (self.gpiod.line.Value.ACTIVE if physical_value else
                         self.gpiod.line.Value.INACTIVE)
            except AttributeError:
                value = int(physical_value)
            try:
                self.request.set_value(self.offset, value)
            except TypeError:
                self.request.set_value(value)
        else:
            self.request.set_values({self.offset: int(physical_value)})

    def close(self):
        if self.request is not None:
            self.set_enabled(False)
            self.request.release()
            self.request = None
        if self.chip is not None:
            close = getattr(self.chip, 'close', None)
            if close is not None:
                close()
            self.chip = None


class OdroidPWMOutputs:
    """Four ESC outputs with neutral-on-open and neutral-on-close safety."""

    def __init__(self, chips=DEFAULT_PWM_CHIPS, channels=None,
                 frequency_hz=50.0, neutral_us=1500,
                 mosfet_chip=DEFAULT_MOSFET_CHIP,
                 mosfet_line=DEFAULT_MOSFET_LINE,
                 mosfet_active_high=True):
        if len(chips) != 4:
            raise ValueError('exactly four PWM chip paths are required')
        channels = channels or (0, 0, 0, 0)
        if len(channels) != 4:
            raise ValueError('exactly four PWM channel numbers are required')
        self.channels = [SysfsPWMChannel(chip, channel)
                         for chip, channel in zip(chips, channels)]
        self.frequency_hz = float(frequency_hz)
        self.neutral_us = int(neutral_us)
        self.mosfet = (MosfetEnable(mosfet_chip, mosfet_line,
                                    mosfet_active_high)
                        if mosfet_chip else None)

    def open(self):
        opened = []
        try:
            for channel in self.channels:
                channel.open().configure(self.frequency_hz, self.neutral_us)
                opened.append(channel)
            if self.mosfet is not None:
                self.mosfet.open()
                self.mosfet.set_enabled(True)
        except Exception:
            if self.mosfet is not None:
                self.mosfet.close()
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
        if self.mosfet is not None:
            self.mosfet.close()
        for channel in self.channels:
            try:
                channel.set_pulse_width(self.neutral_us)
            finally:
                channel.close()
