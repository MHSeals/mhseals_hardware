"""Pico USB-serial controller for four bidirectional ESCs."""

from machine import Pin, PWM
import select
import sys
import time


LED_PIN = Pin('LED', Pin.OUT)
ESTOP_PIN = Pin(15, Pin.OUT)

# Standard physical output order: 1=front-left, 2=front-right,
# 3=rear-right, 4=rear-left. ROS can remap nonstandard wiring before sending.
THRUSTER_PINS = (11, 12, 13, 14)
FREQUENCY = 50
NEUTRAL_US = 1500
MIN_US = 1100
MAX_US = 1900
MAX_DUTY = 65535
PERIOD_US = 1_000_000 // FREQUENCY
COMMAND_TIMEOUT_MS = 500


def pulse_width_to_duty(pulse_width):
    """Convert an ESC pulse width in microseconds to Pico PWM duty."""
    return int(pulse_width / PERIOD_US * MAX_DUTY)


def set_thrusters(channels, pulse_widths):
    """Apply one physical-output command to the PWM channels."""
    for channel, pulse_width in zip(channels, pulse_widths):
        pulse_width = max(MIN_US, min(MAX_US, pulse_width))
        channel.duty_u16(pulse_width_to_duty(pulse_width))


def parse_command(line):
    """Parse four comma-separated physical-output PWM values."""
    values = [int(value.strip()) for value in line.split(',')]
    if len(values) != len(THRUSTER_PINS):
        raise ValueError('expected four PWM values')
    return values


def main():
    """Arm the ESCs and process commands until the Pico is reset."""
    channels = []
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    try:
        for gpio in THRUSTER_PINS:
            channel = PWM(Pin(gpio))
            channel.freq(FREQUENCY)
            channels.append(channel)

        set_thrusters(channels, [NEUTRAL_US] * 4)
        ESTOP_PIN.on()
        LED_PIN.on()
        time.sleep(2)
        last_command = time.ticks_ms()

        while True:
            if poller.poll(10):
                try:
                    command = parse_command(sys.stdin.readline().strip())
                    set_thrusters(channels, command)
                    last_command = time.ticks_ms()
                except (ValueError, TypeError):
                    set_thrusters(channels, [NEUTRAL_US] * 4)

            if time.ticks_diff(time.ticks_ms(), last_command) > COMMAND_TIMEOUT_MS:
                set_thrusters(channels, [NEUTRAL_US] * 4)
    finally:
        set_thrusters(channels, [NEUTRAL_US] * len(channels))
        for channel in channels:
            channel.deinit()
        LED_PIN.off()
        ESTOP_PIN.off()


if __name__ == '__main__':
    main()
