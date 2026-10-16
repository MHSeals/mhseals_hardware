"""Find which physical PWM output drives each thruster by pulsing them one at a time.

The operator watches the boat (tied up, propellers clear and submerged) and
answers which thruster moved, which way its propeller turned, and whether its
thrust pointed the expected way. No sensors are involved: the result is the
``channel_map`` saved to the hardware config.
"""

import argparse
import time

from rich.console import Console
from rich.panel import Panel

from mhseals_hardware.configuration import configure_args, load_config, save_config
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.menu import Cancelled, select_option  # noqa: F401 (Cancelled re-exported)
from mhseals_hardware.outputs import BACKENDS, make_outputs
from mhseals_hardware.thruster_mixer import (
    NEUTRAL_PWM, POSITIONS, channel_map_from_observations, effort_to_pulse, validate_channel_map,
)

# A gentle pulse is enough to see which thruster moves.
IDENTIFY_EFFORT = 0.15
PULSE_SECONDS = 0.75
SETTLE_SECONDS = 0.5
# ESCs need uninterrupted neutral after power-up before they accept throttle.
ARMING_SECONDS = 5.0
ABORT_KEYS = ('space', 'q', 'escape')
POSITION_NAMES = {'fl': 'front-left', 'fr': 'front-right', 'rr': 'rear-right', 'rl': 'rear-left'}


def hold(outputs, pulses, seconds, keys, clock=time.monotonic):
    """Keep writing ``pulses`` for ``seconds``; Space/Q/Esc stops at neutral and raises."""
    deadline = clock() + seconds
    while True:
        outputs.set_pulse_widths(pulses)
        remaining = deadline - clock()
        if remaining <= 0:
            return
        if keys.read(timeout=min(0.05, remaining)) in ABORT_KEYS:
            outputs.neutral()
            raise Cancelled('identification cancelled; outputs neutral')


def identify_thrusters(outputs, prompt, choose, emit=lambda *args, **kwargs: None,
                       open_keys=KeyReader, clock=time.monotonic, effort=IDENTIFY_EFFORT):
    """Pulse each physical output and record the operator's observations.

    ``outputs`` must already be open. ``prompt(message)`` waits for Enter,
    ``choose(title, options, shortcuts)`` returns an option, and ``emit(event,
    **details)`` records progress. Returns ``(channel_map, rotations)``, where
    ``rotations`` maps each position to 'CW' or 'CCW'. Outputs are neutral on
    every exit path.
    """
    neutral = [NEUTRAL_PWM] * 4

    def hold_for(pulses, seconds):
        with open_keys() as keys:
            hold(outputs, pulses, seconds, keys, clock)

    observations, rotations = {}, {}
    try:
        hold_for(neutral, ARMING_SECONDS)
        for channel in range(1, 5):
            prompt(f'Press Enter to pulse physical output {channel} at {effort:.0%}... ')
            pulses = list(neutral)
            pulses[channel - 1] = effort_to_pulse(effort)
            emit('identification_start', physical_channel=channel)
            hold_for(pulses, PULSE_SECONDS)
            hold_for(neutral, SETTLE_SECONDS)
            emit('identification_stop', physical_channel=channel)

            remaining = tuple(p for p in POSITIONS if p not in observations.values())
            position = choose('Which thruster moved?', remaining,
                              {str(index + 1): index for index in range(len(remaining))})
            rotation = choose('Prop rotation (viewed from propeller toward motor)',
                              ('CCW', 'CW'), {'c': 0, 'w': 1})
            # Positive effort should push water aft at the bow thrusters and
            # forward at the stern ones (the mixer's sign convention).
            expected = 'aft' if position.startswith('f') else 'forward'
            if choose(f'Did positive local thrust point {expected}?',
                      ('yes', 'no'), {'y': 0, 'n': 1}) != 'yes':
                raise RuntimeError(f'{POSITION_NAMES[position]} polarity is reversed')
            observations[channel] = position
            rotations[position] = rotation
            emit('identification_result', physical_channel=channel, position=position,
                 rotation=rotation, rotation_view='from propeller toward motor')
    finally:
        outputs.neutral()
    return channel_map_from_observations(observations), rotations


def choose_mapping(choose):
    """Enter the channel map by hand (no pulses): the output wired to each position."""
    available, mapping = [1, 2, 3, 4], []
    for position in POSITIONS:
        chosen = int(choose(f'Physical output for {POSITION_NAMES[position]}',
                            tuple(str(number) for number in available),
                            {str(number): index for index, number in enumerate(available)}))
        mapping.append(chosen)
        available.remove(chosen)
    return validate_channel_map(mapping)


def save_mapping(console, channel_map, path):
    console.print(f'FL, FR, RR, RL -> physical outputs {list(channel_map)}')
    if console.input('Save this mapping? [y/N] ').strip().lower() != 'y':
        return
    # Only the mapping is persisted; one-off overrides such as --output-backend are not.
    config = dict(load_config(path), channel_map=list(channel_map))
    console.print(f'Saved {save_config(config, path)}')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--manual', action='store_true',
                        help='enter the mapping by hand instead of pulsing outputs')
    parser.add_argument('--output-backend', choices=BACKENDS,
                        help='odroid pins or a simulator topic (default from the config)')
    parsed = configure_args(parser, args)
    console = Console()
    choose = lambda title, options, shortcuts: select_option(console, title, options, shortcuts)
    if parsed.manual:
        try:
            save_mapping(console, choose_mapping(choose), parsed.config)
        except (RuntimeError, KeyboardInterrupt) as error:
            console.print(f'[yellow]Stopped: {error or "interrupted"}; mapping unchanged.[/]')
        return
    console.print(Panel(
        'Boat secured; all propellers clear and submerged; power cutoff within reach.\n'
        'Each output is pulsed alone. Space, Q or Esc during a pulse returns to neutral.\n'
        'Type ARM to start; anything else cancels.',
        title='Thruster identification', style='yellow'))
    if console.input('Confirm: ').strip() != 'ARM':
        return
    outputs = make_outputs(parsed.hardware_config).open()
    try:
        channel_map, rotations = identify_thrusters(outputs, console.input, choose)
    except (RuntimeError, KeyboardInterrupt) as error:  # Cancelled is a RuntimeError
        console.print(f'[yellow]Stopped: {error or "interrupted"}; mapping unchanged.[/]')
        return
    finally:
        outputs.close()
    console.print(f'Propeller rotations: {rotations}')
    save_mapping(console, channel_map, parsed.config)


if __name__ == '__main__':
    main()
