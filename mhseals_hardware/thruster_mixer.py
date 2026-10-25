"""ROS-independent planar thruster mixing and output mapping.

Conventions used throughout the package:
  * Body axes follow ROS REP 103: surge is +x forward, sway is +y port/left,
    yaw is +z counterclockwise seen from above.
  * Commands are normalized effort in [-1, 1] per axis, not m/s or rad/s.
  * Thrusters are listed in canonical order FL, FR, RR, RL. ``channel_map``
    says which physical PWM output (1-4) drives each canonical position.
"""

import math

AXES = ('surge', 'sway', 'yaw')

# ESC pulse widths in microseconds: 1500 is stopped, +/-400 is full effort.
NEUTRAL_PWM = 1500
PWM_SCALE = 400
MIN_PWM = 1100
MAX_PWM = 1900

POSITIONS = ('fl', 'fr', 'rr', 'rl')
# Rows are thrusters in POSITIONS order; columns are surge, sway, yaw.
THRUSTER_MIXER = (
    (-1.0, 1.0, 1.0),
    (-1.0, -1.0, -1.0),
    (1.0, -1.0, 1.0),
    (1.0, 1.0, -1.0),
)
# Rows and columns surge, sway, yaw: the fitted correction for cross-coupling.
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def axis_command(axis=None, value=0.0):
    """Return a (surge, sway, yaw) command.

    ``axis`` is an axis name driven at ``value``, an explicit three-value
    command, or None for neutral.
    """
    if isinstance(axis, (tuple, list)):
        return tuple(float(component) for component in axis)
    return tuple(float(value) if name == axis else 0.0 for name in AXES)


def effort_to_pulse(effort):
    """Convert one normalized thruster effort into a clamped ESC pulse (us)."""
    return max(MIN_PWM, min(MAX_PWM, round(NEUTRAL_PWM + PWM_SCALE * effort)))


def channel_map_from_observations(observations):
    """Invert {physical channel: canonical position} into canonical order."""
    if set(observations) != {1, 2, 3, 4}:
        raise ValueError('all four physical outputs must be identified')
    if set(observations.values()) != set(POSITIONS):
        raise ValueError('each canonical position must be assigned exactly once')
    return tuple(next(channel for channel, position in observations.items()
                      if position == expected) for expected in POSITIONS)


def validate_mixer(values):
    """Return a four-row mixer from a flat, finite 12-value sequence."""
    if len(values) != 12:
        raise ValueError('thruster_matrix must contain exactly 12 values')
    if any(type(value) not in (int, float) for value in values):
        raise ValueError('matrix values must be numbers')
    values = tuple(float(value) for value in values)
    if not all(value == value and abs(value) != float('inf')
               for value in values):
        raise ValueError('thruster_matrix values must be finite')
    return tuple(tuple(values[index:index + 3])
                 for index in range(0, 12, 3))


def validate_decoupling(values):
    """Return a 3x3 decoupling matrix from a flat, finite 9-value sequence."""
    if len(values) != 9:
        raise ValueError('decoupling must contain exactly 9 values')
    if any(type(value) not in (int, float) for value in values):
        raise ValueError('matrix values must be numbers')
    values = tuple(float(value) for value in values)
    if not all(value == value and abs(value) != float('inf') for value in values):
        raise ValueError('decoupling values must be finite')
    return tuple(tuple(values[index:index + 3]) for index in range(0, 9, 3))


def decouple(command, decoupling=IDENTITY, negative=None):
    """Apply the decoupling: e' = C e, so asking for one axis yields only that axis.

    With ``negative``, an axis commanded below zero takes its column from that
    matrix instead. Coupling that acts in one direction only (drag at the leading
    corner of the hull, say) needs a different column for each sign; a single
    linear C would fix one direction and worsen the other.
    """
    negative = decoupling if negative is None else negative
    return tuple(sum((row if command[j] >= 0 else below)[j] * command[j] for j in range(3))
                 for row, below in zip(decoupling, negative))


def validate_channel_map(values):
    """Validate canonical-to-physical output numbers."""
    if any(type(value) is not int for value in values):
        raise ValueError('channel_map must contain integers')
    channels = tuple(values)
    if len(channels) != 4 or sorted(channels) != [1, 2, 3, 4]:
        raise ValueError('channel_map must be a permutation of [1, 2, 3, 4]')
    return channels


def validate_reversed(values):
    """Four booleans in FL, FR, RR, RL order: True flips that thruster's effort."""
    if len(values) != 4 or any(type(value) is not bool for value in values):
        raise ValueError('reversed must contain four booleans (FL, FR, RR, RL)')
    return tuple(values)


def validate_reverse_ratio(value):
    """Full reverse over full forward thrust, in (0, 1]: one ratio for all four thrusters."""
    if type(value) not in (int, float) or not 0.0 < value <= 1.0:
        raise ValueError('reverse_thrust_ratio must be a number in (0, 1]')
    return float(value)


def thruster_pulses(command, mixer, channel_map, reversed=(False,) * 4, decoupling=IDENTITY,
                    decoupling_negative=None, reverse_ratio=1.0):
    """Effort command -> ESC pulse per physical output (outputs 1-4 in order).

    Decouple the axes, mix and desaturate, flip reversed thrusters (a motor
    wired or a propeller mounted backwards), convert to pulses, then route each
    position to its output. Efforts are normalised, so the command is clamped
    to [-1, 1] first: a huge finite value must not overflow the mixer into NaN.
    ``reverse_ratio`` is the thrust below 1500 us relative to above it.
    """
    command = tuple(max(-1.0, min(1.0, component)) for component in command)
    # The weak side of a flipped thruster is its positive canonical effort.
    ratios = [1.0 / reverse_ratio if flip else reverse_ratio for flip in reversed]
    efforts = [-effort if flip else effort
               for effort, flip in zip(mix_thrusters(*command, mixer, ratios, decoupling,
                                                    decoupling_negative), reversed)]
    return map_channels([effort_to_pulse(effort) for effort in efforts], channel_map)


def map_channels(values, channel_map):
    """Map canonical FL, FR, RR, RL values onto physical PWM outputs."""
    mapped = [None] * 4
    for value, physical_channel in zip(values, channel_map):
        mapped[physical_channel - 1] = value
    return mapped


def mix_thrusters(surge, sway, yaw, mixer=THRUSTER_MIXER, reverse_ratio=(1.0,) * 4,
                  decoupling=IDENTITY, decoupling_negative=None):
    """Return normalized, desaturated outputs in FL, FR, RR, RL order.

    Mixed in thrust, not effort: thrust goes as e|e| (``reverse_ratio`` weaker
    below zero, per thruster), so each thruster sums the thrust every axis alone
    (decoupled as before) would ask of it and takes the effort that produces the
    total. A single axis is unchanged; a combined command is the sum of its
    single-axis wrenches, so a diagonal neither turns (unequal forward and reverse
    thrust on a diagonal pair) nor bends toward the larger axis. Summing efforts
    instead gets both wrong.
    """
    def thrust(effort, ratio):
        return effort * abs(effort) * (ratio if effort < 0 else 1.0)

    def effort(force, ratio):
        return math.copysign(math.sqrt(abs(force) / (ratio if force < 0 else 1.0)), force)

    parts = [decouple(axis_command(axis, value), decoupling, decoupling_negative)
             for axis, value in zip(AXES, (surge, sway, yaw))]
    commands = [effort(sum(thrust(sum(gain * part[k] for k, gain in enumerate(row)), ratio)
                           for part in parts), ratio)
                for row, ratio in zip(mixer, reverse_ratio)]
    # Scale every thruster down together when any would exceed full effort: every
    # thrust shrinks by the same factor, so the wrench keeps its direction.
    peak = max(1.0, *(abs(command) for command in commands))
    return [command / peak for command in commands]
