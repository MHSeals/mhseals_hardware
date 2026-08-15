"""ROS-independent planar thruster mixing and output mapping."""

NEUTRAL_PWM = 1500
PWM_SCALE = 400
MIN_PWM = 1100
MAX_PWM = 1900
POSITIONS = ('fl', 'fr', 'rr', 'rl')
THRUSTER_MIXER = (
    (-1.0, 1.0, 1.0),
    (-1.0, -1.0, -1.0),
    (1.0, -1.0, 1.0),
    (1.0, 1.0, -1.0),
)


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
    values = tuple(float(value) for value in values)
    if not all(value == value and abs(value) != float('inf')
               for value in values):
        raise ValueError('thruster_matrix values must be finite')
    return tuple(tuple(values[index:index + 3])
                 for index in range(0, 12, 3))


def validate_channel_map(values):
    """Validate canonical-to-physical output numbers."""
    channels = tuple(int(value) for value in values)
    if len(channels) != 4 or sorted(channels) != [1, 2, 3, 4]:
        raise ValueError('channel_map must be a permutation of [1, 2, 3, 4]')
    return channels


def map_channels(values, channel_map):
    """Map canonical FL,FR,RR,RL values onto four physical Pico outputs."""
    mapped = [None] * 4
    for value, physical_channel in zip(values, channel_map):
        mapped[physical_channel - 1] = value
    return mapped


def mix_thrusters(surge, sway, yaw, mixer=THRUSTER_MIXER):
    """Return normalized, desaturated outputs in FL, FR, RR, RL order."""
    commands = [row[0] * surge + row[1] * sway + row[2] * yaw
                for row in mixer]
    peak = max(1.0, *(abs(command) for command in commands))
    return [command / peak for command in commands]
