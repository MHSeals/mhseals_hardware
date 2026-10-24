"""Pure controller domains shared by startup and parameter preflight."""
from array import array
import math

VECTOR_GAINS = ('kp', 'ki', 'integral_limit')
SCALAR_GAINS = ('drift_time_constant', 'odom_timeout', 'drift_command_limit')
GAIN_KEYS = VECTOR_GAINS + SCALAR_GAINS
RESTART_KEYS = ('config_file', 'input_topic', 'output_topic', 'odom_topic')


def nonnegative(value, name, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (
            value <= 0 if positive else value < 0):
        raise ValueError(f'{name} must be finite and {"positive" if positive else "nonnegative"}')


def validate_controller(values):
    for name in VECTOR_GAINS:
        value = values[name]
        if not isinstance(value, (list, tuple, array)) or len(value) != 3:
            raise ValueError(f'{name} needs three numbers')
        for item in value:
            nonnegative(item, name)
    for name in SCALAR_GAINS + ('command_timeout',):
        nonnegative(values[name], name, positive=name in ('odom_timeout', 'command_timeout'))
    if type(values['feedback']) is not bool:
        raise ValueError('feedback must be a boolean')
    return values
