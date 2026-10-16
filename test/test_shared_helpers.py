"""Shared command and CLI helpers, without ROS or hardware."""
import ast
from pathlib import Path

from mhseals_hardware.configuration import parse_csv
from mhseals_hardware.thruster_mixer import (
    AXES, MAX_PWM, MIN_PWM, NEUTRAL_PWM, axis_command, effort_to_pulse,
)


def test_axis_commands_and_pulse_conversion():
    assert axis_command() == (0.0, 0.0, 0.0)
    assert axis_command('sway', -0.5) == (0.0, -0.5, 0.0)
    assert axis_command((0.1, 0.2, 0.3)) == (0.1, 0.2, 0.3)
    assert effort_to_pulse(0) == NEUTRAL_PWM
    assert effort_to_pulse(0.15) == 1560
    assert (effort_to_pulse(5), effort_to_pulse(-5)) == (MAX_PWM, MIN_PWM)


def test_parse_csv():
    assert parse_csv(' a, b ') == ('a', 'b')
    assert parse_csv('1,2,3,4', int) == (1, 2, 3, 4)


def test_velocity_controller_reads_the_saved_gain_mapping():
    # Regression: the saved velocity_gain is {axis: {axis: gain}}; iterating the
    # mapping itself yielded key strings and crashed the controller on startup.
    source = Path(__file__).parents[1] / 'mhseals_hardware/velocity_controller.py'
    function = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == '_matrix')
    scope = {'AXES': AXES}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
    gain = {row: {column: float(i * 3 + j) for j, column in enumerate(AXES)}
            for i, row in enumerate(AXES)}
    assert scope['_matrix'](gain) == ((0.0, 1.0, 2.0), (3.0, 4.0, 5.0), (6.0, 7.0, 8.0))
