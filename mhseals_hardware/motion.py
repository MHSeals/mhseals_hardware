"""The motion topology: which nodes one arm starts, in order, and their argv.

Arming (mhseals_testing) is the only starter, so this is a function and not a launch file: a
second way to start the driver would power the ESCs outside the arm gate. Every node reads its
lanes from the ``topics`` section of ``config_path`` (the arm snapshot), so no argv carries a
remap: in velocity mode the controller turns ``topics.auto`` into ``topics.effort``, the mux
auto input; in effort mode nothing reads ``topics.auto`` and an effort publisher drives
``topics.effort`` itself. The mux output ``topics.applied`` reaches the driver in both modes.

Effort mode here answers atlas #0 as recommended (nothing on ``topics.auto`` drives without the
controller), so arming adopts this only once the owner decides #0. If the owner vetoes it and
keeps effort on ``topics.auto``, effort mode is one argv override here: the mux gets
``-r <topics.effort>:=<topics.auto>``, read from the snapshot.
"""

PACKAGE = 'mhseals_hardware'


def children(config_path, velocity, use_sim_time, outputs):
    """``[(name, argv)]`` in start order: the driver first so it holds neutral before any command,
    then the controller (only when ``velocity``), the mux last. Stopping runs it reversed.

    ``outputs`` is the driver's ``output_backend``.
    """
    common = ['--ros-args', '-p', f'config_file:={config_path}',
              '-p', f'use_sim_time:={str(bool(use_sim_time)).lower()}']

    def node(executable, *extra):
        return ['ros2', 'run', PACKAGE, executable, *common, *extra]

    controller = [('velocity', node('calibrated_velocity_controller'))] if velocity else []
    return [('hardware', node('thruster_pwm_node', '-p', f'output_backend:={outputs}')),
            *controller, ('mux', node('command_mux'))]
