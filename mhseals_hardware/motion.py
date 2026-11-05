"""The motion topology: which nodes one arm starts, in order, and their argv.

Arming (mhseals_testing) is the only starter, so this is a function and not a launch file: a
second way to start the driver would power the ESCs outside the arm gate. Every node reads its
lanes from the ``topics`` section of ``config_path`` (the arm snapshot), so no argv carries a
remap: in velocity mode the controller turns ``topics.auto`` into ``topics.effort``, the mux
auto input; in effort mode nothing reads ``topics.auto`` and an effort publisher drives
``topics.effort`` itself. The mux output ``topics.applied`` reaches the driver in both modes.

Atlas #0 is answered by the mux reading ``topics.effort`` (config/default.yaml), whoever starts
the nodes: landing this branch decides #0 (nothing on ``topics.auto`` drives without the
controller), so it lands only once the owner decides #0, in one deploy with the effort
publishers moving to ``topics.effort`` (#5). On a veto the mux argv in arming gets
``-r <topics.effort>:=<topics.auto>`` (topics_section refuses them equal), which this carries
once arming adopts it.
"""

PACKAGE = 'mhseals_hardware'


def children(config_path, velocity, use_sim_time):
    """``[(name, argv)]`` in start order: the driver first so it holds neutral before any command,
    then the controller (only when ``velocity``), the mux last. Stopping runs it reversed.

    The snapshot is the one source of wiring, ``output_backend`` included.
    """
    common = ['--ros-args', '-p', f'config_file:={config_path}',
              '-p', f'use_sim_time:={str(bool(use_sim_time)).lower()}']

    def node(executable):
        return ['ros2', 'run', PACKAGE, executable, *common]

    controller = [('velocity', node('calibrated_velocity_controller'))] if velocity else []
    return [('hardware', node('thruster_pwm_node')),
            *controller, ('mux', node('command_mux'))]
