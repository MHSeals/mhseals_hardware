"""The thruster stack: driver and command_mux, plus the velocity controller with controller:=true.

    ros2 launch mhseals_hardware drive.launch.py [output_backend:=topic] [use_sim_time:=true]
        [controller:=true] [config_file:=PATH]

Every node reads its topics from the config (topics:), so nothing is remapped here.
output_backend empty (the default) keeps the config's value.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

PACKAGE = 'mhseals_hardware'


def nodes(context):
    arg = lambda name: context.launch_configurations[name]  # noqa: E731
    common = {'config_file': arg('config_file'),
              'use_sim_time': arg('use_sim_time').lower() == 'true'}
    driver = dict(common)
    if arg('output_backend'):
        driver['output_backend'] = arg('output_backend')
    # The driver first, so it holds neutral before any command reaches it.
    started = [Node(package=PACKAGE, executable='thruster_pwm_node', output='screen',
                    parameters=[driver])]
    if arg('controller').lower() == 'true':
        started.append(Node(package=PACKAGE, executable='calibrated_velocity_controller',
                            output='screen', parameters=[common]))
    started.append(Node(package=PACKAGE, executable='command_mux', output='screen',
                        parameters=[common]))
    return started


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_backend', default_value='',
                              description='odroid or topic; empty keeps the config value'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('controller', default_value='false',
                              description='true also starts calibrated_velocity_controller '
                                          '(topics.auto -> topics.effort)'),
        DeclareLaunchArgument('config_file', default_value='',
                              description='hardware YAML; empty = $MHSEALS_HARDWARE_CONFIG '
                                          'or ~/.config/mhseals/hardware.yaml'),
        OpaqueFunction(function=nodes),
    ])
