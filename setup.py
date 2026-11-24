from glob import glob

from setuptools import find_packages, setup


package_name = 'mhseals_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', sorted(glob('config/*.yaml'))),
        ('share/' + package_name + '/launch', ['launch/drive.launch.py']),
    ],
    install_requires=['setuptools', 'rich', 'gpiod', 'PyYAML', 'numpy'],
    zip_safe=False,
    author='MHS Seals',
    description='Native ODROID-M2 PWM thruster control for MHS Seals',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_pwm_node = mhseals_hardware.thruster_pwm_node:main',
            'thrusters = mhseals_hardware.thrusters:main',
            'calibrated_velocity_controller = mhseals_hardware.velocity_controller:main',
            'keyboard = mhseals_hardware.keyboard_control:main',
            'command_mux = mhseals_hardware.command_mux:main',
            'config_check = mhseals_hardware.config_check:main',
            'mix = mhseals_hardware.mix:main',
        ],
    },
)
