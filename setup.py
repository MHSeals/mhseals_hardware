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
    ],
    install_requires=['setuptools', 'rich', 'gpiod'],
    zip_safe=True,
    author='MHS Seals',
    description='Native ODROID-M2 PWM thruster control for MHS Seals',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_pwm_node = mhseals_hardware.thruster_pwm_node:main',
            'thruster_test = mhseals_hardware.thruster_test_tui:main',
            'boat_test = mhseals_hardware.boat_test:main',
            'boat_manual = mhseals_hardware.manual_control:main',
        ],
    },
)
