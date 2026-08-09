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
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    author='MHS Seals',
    description='Pico serial thruster control for the MHS Seals boat',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_serial_node = '
            'mhseals_hardware.thruster_serial_node:main',
        ],
    },
)
