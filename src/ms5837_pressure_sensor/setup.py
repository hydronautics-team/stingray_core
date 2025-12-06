import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ms5837_pressure_sensor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waspishraccoon',
    maintainer_email='p.v.elesin@gmail.com',
    description='ROS 2 driver package for MS5837 pressure and temperature sensor.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ms5837_node = ms5837_pressure_sensor.ms5837_component:main',
        ],
    },
)