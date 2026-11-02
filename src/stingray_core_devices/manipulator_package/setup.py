from glob import glob
from setuptools import find_packages, setup

package_name = 'manipulator_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavel',
    maintainer_email='aleksejberes06@gmail.com',
    description='ROS2 package for manipulator PWM control via serial port',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'manipulator_sub_node = manipulator_package.manipulator_sub_node:main'
        ],
    },
)
