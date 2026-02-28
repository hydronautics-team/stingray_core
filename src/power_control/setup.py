from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'power_control'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='GPIO-based power control (VMA enable) for Raspberry Pi using libgpiod python bindings.',
    license='TODO',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'power_gpio_node = power_control.power_gpio_node:main',
        ],
    },
)
