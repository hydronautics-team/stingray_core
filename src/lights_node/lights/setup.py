from setuptools import find_packages, setup

package_name = 'lights'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS 2 control and driver nodes for a simple light system.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lantern_control = lights.light_system:main',
            'light_check = lights.check:main',
        ],
    },
)
