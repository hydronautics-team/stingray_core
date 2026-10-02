from setuptools import find_packages, setup

package_name = 'stingray_core_doctor'

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
    maintainer='root',
    maintainer_email='kirillch030309@gmail.com',
    description='Pre-flight diagnostic tool for Stingray Core',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stingray_core_doctor = stingray_core_doctor.cli:main'
        ],
    },
)
