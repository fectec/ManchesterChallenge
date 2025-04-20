from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_control_real'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rovestrada',
    maintainer_email='rovestrada@gmail.com',
    description='This ROS 2 package provides a dynamic setpoint generator designed for controlling a real DC motor.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'setpoint = motor_control_real.setpoint:main',
        ],
    },
)