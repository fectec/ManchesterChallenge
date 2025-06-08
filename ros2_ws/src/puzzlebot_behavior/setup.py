from setuptools import find_packages, setup

package_name = 'puzzlebot_behavior'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fectec',
    maintainer_email='fectec151@gmail.com',
    description='Behavior descriptions for the Puzzlebot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_fsm = puzzlebot_behavior.traffic_light_fsm:main',
            'traffic_fsm = puzzlebot_behavior.traffic_fsm:main',
        ],
    },
)