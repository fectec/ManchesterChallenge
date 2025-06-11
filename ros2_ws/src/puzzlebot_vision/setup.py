from setuptools import find_packages, setup

package_name = 'puzzlebot_vision'

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
    description='This package implements computer vision algorithms for the Puzzlebot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_blob_detection = puzzlebot_vision.color_blob_detection:main',
            'line_detection = puzzlebot_vision.line_detection:main',
            'adaptive_image_filter = puzzlebot_vision.adaptive_image_filter:main',
        ],
    },
)