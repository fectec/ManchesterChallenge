from setuptools import find_packages, setup

package_name = 'puzzlebot_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy'  
    ],
    zip_safe=True,
    maintainer='fectec',
    maintainer_email='fectec151@gmail.com',
    description='Utility functions and helpers for Puzzlebot packages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)