import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ushan Fernando',
    maintainer_email='ushanfernando123@gmail.com',
    description='Robot controller for the autonomous mobile robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = robot_controller.robot_controller:main',
            'robot_serial = robot_controller.robot_serial:main',
            'robot_pid = robot_controller.robot_pid:main',
            'robot_odometry = robot_controller.robot_odometry:main',
        ],
    },
)
