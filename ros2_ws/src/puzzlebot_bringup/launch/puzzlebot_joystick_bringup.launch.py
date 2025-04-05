#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    puzzlebot_description_dir = get_package_share_directory('puzzlebot_description')
    puzzlebot_controller_dir = get_package_share_directory('puzzlebot_controller')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puzzlebot_description_dir, 'launch', 'gazebo.launch.py')
        )
    )

    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puzzlebot_controller_dir, 'launch', 'joystick_teleop.launch.py')
        )
    )

    simulation_velocity_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puzzlebot_controller_dir, 'launch', 'simulation_velocity_controller.launch.py')
        )
    )

    velocity_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puzzlebot_controller_dir, 'launch', 'velocity_controller.launch.py')
        )
    )

    return LaunchDescription([
        gazebo_launch,
        joystick_teleop_launch,
        simulation_velocity_controller_launch,
        velocity_controller_launch,
    ])

if __name__ == '__main__':
    generate_launch_description()