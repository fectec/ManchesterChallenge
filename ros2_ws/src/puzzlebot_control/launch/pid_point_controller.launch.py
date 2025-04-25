import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_control = get_package_share_directory('puzzlebot_control')
    shared_param_file = os.path.join(pkg_control, 'config', 'pid_point_controller_config.yaml')

    odometry_localization_node = Node(
        package='puzzlebot_control',
        executable='odometry_localization.py',
        name='odometry_localization',
        parameters=[shared_param_file]
    )
    
    pid_path_generator_node = Node(
        package='puzzlebot_control',
        executable='pid_path_generator.py',
        name='pid_path_generator',
        parameters=[shared_param_file]
    )

    pid_point_controller_node = Node(
        package='puzzlebot_control',
        executable='pid_point_controller.py',
        name='pid_point_controller',
        parameters=[shared_param_file]
    )

    return LaunchDescription([
        odometry_localization_node,
        pid_path_generator_node,
        pid_point_controller_node
    ])