import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_control = get_package_share_directory('puzzlebot_control')
    shared_param_file = os.path.join(pkg_control, 'config', 'open_loop_point_controller_config.yaml')

    open_loop_path_generator_node = Node(
        package='puzzlebot_control',
        executable='open_loop_path_generator.py',
        name='open_loop_path_generator',
        parameters=[shared_param_file]
    )

    open_loop_point_controller_node = Node(
        package='puzzlebot_control',
        executable='open_loop_point_controller.py',
        name='open_loop_point_controller',
        parameters=[shared_param_file]
    )

    return LaunchDescription([
        open_loop_path_generator_node,
        open_loop_point_controller_node,
    ])