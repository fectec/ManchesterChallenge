import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_control')
    param_file = os.path.join(pkg_share, 'config', 'point_open_loop_path.yaml')
    
    point_open_loop_path_generator_node = Node(
        package='puzzlebot_control',
        executable='point_open_loop_path_generator.py',
        name='point_open_loop_path_generator',
        parameters=[param_file]
    )
    
    point_open_loop_controller_node = Node(
        package='puzzlebot_control',
        executable='point_open_loop_controller.py',
        name='point_open_loop_controller',
        parameters=[{
            'timer_period': 0.1
        }]
    )
    
    return LaunchDescription([
        point_open_loop_path_generator_node,
        point_open_loop_controller_node
    ])