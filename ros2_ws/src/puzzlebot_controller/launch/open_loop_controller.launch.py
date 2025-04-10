import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_controller')
    param_file = os.path.join(pkg_share, 'config', 'open_loop_path_generator_params.yaml')
    
    # Node launching the open_loop_path_generator
    open_loop_path_generator_node = Node(
        package='puzzlebot_controller',
        executable='open_loop_path_generator.py',
        name='open_loop_path_generator',
        output='screen',
        parameters=[param_file]
    )
    
    # Node launching the open_loop_controller
    open_loop_controller_node = Node(
        package='puzzlebot_controller',
        executable='open_loop_controller.py',
        name='open_loop_controller',
        output='screen'
    )
    
    return LaunchDescription([
        open_loop_path_generator_node,
        open_loop_controller_node
    ])