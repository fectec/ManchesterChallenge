import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('puzzlebot_bringup')
    shared_param_file = os.path.join(pkg_bringup, 'config', 'line_following_config.yaml')

    line_detection_node = Node(
        package='puzzlebot_vision',
        executable='line_detection',
        name='line_detection',
        parameters=[shared_param_file]
    )

    line_follow_controller_node = Node(
        package='puzzlebot_control',
        executable='line_follow_controller.py',
        name='line_follow_controller',
        parameters=[shared_param_file]
    )



    return LaunchDescription([

        line_detection_node,
        line_follow_controller_node
    ])