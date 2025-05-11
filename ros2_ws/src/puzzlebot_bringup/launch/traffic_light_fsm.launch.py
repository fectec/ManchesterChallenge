import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('puzzlebot_bringup')
    shared_param_file = os.path.join(pkg_bringup, 'config', 'traffic_light_fsm_config.yaml')

    pid_point_controller_launch_dir = os.path.join(
        get_package_share_directory('puzzlebot_control'),
        'launch',
        'pid_point_controller.launch.py'  
    )

    pid_point_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pid_point_controller_launch_dir)
    )

    color_blob_detection_node = Node(
        package='puzzlebot_vision',
        executable='color_blob_detection',
        name='color_blob_detection',
        parameters=[shared_param_file]
    )

    traffic_light_fsm_node = Node(
        package='puzzlebot_behavior',
        executable='traffic_light_fsm',
        name='traffic_light_fsm',
        parameters=[shared_param_file]
    )

    return LaunchDescription([
        pid_point_controller_launch,
        color_blob_detection_node,
        traffic_light_fsm_node
    ])