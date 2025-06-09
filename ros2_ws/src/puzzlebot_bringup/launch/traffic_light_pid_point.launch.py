import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('puzzlebot_bringup')
    shared_param_file = os.path.join(pkg_bringup, 'config', 'traffic_light_fsm_config.yaml')

    color_blob_detection_node = Node(
        package='puzzlebot_vision',
        executable='color_blob_detection',
        name='color_blob_detection',
        parameters=[shared_param_file]
    )

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

    traffic_light_fsm_node = Node(
        package='puzzlebot_behavior',
        executable='traffic_light_fsm',
        name='traffic_light_fsm',
        parameters=[shared_param_file]
    )

    return LaunchDescription([
        color_blob_detection_node,
        odometry_localization_node,
        pid_path_generator_node,
        pid_point_controller_node,
        traffic_light_fsm_node
    ])