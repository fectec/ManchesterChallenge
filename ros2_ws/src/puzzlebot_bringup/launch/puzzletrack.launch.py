import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('puzzlebot_bringup')
    shared_param_file = os.path.join(pkg_bringup, 'config', 'puzzletrack.yaml')

    color_blob_detection_node = Node(
        package='puzzlebot_vision',
        executable='color_blob_detection',
        name='color_blob_detection',
        parameters=[shared_param_file]
    )

    yolov8_recognition_node = Node(
        package='yolobot_recognition',
        executable='yolov8_recognition',
        name='yolov8_recognition',
        parameters=[shared_param_file]
    )

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

    traffic_fsm_node = Node(
        package='puzzlebot_behavior',
        executable='traffic_fsm',
        name='traffic_fsm',
        parameters=[shared_param_file]
    )

    return LaunchDescription([
        color_blob_detection_node,
        yolov8_recognition_node,
        line_detection_node,
        line_follow_controller_node,
        traffic_fsm_node
    ])