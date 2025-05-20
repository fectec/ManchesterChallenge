import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    line_detection_node = Node(
        package='puzzlebot_vision',
        executable='line_detection',
        name='line_detection',
    )

    line_follow_controller_node = Node(
        package='puzzlebot_control',
        executable='line_follow_controller.py',
        name='line_follow_controller',
    )

    return LaunchDescription([
        line_detection_node,
        line_follow_controller_node
    ])