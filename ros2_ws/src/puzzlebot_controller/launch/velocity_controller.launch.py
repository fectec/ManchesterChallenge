from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    real_velocity_controller_node = Node(
        package="puzzlebot_controller",
        executable="real_velocity_controller.py", 
        name="real_velocity_controller",
        output="screen",
        parameters=[{
            "wheel_radius": 0.05,
            "wheel_separation": 0.18
        }]
    )

    return LaunchDescription([
        real_velocity_controller_node
    ])