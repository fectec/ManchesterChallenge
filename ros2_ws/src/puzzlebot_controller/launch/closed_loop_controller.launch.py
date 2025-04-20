from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    localization_node = Node(
        package='puzzlebot_controller',
        executable='localization.py',
        name='localization',
        parameters=[{
            'wheel_base': 0.18,
            'wheel_radius': 0.05,
            'update_rate': 200.0,
            'integration_period': 0.01
        }]
    )
        
    point_controller_node = Node(
        package='puzzlebot_controller',
        executable='point_controller.py',
        name='point_controller',
        parameters=[{
            'Kp_V': 1.25,
            'Ki_V': 0.0008,
            'Kd_V': 0.0004,
            'Kp_Omega': 1.6,
            'Ki_Omega': 0.0008,
            'Kd_Omega': 0.0004,
            'position_tolerance': 0.01,
            'angle_tolerance': 0.01,
            'update_rate': 200.0
        }]
    )

    return LaunchDescription([
        localization_node,
        point_controller_node
    ])