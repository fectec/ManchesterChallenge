import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_control')
    param_file = os.path.join(pkg_share, 'config', 'point_PID_path.yaml')

    odometry_localization_node = Node(
        package='puzzlebot_control',
        executable='odometry_localization.py',
        name='odometry_localization',
        parameters=[{
            'wheel_base': 0.18,
            'wheel_radius': 0.05,
            'update_rate': 200.0,
            'integration_period': 0.01
        }]
    )
        
    point_PID_controller_node = Node(
        package='puzzlebot_control',
        executable='point_PID_controller.py',
        name='point_PID_controller',
        parameters=[{
            'Kp_V': 0.15,
            'Ki_V': 0.0,
            'Kd_V': 0.0,
            'Kp_Omega': 0.1,
            'Ki_Omega': 0.0,
            'Kd_Omega': 0.0,
            'goal_tolerance': 0.1,
            'heading_tolerance': 0.1,
            'min_linear_vel': 0.11, 
            'max_linear_vel': 0.16,
            'min_angular_vel': -0.7,
            'max_angular_vel': 0.7,
            'update_rate': 100.0, 
            'auto_request_next': True
        }]
    )

    point_pid_path_generator_node = Node(
        package='puzzlebot_control',
        executable='point_PID_path_generator.py',     
        name='point_PID_path_generator',
        parameters=[param_file]                    
    )

    return LaunchDescription([
        odometry_localization_node,
        point_PID_controller_node,
        point_pid_path_generator_node
    ])
