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
            'update_rate': 100.0,
            'integration_period': 0.02
        }]
    )
        
    point_PID_controller_node = Node(
        package='puzzlebot_control',
        executable='point_PID_controller.py',
        name='point_PID_controller',
        parameters=[{
            'Kp_V': 0.2,
            'Ki_V': 0.0,
            'Kd_V': 0.0,
            'Kp_Omega': 0.2,
            'Ki_Omega': 0.0,
            'Kd_Omega': 0.0,
            'position_tolerance': 0.05,
            'angle_tolerance': 0.05,
            'min_linear_vel': 0.05, 
            'max_linear_vel': 0.16,
            'min_angular_vel': 0.1,
            'max_angular_vel': 0.9,
            'update_rate': 100.0
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
