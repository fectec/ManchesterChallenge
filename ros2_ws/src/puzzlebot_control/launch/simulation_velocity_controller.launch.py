from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0505'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.173'
    )

    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    
    joint_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    simple_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'simple_velocity_controller',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    simulation_velocity_controller_node = Node(
        package='puzzlebot_control',
        executable='simulation_velocity_controller.py',
        parameters=[{'wheel_radius': wheel_radius,
                     'wheel_separation': wheel_separation}]
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner_node,
        simple_controller_node,
        simulation_velocity_controller_node
    ])