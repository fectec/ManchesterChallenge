from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    signal_generator_node = Node(package='signal_processing'),
                                executable='signal_generator',
                                output='screen'
                                )

    signal_processor_node = Node(package='signal_processing'),
                                executable='signal_processor',
                                output='screen'
                                )

    l_d = LaunchDescription([signal_generator_node, signal_processor_node])
    return l_d