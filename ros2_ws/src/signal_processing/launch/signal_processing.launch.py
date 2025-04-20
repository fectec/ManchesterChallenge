from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    signal_generator_node = Node(package='signal_processing',
                                executable='signal_generator',
                                output='screen'
                                )

    signal_processor_node = Node(package='signal_processing',
                                executable='signal_processor',
                                output='screen'
                                )
                                
    rqt_node = Node(name='rqt_plot',
	           package='rqt_plot',
	           executable='rqt_plot',
	           arguments=['/signal/data', '/proc_signal/data']
	           )
    
    return LaunchDescription([signal_generator_node, signal_processor_node, rqt_node])
