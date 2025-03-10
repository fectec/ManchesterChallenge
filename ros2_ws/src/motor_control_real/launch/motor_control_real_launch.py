from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    micro_ros_agent = Node(name="micro_ros_agent",
                       package='micro_ros_agent',
                       executable='micro_ros_agent',
                       output='screen',
                       arguments=[
                        'serial',
                        "--dev", '/dev/ttyUSB0',
                        ]
                       )
    
    set_point = Node(name="set_point",
                    package='motor_control_real',
                    executable='set_point',
                    output='screen'
                    )
    
    l_d = LaunchDescription([micro_ros_agent, set_point])

    return l_d