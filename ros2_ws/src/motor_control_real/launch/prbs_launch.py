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
    
    set_point = Node(name="prbs",
                    package='motor_control_real',
                    executable='prbs',
                    output='screen',
                    parameters=[
                        {"timer_period": 0.1},
                        {"prbs_interval": 1.0},
                        {"base_duty_cycle": 40.0},
                        {"amplitude_increment": 0.1},
                        {"amplitude_increment_period": 5.0}
                    ]
                    )
    
    l_d = LaunchDescription([micro_ros_agent, set_point])

    return l_d