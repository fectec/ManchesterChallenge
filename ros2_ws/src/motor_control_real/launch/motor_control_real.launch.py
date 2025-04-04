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
    
    setpoint = Node(name="setpoint",
                    package='motor_control_real',
                    executable='setpoint',
                    output='screen',
                    parameters=[
                        {"timer_period": 0.1},
                        {"amplitude": 20.0},
                        {"frequency": 0.05},
                        {"hold_single": True},
                        {"fixed_wave": "sine"}
                        ]
                    )
    
    l_d = LaunchDescription([micro_ros_agent, setpoint])

    return l_d