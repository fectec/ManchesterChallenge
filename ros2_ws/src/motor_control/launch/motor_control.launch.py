from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node = Node(
        name='motor_sys',
        package='motor_control',
        executable='dc_motor',
        parameters=[{
            'sample_time': 0.01,
            'gain_K': 2.16,
            'tau_T': 0.05,
            'initial_conditions': 0.0
        }]
    )

    setpoint_node = Node(
        name='sp_gen',
        package='motor_control',
        executable='setpoint',
        parameters=[{
            'timer_period': 0.01,
            'amplitude': 1.0,
            'omega': 1.0,
            'wave_type': 'sine'
        }]
    )

    controller_node = Node(
        name='ctrl',
        package='motor_control',
        executable='controller',
        parameters=[{
            'sample_time': 0.01,
            'Kp': 4.0,
            'Kd': 0.0,
            'Ki': 0.7
        }]
    )

    rqt_node = Node(
        name='rqt_plot',
        package='rqt_plot',
        executable='rqt_plot',
        arguments=['/setpoint/data', '/motor_output_y/data']
    )

    return LaunchDescription([
        motor_node,
        controller_node,
        setpoint_node,
        rqt_node
    ])
