# Imports required to set the package address (directories)
import os
from ament_index_python.packages import get_package_share_directory

# Imports required for calling other launch files (nesting)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Imports required to push a namespace
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    package = "motor_control"
    launch_file = "motor_control.launch.py"

    group1_ns = "group_1"
    group2_ns = "group_2"
    group3_ns = "group_3"

    package_directory = get_package_share_directory(package)
    launch_file_path = os.path.join(package_directory, "launch", launch_file)

    launch_source1 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source2 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source3 = PythonLaunchDescriptionSource(launch_file_path)

    launch_1 = IncludeLaunchDescription(launch_source1)
    launch_2 = IncludeLaunchDescription(launch_source2)
    launch_3 = IncludeLaunchDescription(launch_source3)

    motor_control_group1 = GroupAction(
        actions=[PushRosNamespace(group1_ns), launch_1]
    )

    motor_control_group2 = GroupAction(
        actions=[PushRosNamespace(group2_ns), launch_2]
    )

    motor_control_group3 = GroupAction(
        actions=[PushRosNamespace(group3_ns), launch_3]
    )

    return LaunchDescription([
        motor_control_group1,
        motor_control_group2,
        motor_control_group3
    ])