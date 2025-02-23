# IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)

import os
from ament_index_python.packages import get_package_share_directory

# IMPORTS REQUIRED FOR CALLING OTHER LAUNCH FILES (NESTING)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# IMPORTS REQUIRED TO PUSH A NAMESPACE [APPEND] A NAMESPACE TO A NESTED LAUNCH FILE

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

# INITIALIZE LAUNCH FILE

def generate_launch_description():

    package = "motor_control"
    launch_file = "motor_launch.py"

    group1_ns = "group_1"
    group2_ns = "group_2"

    package_directory = get_package_share_directory(package)
    launch_file_path = os.path.join(package_directory, "launch", launch_file)

    launch_source1 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source2 = PythonLaunchDescriptionSource(launch_file_path)

    launch_1 = IncludeLaunchDescription(launch_source1)
    launch_2 = IncludeLaunchDescription(launch_source2)

    motor_control_group1 = GroupAction(
        actions=[PushRosNamespace(group1_ns), launch_1]
    )

    motor_control_group2 = GroupAction(
        actions=[PushRosNamespace(group2_ns), launch_2]
    )

    # LAUNCH THE DESCRIPTION

    l_d = LaunchDescription([motor_control_group1, motor_control_group2])

    return l_d