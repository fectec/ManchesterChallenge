# OpenLoopPose.msg

# Standard pose information
geometry_msgs/Pose pose

# Control mode selection
# 0 => use provided target velocities (compute time if needed)
# 1 => use provided execution time (compute velocities if needed)
uint8 control_mode

# Trajectory control fields:
float64 target_linear_velocity    # The commanded linear velocity (m/s); used in velocity-based mode
float64 target_angular_velocity   # The commanded angular velocity (rad/s); used in velocity-based mode
float64 execution_time            # The time in seconds to execute the trajectory segment; used in time-based mode