# OpenLoopPose.msg

# Standard pose information
geometry_msgs/Pose pose

# Mode constants (simulate an enum)
uint8 CONTROL_MODE_VELOCITY=0
uint8 CONTROL_MODE_TIME=1

# Control mode selection.
# 0 => use provided target velocities (compute time if needed)
# 1 => use provided execution time (compute velocities if needed)
uint8 control_mode

# Trajectory control fields:
float64 target_linear_velocity    # The commanded linear velocity (m/s); used in velocity-based mode.
float64 target_angular_velocity   # The commanded angular velocity (rad/s); used in velocity-based mode.
float64 execution_time            # The time in seconds to execute the trajectory segment; used in time-based mode.