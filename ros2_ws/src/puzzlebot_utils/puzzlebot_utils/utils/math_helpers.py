import numpy as np
import math

from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler

def wrap_to_pi(theta):
    # Wrap angle to [-pi, pi]
    result = np.fmod(theta + math.pi, 2.0 * math.pi)
    if result < 0:
        result += 2.0 * math.pi
    return result - math.pi

def normalize_angle_diff(current_angle, goal_angle):
    # Normalize the angle difference to the range [-pi, pi]
    diff = goal_angle - current_angle
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    return diff

def angle_diff_signed(sign, diff):
    # Return signed angle difference based on the sign
    return diff if sign > 0 else -diff

def yaw_to_quaternion(yaw):
    # Convert a yaw (around Z) in radians into a ROS Quaternion message
    x, y, z, w = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)