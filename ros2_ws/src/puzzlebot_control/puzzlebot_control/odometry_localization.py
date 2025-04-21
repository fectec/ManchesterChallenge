#!/usr/bin/env python3

import rclpy
import transforms3d
import numpy as np
import math

from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32  
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

# ========================
# Utility Functions
# ========================
def wrap_to_pi(theta):
    # Wrap angle to [-pi, pi]
    result = np.fmod(theta + math.pi, 2.0 * math.pi)
    if result < 0:
        result += 2.0 * math.pi
    return result - math.pi

# ========================
# Odometry Localization Node
# ========================
class OdometryLocalization(Node):
    """
    Estimates the robot's pose (x, y, theta) by dead reckoning, using the angular
    velocities of the left and right wheels of a differential-drive robot. The 
    robot's position and orientation are updated over time using Euler integration
    of the kinematic model.

    The orientation is maintained in the range [-pi, pi].
    """

    def __init__(self):
        super().__init__('odometry_localization')

        # Declare parameters for wheel geometry (m), update rate (Hz), and integration period (s)
        self.declare_parameter('wheel_base', 0.18)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('update_rate', 200.0)
        self.declare_parameter('integration_period', 0.01)

        # Load parameter values
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value 
        self.integration_period = self.get_parameter('integration_period').get_parameter_value().double_value 

        # Robot pose (x, y) and heading (theta in [-pi, pi])
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel angular velocities (rad/s)
        self.omega_r = 0.0
        self.omega_l = 0.0

        # Last timestamp for integration
        self.last_time = None

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
   
        # Subscribe to wheel speeds
        self.create_subscription(
            Float32,
            '/VelocityEncR',
            self.right_wheel_callback,
            qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            '/VelocityEncL',
            self.left_wheel_callback,
            qos.qos_profile_sensor_data
        )

        # Odometry publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            qos.qos_profile_sensor_data
        )   

        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_odometry)
        self.last_log_time = 0.0

        self.get_logger().info("Differential-Drive Dead-Reckoning Node Started.")

    def right_wheel_callback(self, msg):
        """Update right wheel angular velocity (rad/s) from the encoder."""
        self.omega_r = msg.data

    def left_wheel_callback(self, msg):
        """Update left wheel angular velocity (rad/s) from the encoder."""
        self.omega_l = msg.data

    def update_odometry(self):
        """
        Integrate the differential-drive equations using dt
        to update (x, y, theta), publish the odometry message,
        and broadcast the corresponding TF transform.
        """
        # Get current time
        now = self.get_clock().now()
        now_time = now.nanoseconds * 1e-9

        # Initialization on first run
        if self.last_time is None:
            self.last_time = now_time
            return

        # Compute elapsed time since last update (s); skip integration if dt is less than integration_period
        dt = now_time - self.last_time
        if dt < self.integration_period:
            return
        self.last_time = now_time

        # Convert wheel angular velocities (rad/s) to linear speeds (m/s)
        v_r = self.wheel_radius * self.omega_r
        v_l = self.wheel_radius * self.omega_l

        # Compute linear (m/s) and angular (rad/s) velocities of the robot
        V = 0.5 * (v_r + v_l)
        Omega = (v_r - v_l) / self.wheel_base

        # Integrate pose using Euler's method
        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt
        self.theta = wrap_to_pi(self.theta + Omega * dt)

        # Prepare the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        quat = transforms3d.euler.euler2quat(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])

        # Fill in twist
        odom_msg.twist.twist.linear.x = V
        odom_msg.twist.twist.angular.z = Omega

        # Publish Odometry
        self.odom_pub.publish(odom_msg)

        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        t.transform.rotation.w = quat[0]
        self.tf_broadcaster.sendTransform(t)   

        # Log the updated pose
        self.get_logger().info(
            f"Pose -> x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f} rad"
        )
        
def main(args=None):
    rclpy.init(args=args)
    node = OdometryLocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()