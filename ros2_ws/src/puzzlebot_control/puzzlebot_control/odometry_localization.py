#!/usr/bin/env python3

import rclpy
import math
import time

from rclpy.node import Node
from rclpy import qos

from tf2_ros import TransformBroadcaster
from puzzlebot_utils.utils.math_helpers import wrap_to_pi, yaw_to_quaternion

from std_msgs.msg import Float32 
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdometryLocalization(Node):
    """
    Estimates the robot's pose (x, y, theta) by dead reckoning, using the angular
    velocities of the left and right wheels of a differential-drive robot. The 
    robot's position and orientation are updated over time using Euler integration
    of the kinematic model.
    """

    def __init__(self):
        super().__init__('odometry_localization')

        # Declare parameter
        self.declare_parameter('update_rate', 200.0)
        self.declare_parameter('integration_rate', 100.0)
        self.declare_parameter('wheel_base', 0.173)
        self.declare_parameter('wheel_radius', 0.0505)

        # Load parameter values
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value 
        self.integration_rate = self.get_parameter('integration_rate').get_parameter_value().double_value 
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Robot pose (x, y) and heading (theta in [-pi, pi])
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel angular velocities (rad/s)
        self.omega_r = 0.0
        self.omega_l = 0.0

        # Last timestamp for integration
        self.last_time = None
        
        # Limit logging frequency
        self.last_log_time = 0.0

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
            '/puzzlebot_real/odom',
            qos.qos_profile_sensor_data
        )   

        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_odometry)
        
        self.get_logger().info("OdometryLocalization Start.")

    def right_wheel_callback(self, msg):
        # Update right wheel angular velocity (rad/s) from the encoder#
        self.omega_r = msg.data

    def left_wheel_callback(self, msg):
        # Update left wheel angular velocity (rad/s) from the encoder
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

        # Compute elapsed time since last update (s)
        # Skip integration if dt is less than integration period
        dt = now_time - self.last_time
        if dt < 1.0 / self.integration_rate:
            return
        self.last_time = now_time

        # Convert wheel angular velocities (rad/s) to linear velocities (m/s)
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

        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.theta)

        odom_msg.twist.twist.linear.x = V
        odom_msg.twist.twist.angular.z = Omega

        self.odom_pub.publish(odom_msg)

        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation = yaw_to_quaternion(self.theta)

        self.tf_broadcaster.sendTransform(t)   

        # Log the updated pose
        current_time = time.time()
        if current_time - self.last_log_time > 1.0:  # Log once per second at most
            self.get_logger().info(
                f"Pose -> x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f} rad"
            )
            self.last_log_time = current_time
        
def main(args=None):
    rclpy.init(args=args)
    node = OdometryLocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted with Ctrl+C.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()