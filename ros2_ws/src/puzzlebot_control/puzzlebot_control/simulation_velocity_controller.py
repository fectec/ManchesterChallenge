#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformBroadcaster

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from tf_transformations import quaternion_from_euler

import numpy as np
import math

class VelocityController(Node):
    """
    Converts the robot's linear and angular velocity commands into wheel speeds.
    Subscribes to:
      - puzzlebot_control/cmd_vel (TwistStamped)
      - joint_states (JointState)
    Publishes to:
      - simple_velocity_controller/commands (Float64MultiArray)
      - puzzlebot_control/odom (Odometry)
    """
    def __init__(self):
        super().__init__('velocity_controller')

        # Declare and load parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.18)
        self.wheel_radius     = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value

        # Previous wheel encoder readings and timestamp
        self.left_wheel_prev_pos  = 0.0
        self.right_wheel_prev_pos = 0.0
        self.prev_time            = self.get_clock().now()

        # Robot pose
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Odometry message template
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id       = 'odom'
        self.odom_msg.child_frame_id        = 'base_footprint'

        # 
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint"

        # Publishers
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'simple_velocity_controller/commands',
            10
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            'puzzlebot_control/odom',
            10
        )

        # Subscribers
        self.vel_sub = self.create_subscription(
            TwistStamped,
            'puzzlebot_control/cmd_vel',
            self.velocity_callback,
            10
        )
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )

        # Conversion matrix from [V; Omega] to [ω_r; ω_l]
        self.speed_conversion = np.array([
            [ self.wheel_radius/2,  self.wheel_radius/2],
            [ self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]
        ])

    def joint_callback(self, msg):
        # Compute delta positions from encoder readings
        dp_left  = msg.position[1] - self.left_wheel_prev_pos
        dp_right = msg.position[0] - self.right_wheel_prev_pos

        # Compute elapsed time since last update
        stamp   = Time.from_msg(msg.header.stamp)
        dt      = stamp - self.prev_time
        dt_sec  = dt.nanoseconds * 1e-9

        # Update history for next callback
        self.left_wheel_prev_pos  = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
        self.prev_time            = stamp

        # Guard against zero dt
        if dt_sec > 0.0:
            fi_left  = dp_left  / dt_sec
            fi_right = dp_right / dt_sec
        else:
            fi_left = fi_right = 0.0

        # Compute robot velocities for logging
        linear  = ( self.wheel_radius*fi_right + self.wheel_radius*fi_left ) / 2
        angular = ( self.wheel_radius*fi_right - self.wheel_radius*fi_left ) / self.wheel_separation

        # Dead-reckoning integration
        d_s     = ( self.wheel_radius*dp_right + self.wheel_radius*dp_left ) / 2
        d_theta = ( self.wheel_radius*dp_right - self.wheel_radius*dp_left ) / self.wheel_separation

        self.theta += d_theta
        self.x     += d_s * math.cos(self.theta)
        self.y     += d_s * math.sin(self.theta)

        # Update odometry message
        q = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x    = self.x
        self.odom_msg.pose.pose.position.y    = self.y
        self.odom_msg.pose.pose.position.z    = 0.0
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.twist.twist.linear.x   = linear
        self.odom_msg.twist.twist.angular.z  = angular
        
        self.transform_stamped.transform.translation.x = self.x
        self.transform_stamped.transform.translation.y = self.y
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

        self.odom_pub.publish(self.odom_msg)
        self.tf_broadcaster.sendTransform(self.transform_stamped)
        
    def velocity_callback(self, msg):
        """
        Convert incoming TwistStamped (V, Omega) into individual wheel speeds.
        """
        robot_speed = np.array([
            [ msg.twist.linear.x  ],
            [ msg.twist.angular.z ]
        ])
        wheel_speed = np.linalg.inv(self.speed_conversion) @ robot_speed

        m = Float64MultiArray()
        m.data = [ wheel_speed[1,0], wheel_speed[0,0] ]
        self.wheel_cmd_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()