#!/usr/bin/env python3

import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from puzzlebot_utils.utils.math_helpers import wrap_to_pi, yaw_to_quaternion
from tf2_ros import TransformBroadcaster

from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdometryLocalization(Node):
    """
    Estimates the robot's pose (x, y, theta) by dead reckoning, using the angular
    velocities of the left and right wheels of a differential-drive robot. The 
    robot's position and orientation are updated over time using Euler integration
    of the kinematic model.
    """

    def __init__(self):
        super().__init__('odometry_localization')

        # Declare parameters
        self.declare_parameter('update_rate',      200.0)   # Hz
        self.declare_parameter('integration_rate', 100.0)   # Hz
        self.declare_parameter('wheel_base',       0.173)   # m
        self.declare_parameter('wheel_radius',     0.0505)  # m

        # Load parameters
        self.update_rate      = self.get_parameter('update_rate').value
        self.integration_rate = self.get_parameter('integration_rate').value
        self.wheel_base       = self.get_parameter('wheel_base').value
        self.wheel_radius     = self.get_parameter('wheel_radius').value

        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_odometry)

        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate',      Parameter.Type.DOUBLE, self.update_rate),
            Parameter('integration_rate', Parameter.Type.DOUBLE, self.integration_rate),
            Parameter('wheel_base',       Parameter.Type.DOUBLE, self.wheel_base),
            Parameter('wheel_radius',     Parameter.Type.DOUBLE, self.wheel_radius),
        ]
        result = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}.")

        # Robot pose (x, y) (m) and heading (theta in [-pi, pi] (rad))
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel angular velocities (rad/s)
        self.omega_r = 0.0
        self.omega_l = 0.0

        # Last timestamp for integration (s)
        self.last_time = None
        
        # Limit logging frequency (s)
        self.last_log_time = 0.0

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
   
        # Wheel speeds subscribers
        self.create_subscription(
            Float32,
            'VelocityEncR',
            self.right_wheel_callback,
            qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            'VelocityEncL',
            self.left_wheel_callback,
            qos.qos_profile_sensor_data
        )

        # Odometry publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            'puzzlebot_real/odom',
            qos.qos_profile_sensor_data
        )   

        self.get_logger().info("OdometryLocalization Start.")

    def right_wheel_callback(self, msg):
        # Update right wheel angular velocity (rad/s) from the encoder
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

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="update_rate must be > 0."
                    )
                self.update_rate = float(param.value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.update_rate, self.update_odometry)
                self.get_logger().info(f"update_rate set to {self.update_rate} Hz.")

            elif param.name == 'integration_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="integration_rate must be > 0."
                    )
                self.integration_rate = float(param.value)
                self.get_logger().info(f"integration_rate set to {self.integration_rate} Hz.")

            elif param.name == 'wheel_base':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="wheel_base must be > 0."
                    )
                self.wheel_base = float(param.value)
                self.get_logger().info(f"wheel_base set to {self.wheel_base} m.")

            elif param.name == 'wheel_radius':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="wheel_radius must be > 0."
                    )
                self.wheel_radius = float(param.value)
                self.get_logger().info(f"wheel_radius set to {self.wheel_radius} m.")

        return SetParametersResult(successful=True)
    
def main(args=None):
    rclpy.init(args=args)

    try:
        node = OdometryLocalization()
    except Exception as e:
        print(f"[FATAL] OdometryLocalization failed to initialize: {e}.", file=sys.stderr)
        rclpy.shutdown()
        return

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