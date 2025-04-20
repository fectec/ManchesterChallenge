#!/usr/bin/env python3

import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

from std_srvs.srv import SetBool

# ========================
# Utility Functions
# ========================
def wrap_to_pi(theta):
    # Wrap angle to [-pi, pi]
    result = np.fmod((theta + math.pi), (2 * math.pi))
    if result < 0:
        result += 2 * math.pi
    return result - math.pi

def quaternion_to_euler(x, y, z, w):
    # Convert quaternion (x,y,z,w) to Euler angles (roll, pitch, yaw)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

# ========================
# PID Point Controller Node
# ========================
class PIDPointController(Node):
    """
    Computes velocity commands (V, Omega) using PID control to drive a differential‑drive 
    robot toward a desired goal pose (x_g, y_g, theta_g) based on current odometry (x_r, y_r, theta_r).

    Subscribes to:
      - /setpoint (std_msgs/Float32MultiArray): Desired goal, as [x_g, y_g, theta_g]
      - /odom (nav_msgs/Odometry): Current robot pose and orientation
    Publishes to:
      - /cmd_vel (geometry_msgs/Twist): Commanded linear and angular velocity

    Control logic:
      - Compute global position errors:
            e_x = x_g - x_r
            e_y = y_g - y_r
      - Compute the signed distance error (projected along the robot’s forward direction):
            signed_e_d = e_x*cos(theta_r) + e_y*sin(theta_r)
      - Compute the angular error (wrapped to [-pi, pi]):
            e_theta = wrap_to_pi(atan2(e_y, e_x) - theta_r)
      - The PID controllers compute:
            V     = Kp_V * (signed_e_d) + Ki_V * ∫(signed_e_d) dt + Kd_V * d(signed_e_d)/dt
            Omega = Kp_Omega * e_theta + Ki_Omega * ∫(e_theta) dt + Kd_Omega * d(e_theta)/dt

      The robot stops when both the absolute distance and angular errors are below the tolerances.
    """

    def __init__(self):
        super().__init__('point_controller')

        # Declare PID parameters
        self.declare_parameter('Kp_V', 1.0)
        self.declare_parameter('Ki_V', 0.0)
        self.declare_parameter('Kd_V', 0.0)

        self.declare_parameter('Kp_Omega', 1.0)
        self.declare_parameter('Ki_Omega', 0.0)
        self.declare_parameter('Kd_Omega', 0.0)

        # Declare stop tolerances (absolute thresholds)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('angle_tolerance', 0.01)

        # Declare control loop update rate (Hz)
        self.declare_parameter('update_rate', 200.0)

        # Load parameters
        self.Kp_V = self.get_parameter('Kp_V').get_parameter_value().double_value
        self.Ki_V = self.get_parameter('Ki_V').get_parameter_value().double_value
        self.Kd_V = self.get_parameter('Kd_V').get_parameter_value().double_value

        self.Kp_Omega = self.get_parameter('Kp_Omega').get_parameter_value().double_value
        self.Ki_Omega = self.get_parameter('Ki_Omega').get_parameter_value().double_value
        self.Kd_Omega = self.get_parameter('Kd_Omega').get_parameter_value().double_value

        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        # Robot state
        self.current_pose = Pose2D()
        self.setpoint = Pose2D()

        # PID internals (using signed error for linear control)
        self.last_time = None
        self.last_signed_e_d = 0.0
        self.last_e_theta = 0.0
        self.integral_signed_e_d = 0.0
        self.integral_e_theta = 0.0
        self.stopped = False

        # Publisher for commanded velocities (/cmd_vel)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )

        # Subscribers for odometry and goal setpoint
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        self.goal_sub = self.create_subscription(
            Float32MultiArray,
            '/setpoint',
            self.setpoint_callback,
            qos.qos_profile_sensor_data
        )

        # Timer for the control loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        self.get_logger().info("PID Position Controller Node Started.")

    def setpoint_callback(self, msg):
        # Update the target goal pose from the setpoint message
        self.setpoint.x = msg.data[0]
        self.setpoint.y = msg.data[1]

    def odom_callback(self, msg):
        # Update the current robot pose from odometry
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_pose.theta = yaw

    def control_loop(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        if dt <= 0 or dt < 1.0 / self.update_rate:
            return

        # Compute position error components
        e_x = self.setpoint.x - self.current_pose.x
        e_y = self.setpoint.y - self.current_pose.y

        # Signed distance error along the robot's heading
        signed_e_d = e_x * math.cos(self.current_pose.theta) + e_y * math.sin(self.current_pose.theta)
    
        # Absolute distance error
        abs_e_d = math.sqrt(e_x**2 + e_y**2)

        # Compute angular error (wrapped to [-pi, pi])
        e_theta = wrap_to_pi(math.atan2(e_y, e_x) - self.current_pose.theta)

        # Auto-stop if both errors are below thresholds
        if abs_e_d < self.position_tolerance and abs(e_theta) < self.angle_tolerance:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            if not self.stopped:
                self.get_logger().info("Target reached. Stopping the robot.")
                self.stopped = True
            self.last_time = current_time
            return
        else:
            self.stopped = False

        # PID control for linear velocity
        self.integral_signed_e_d += signed_e_d * dt
        d_signed_e_d = (signed_e_d - self.last_signed_e_d) / dt
        V = self.Kp_V * signed_e_d + self.Ki_V * self.integral_signed_e_d + self.Kd_V * d_signed_e_d

        # PID control for angular velocity
        self.integral_e_theta += e_theta * dt
        d_e_theta = (e_theta - self.last_e_theta) / dt
        Omega = self.Kp_Omega * e_theta + self.Ki_Omega * self.integral_e_theta + self.Kd_Omega * d_e_theta

        # Save errors and time for the next iteration
        self.last_signed_e_d = signed_e_d
        self.last_e_theta = e_theta
        self.last_time = current_time

        # Publish the computed command
        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = Omega
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDPointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()