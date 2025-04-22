#!/usr/bin/env python3

import rclpy
import math
import numpy as np
import time

from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from custom_interfaces.srv import NextWaypoint

# ========================
# Utility Functions
# ========================
def wrap_to_pi(theta):
    # Wrap angle to [-pi, pi]
    result = np.fmod(theta + math.pi, 2.0 * math.pi)
    if result < 0:
        result += 2.0 * math.pi
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
      - /odom (nav_msgs/Odometry): Current robot pose and orientation
    
    Service Clients:
      - /next_waypoint (custom_interfaces/srv/NextWaypoint): Requests the next waypoint
    
    Publishes to:
      - /cmd_vel (geometry_msgs/Twist): Commanded linear and angular velocity

    Control logic:
      - Compute global position errors:
            e_x = x_g - x_r
            e_y = y_g - y_r
      - Compute the signed distance error (projected along the robot's forward direction):
            signed_e_d = e_x*cos(theta_r) + e_y*sin(theta_r)
      - Compute the angular error (wrapped to [-pi, pi]):
            e_theta = wrap_to_pi(atan2(e_y, e_x) - theta_r)
      - The PID controllers compute:
            V     = Kp_V * (signed_e_d) + Ki_V * ∫(signed_e_d) dt + Kd_V * d(signed_e_d)/dt
            Omega = Kp_Omega * e_theta + Ki_Omega * ∫(e_theta) dt + Kd_Omega * d(e_theta)/dt

      The robot stops when both the absolute distance and angular errors are below the tolerances.
    """

    def __init__(self):
        super().__init__('pid_point_controller')

        # Declare PID parameters
        self.declare_parameter('Kp_V', 0.1)
        self.declare_parameter('Ki_V', 0.0)
        self.declare_parameter('Kd_V', 0.0)

        self.declare_parameter('Kp_Omega', 0.1)
        self.declare_parameter('Ki_Omega', 0.0)
        self.declare_parameter('Kd_Omega', 0.0)

        # Declare stop tolerances (absolute thresholds)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('heading_tolerance', 0.05)

        # Declare velocity constraints (m/s & rad/s) for nonlinearity handling
        self.declare_parameter('min_linear_vel', 0.05)  # Minimum linear velocity to overcome friction
        self.declare_parameter('max_linear_vel', 0.16)  # Maximum safe linear velocity
        self.declare_parameter('min_angular_vel', 0.1)  # Minimum angular velocity to overcome inertia
        self.declare_parameter('max_angular_vel', 0.9)  # Maximum safe angular velocity

        # Declare control loop update rate (Hz)
        self.declare_parameter('update_rate', 100.0)
        
        # Declare auto-request parameter
        self.declare_parameter('auto_request_next', True)

        # Load parameters
        self.Kp_V = self.get_parameter('Kp_V').get_parameter_value().double_value
        self.Ki_V = self.get_parameter('Ki_V').get_parameter_value().double_value
        self.Kd_V = self.get_parameter('Kd_V').get_parameter_value().double_value

        self.Kp_Omega = self.get_parameter('Kp_Omega').get_parameter_value().double_value
        self.Ki_Omega = self.get_parameter('Ki_Omega').get_parameter_value().double_value
        self.Kd_Omega = self.get_parameter('Kd_Omega').get_parameter_value().double_value

        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.heading_tolerance = self.get_parameter('heading_tolerance').get_parameter_value().double_value
        
        self.min_linear_vel = self.get_parameter('min_linear_vel').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.min_angular_vel = self.get_parameter('min_angular_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.auto_request_next = self.get_parameter('auto_request_next').get_parameter_value().bool_value

        # Robot state
        self.current_pose = Pose2D()
        self.setpoint = Pose2D()
        self.goal_active = False
        self.current_waypoint_id = -1
        self.path_completed = False
        
        # PID internals 
        self.last_time = None
        self.integral_e_d = 0.0
        self.integral_e_theta = 0.0
        self.last_signed_e_d = 0.0
        self.last_e_theta = 0.0

        # Publisher for commanded velocities 
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        
        # Service client for requesting the next waypoint
        self.next_waypoint_client = self.create_client(NextWaypoint, '/next_waypoint')
        while not self.next_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the next_waypoint service...')
        
        # Timer for the control loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)
        self.last_log_time = 0.0

        self.get_logger().info("PID Position Controller Node Started.")
        
        # Request the first waypoint
        self.request_next_waypoint(True)
    
    def apply_velocity_constraints(self, velocity, min_vel, max_vel):
        """
        Apply minimum and maximum velocity constraints to handle nonlinearity.
        
        If the absolute value of velocity is below min_vel but not zero, set it to min_vel
        while preserving the sign. Limit the velocity to max_vel.
        """
        if abs(velocity) < 0.001:  # Effectively zero
            return 0.0
            
        # Apply minimum threshold (to overcome friction/inertia)
        if abs(velocity) < min_vel:
            velocity = min_vel * (1 if velocity > 0 else -1)
            
        # Apply maximum limit (saturation)
        if abs(velocity) > max_vel:
            velocity = max_vel * (1 if velocity > 0 else -1)
            
        return velocity
    
    def request_next_waypoint(self, previous_reached=True):
        # Request the next waypoint from the service provider
        request = NextWaypoint.Request()
        request.previous_reached = previous_reached
        
        # Send the request
        future = self.next_waypoint_client.call_async(request)
        future.add_done_callback(self.process_waypoint_response)
    
    def process_waypoint_response(self, future):
        # Process the response from the waypoint service
        try:
            response = future.result()
            
            if response.completed:
                self.get_logger().info("Path completed! No more waypoints available.")
                self.path_completed = True
                self.goal_active = False
                
                # Stop the robot
                cmd = Twist()
                self.cmd_pub.publish(cmd)
                return
            
            # Process the new waypoint
            self.current_waypoint_id = response.waypoint_id
            self.setpoint.x = response.goal.pose.position.x
            self.setpoint.y = response.goal.pose.position.y
            self.setpoint.theta = response.goal.theta
            
            # Reset PID state for the new goal
            self.last_time = None
            self.integral_e_d = 0.0
            self.integral_e_theta = 0.0
            self.last_signed_e_d = 0.0
            self.last_e_theta = 0.0
            self.goal_active = True
            
            self.get_logger().info(f'New waypoint {self.current_waypoint_id+1}: ' +
                                   f'x={self.setpoint.x:.2f}, y={self.setpoint.y:.2f}.')
        
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint response: {e}.")

    def odom_callback(self, msg):
        # Update robot pose from odometry
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        _, _, yaw = quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self.current_pose.theta = yaw
    
    def control_loop(self):
        # Run PID control only when a goal is active
        if not self.goal_active or self.path_completed:
            return
        
        now_time = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now_time
            return
        
        dt = now_time - self.last_time
        if dt < 1.0 / self.update_rate:
            return

        # Compute position error components
        e_x = self.setpoint.x - self.current_pose.x
        e_y = self.setpoint.y - self.current_pose.y

        # Signed distance error along the robot's heading
        signed_e_d = e_x * math.cos(self.current_pose.theta) + e_y * math.sin(self.current_pose.theta)
    
        # Absolute distance error
        abs_e_d = math.hypot(e_x, e_y)

        # Compute angular error (wrapped to [-pi, pi])
        e_theta = wrap_to_pi(math.atan2(e_y, e_x) - self.current_pose.theta)

        # Auto-stop if both errors are below thresholds
        if abs_e_d < self.goal_tolerance and abs(e_theta) < self.heading_tolerance:
            # Stop the robot temporarily
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'Waypoint {self.current_waypoint_id+1} reached at ' + 
                                   f'x={self.current_pose.x:.3f}, y={self.current_pose.y:.3f}.')
            
            # Mark the current goal as completed
            self.goal_active = False
            
            # Request the next waypoint if auto-request is enabled
            if self.auto_request_next:
                self.request_next_waypoint(True)
            
            return

        # PID control for linear velocity
        self.integral_e_d += signed_e_d * dt
        derivative_e_d = (signed_e_d - self.last_signed_e_d) / dt
        V = self.Kp_V * signed_e_d + self.Ki_V * self.integral_e_d + self.Kd_V * derivative_e_d

        # PID control for angular velocity
        self.integral_e_theta += e_theta * dt
        derivative_e_theta = (e_theta - self.last_e_theta) / dt
        Omega = self.Kp_Omega * e_theta + self.Ki_Omega * self.integral_e_theta + self.Kd_Omega * derivative_e_theta

        # Save errors and time for the next iteration
        self.last_signed_e_d = signed_e_d
        self.last_e_theta = e_theta
        self.last_time = now_time

        # Apply nonlinearity handling
        V = self.apply_velocity_constraints(V, self.min_linear_vel, self.max_linear_vel)
        Omega = self.apply_velocity_constraints(Omega, self.min_angular_vel, self.max_angular_vel)

        # Publish the computed command
        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = Omega
        self.cmd_pub.publish(cmd)

        # Debug info - limit logging frequency
        current_time = time.time()
        if current_time - self.last_log_time > 1.0:  
            self.get_logger().info(
                f"Control: dist_err={abs_e_d:.3f}, ang_err={e_theta:.3f}, V={V:.3f}, Omega={Omega:.3f}."
            )
            self.last_log_time = current_time

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