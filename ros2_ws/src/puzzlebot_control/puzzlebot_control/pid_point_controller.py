#!/usr/bin/env python3

import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from tf_transformations import euler_from_quaternion
from puzzlebot_utils.utils.math_helpers import wrap_to_pi

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

from std_srvs.srv import SetBool
from custom_interfaces.srv import NextPIDWaypoint

class PIDPointController(Node):
    """
    Computes velocity commands (V, Omega) using PID control to drive a differential‑drive 
    robot toward a desired goal pose (x_g, y_g, theta_g) based on current odometry (x_r, y_r, theta_r).

    Subscribes to:
      - /puzzlebot_real/odom (nav_msgs/Odometry): Current robot pose and orientation
    
    Service Clients:
      - /puzzlebot_real/point_PID/next_PID_waypoint (custom_interfaces/srv/NextPIDWaypoint): Requests the next waypoint
    
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

    This node also includes a service (/puzzlebot_real/point_PID/PID_stop) that, when called, forces the controller to publish a zero Twist.
    In addition, if a new waypoint is received (via /puzzlebot_real/point_PID/waypoint), the controller automatically resumes.
    """

    def __init__(self):
        super().__init__('pid_point_controller')
        
        # Declare parameters
        self.declare_parameter('update_rate',         100.0)        # Hz
        self.declare_parameter('Kp_V',                0.1)      
        self.declare_parameter('Ki_V',                0.0)
        self.declare_parameter('Kd_V',                0.0)
        self.declare_parameter('Kp_Omega',            0.1)
        self.declare_parameter('Ki_Omega',            0.0)
        self.declare_parameter('Kd_Omega',            0.0)
        self.declare_parameter('goal_tolerance',      0.08)         # m
        self.declare_parameter('heading_tolerance',   0.08)         # m
        self.declare_parameter('min_linear_speed',    0.1)          # m/s
        self.declare_parameter('max_linear_speed',    0.17)         # m/s
        self.declare_parameter('min_angular_speed',  -1.0)          # rad/s
        self.declare_parameter('max_angular_speed',    1.0)         # rad/s
        self.declare_parameter('auto_request_next',   True)
        self.declare_parameter('velocity_scale_factor', 1.0)
    
        # Retrieve parameters
        self.update_rate                = self.get_parameter('update_rate').value
        self.Kp_V                       = self.get_parameter('Kp_V').value
        self.Ki_V                       = self.get_parameter('Ki_V').value
        self.Kd_V                       = self.get_parameter('Kd_V').value
        self.Kp_Omega                   = self.get_parameter('Kp_Omega').value
        self.Ki_Omega                   = self.get_parameter('Ki_Omega').value
        self.Kd_Omega                   = self.get_parameter('Kd_Omega').value
        self.goal_tolerance             = self.get_parameter('goal_tolerance').value
        self.heading_tolerance          = self.get_parameter('heading_tolerance').value
        self.min_linear_speed           = self.get_parameter('min_linear_speed').value
        self.max_linear_speed           = self.get_parameter('max_linear_speed').value
        self.min_angular_speed          = self.get_parameter('min_angular_speed').value
        self.max_angular_speed          = self.get_parameter('max_angular_speed').value
        self.auto_request_next          = self.get_parameter('auto_request_next').value
        self.velocity_scale_factor      = self.get_parameter('velocity_scale_factor').value

        # Timer for the control loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate',                Parameter.Type.DOUBLE, self.update_rate),
            Parameter('Kp_V',                       Parameter.Type.DOUBLE, self.Kp_V),
            Parameter('Ki_V',                       Parameter.Type.DOUBLE, self.Ki_V),
            Parameter('Kd_V',                       Parameter.Type.DOUBLE, self.Kd_V),
            Parameter('Kp_Omega',                   Parameter.Type.DOUBLE, self.Kp_Omega),
            Parameter('Ki_Omega',                   Parameter.Type.DOUBLE, self.Ki_Omega),
            Parameter('Kd_Omega',                   Parameter.Type.DOUBLE, self.Kd_Omega),
            Parameter('goal_tolerance',             Parameter.Type.DOUBLE, self.goal_tolerance),
            Parameter('heading_tolerance',          Parameter.Type.DOUBLE, self.heading_tolerance),
            Parameter('min_linear_speed',           Parameter.Type.DOUBLE, self.min_linear_speed),
            Parameter('max_linear_speed',           Parameter.Type.DOUBLE, self.max_linear_speed),
            Parameter('min_angular_speed',          Parameter.Type.DOUBLE, self.min_angular_speed),
            Parameter('max_angular_speed',          Parameter.Type.DOUBLE, self.max_angular_speed),
            Parameter('auto_request_next',          Parameter.Type.BOOL,   self.auto_request_next),
            Parameter('velocity_scale_factor',      Parameter.Type.DOUBLE, self.velocity_scale_factor),

        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")

        # Robot state
        self.current_pose = Pose2D()
        self.waypoint = Pose2D()
        self.goal_active = False
        self.current_waypoint_id = -1
        self.path_completed = False

        # Time tracking (s)
        self.now_time = None
        self.last_time = None
        
        # PID internals 
        self.integral_e_d = 0.0
        self.integral_e_theta = 0.0
        self.last_signed_e_d = 0.0
        self.last_e_theta = 0.0

        # Limit logging frequency (s)
        self.last_log_time = 0.0

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )
        
        self.waypoint_pub   = self.create_publisher(Pose2D,  'puzzlebot_real/point_PID/waypoint', 10)
        self.signed_e_d_pub   = self.create_publisher(Float32, 'puzzlebot_real/point_PID/signed_e_d', 10)
        self.abs_e_d_pub = self.create_publisher(Float32, 'puzzlebot_real/point_PID/abs_e_d', 10)
        self.e_theta_pub    = self.create_publisher(Float32, 'puzzlebot_real/point_PID/e_theta', 10)

        # Subscriber for odometry
        self.create_subscription(
            Odometry,
            'puzzlebot_real/odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        
        # Service client for requesting the next waypoint
        self.next_waypoint_client = self.create_client(
            NextPIDWaypoint, 'puzzlebot_real/point_PID/next_PID_waypoint')
        while not self.next_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the next_waypoint service...')

        # Service server to allow external stopping/resuming of PID output
        self.create_service(SetBool, 'puzzlebot_real/point_PID/PID_stop', self.pid_stop_callback)
        self.pid_stop = False  

        # Request the first waypoint
        self.request_next_waypoint(True)

        self.get_logger().info("PIDPointController Start.")
    
    def odom_callback(self, msg):
        # Update x, y
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        # Convert the incoming orientation quaternion to Euler angles,
        # then store the yaw component
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose.theta = yaw
    
    def request_next_waypoint(self, previous_reached=True):
        # Request the next waypoint from the service provider
        request = NextPIDWaypoint.Request()
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
                self.cmd_pub.publish(Twist())
                return
            
            # Process and publish the new waypoint
            self.current_waypoint_id = response.waypoint_id
            self.waypoint.x = response.goal.pose.position.x
            self.waypoint.y = response.goal.pose.position.y
            self.waypoint.theta = response.goal.theta
            self.waypoint_pub.publish(self.waypoint)
            
            # Reset PID state for the new goal
            self.last_time = None
            self.integral_e_d = 0.0
            self.integral_e_theta = 0.0
            self.last_signed_e_d = 0.0
            self.last_e_theta = 0.0
            self.goal_active = True
            
            self.get_logger().info(f'New waypoint {self.current_waypoint_id+1}: ' +
                                   f'x={self.waypoint.x:.2f}, y={self.waypoint.y:.2f}.')
        
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint response: {e}.")

    def pid_stop_callback(self, request, response):
        self.pid_stop = request.data
        response.success = True
        if self.pid_stop:
            response.message = "PID controller stopped."
            self.get_logger().info("Received PID stop command.")
        else:
            response.message = "PID controller resumed."
            self.get_logger().info("Received PID resume command.")
        return response
    
    def control_loop(self):
        # Run PID control only when a goal is active
        if not self.goal_active or self.path_completed:
            return
        
        # Record the current timestamp on the very first control cycle 
        # so that we have a baseline for computing dt
        now_time = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now_time
            return
        
        # Calculate the elapsed time 
        # since the last update and return early 
        # if it’s less than the interval dictated by update_rate,
        # to enforce a consistent loop frequency
        dt = now_time - self.last_time
        if dt < 1.0 / self.update_rate:
            return
        self.last_time = now_time
        
        # If the PID stop flag is set, publish a zero Twist and return immediately
        if self.pid_stop:
            self.cmd_pub.publish(Twist())
            self.get_logger().debug("PID stop flag active; publishing zero command.")
            self.last_time = now_time
            return

        # Compute position error components
        e_x = self.waypoint.x - self.current_pose.x
        e_y = self.waypoint.y - self.current_pose.y

        # Signed distance error along the robot's heading
        signed_e_d = e_x * math.cos(self.current_pose.theta) + e_y * math.sin(self.current_pose.theta)
    
        # Absolute distance error
        abs_e_d = math.hypot(e_x, e_y)

        # Compute angular error (wrapped to [-pi, pi])
        e_theta = wrap_to_pi(math.atan2(e_y, e_x) - self.current_pose.theta)

        # Publish the intermediate error signals
        self.signed_e_d_pub.publish(Float32(data=signed_e_d))
        self.abs_e_d_pub.publish(Float32(data=abs_e_d))
        self.e_theta_pub.publish(Float32(data=e_theta))

        # Auto-stop if both errors are below thresholds
        if abs_e_d < self.goal_tolerance and abs(e_theta) < self.heading_tolerance:
            self.cmd_pub.publish(Twist())

            GREEN = "\033[32m"
            RESET = "\033[0m"

            msg = (
                "----------------------\n"
                f"Waypoint {self.current_waypoint_id+1} reached at "
                f"x={self.current_pose.x:.3f}, y={self.current_pose.y:.3f}.\n"
                "----------------------"
            )
            self.get_logger().info(f"{GREEN}{msg}{RESET}")

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
        V = self.apply_velocity_constraints(V, self.min_linear_speed, self.max_linear_speed)
        Omega = self.apply_velocity_constraints(Omega, self.min_angular_speed, self.max_angular_speed)

        # Scale the final velocity output
        V = V * self.velocity_scale_factor    

        # Publish the computed command
        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = Omega
        self.cmd_pub.publish(cmd)

        # Debug info
        current_time = time.time()
        if current_time - self.last_log_time > 1.0:  # Log once per second at most
            self.get_logger().info(
                f"Control: dist_err={abs_e_d:.3f}, ang_err={e_theta:.3f}, V={V:.3f}, Omega={Omega:.3f}."
            )
            self.last_log_time = current_time

    def apply_velocity_constraints(self, velocity, min_vel, max_vel):
        if abs(velocity) < 0.01:    # Effectively zero
            return 0.0
            
        # Apply minimum threshold (to overcome friction/inertia)
        if abs(velocity) < abs(min_vel):
            velocity = abs(min_vel) * (1 if velocity > 0 else -1)
            
        # Apply maximum limit (saturation)
        if abs(velocity) > abs(max_vel):
            velocity = abs(max_vel) * (1 if velocity > 0 else -1)
            
        return velocity

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
                self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)
                self.get_logger().info(f"Update rate changed: {self.update_rate} Hz.")

            elif param.name in (
                'Kp_V', 'Ki_V', 'Kd_V',
                'Kp_Omega', 'Ki_Omega', 'Kd_Omega'
            ):
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a non-negative number."
                    )
                setattr(self, param.name, float(param.value))
                self.get_logger().info(f"{param.name} updated: {param.value}.")

            elif param.name == 'goal_tolerance':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="goal_tolerance must be a non-negative number."
                    )
                self.goal_tolerance = float(param.value)
                self.get_logger().info(f"Goal tolerance updated: {self.goal_tolerance}.")

            elif param.name == 'heading_tolerance':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="heading_tolerance must be a non-negative number."
                    )
                self.heading_tolerance = float(param.value)
                self.get_logger().info(f"Heading tolerance updated: {self.heading_tolerance}.")

            elif param.name == 'min_linear_speed':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="min_linear_speed must be a non-negative number."
                    )
                self.min_linear_speed = float(param.value)
                self.get_logger().info(f"Min linear speed updated: {self.min_linear_speed}.")

            elif param.name == 'max_linear_speed':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="max_linear_speed must be a number."
                    )
                self.max_linear_speed = float(param.value)
                self.get_logger().info(f"Max linear speed updated: {self.max_linear_speed}.")

            elif param.name == 'min_angular_speed':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="min_angular_speed must be a number."
                    )
                self.min_angular_speed = float(param.value)
                self.get_logger().info(f"Min angular speed updated: {self.min_angular_speed}.")

            elif param.name == 'max_angular_speed':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="max_angular_speed must be a number."
                    )
                self.max_angular_speed = float(param.value)
                self.get_logger().info(f"Max angular speed updated: {self.max_angular_speed}.")

            elif param.name == 'auto_request_next':
                if not isinstance(param.value, bool):
                    return SetParametersResult(
                        successful=False,
                        reason="auto_request_next must be true or false."
                    )
                self.auto_request_next = param.value
                self.get_logger().info(f"Auto‐request next waypoint: {self.auto_request_next}.")
            
            elif param.name == 'velocity_scale_factor':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="velocity_scale_factor must be a non-negative number."
                    )
                self.velocity_scale_factor = float(param.value)
                self.get_logger().info(f"Velocity scale factor updated: {self.velocity_scale_factor}.")

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = PIDPointController()
    except Exception as e:
        print(f"[FATAL] PIDPointController failed to initialize: {e}.", file=sys.stderr)
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