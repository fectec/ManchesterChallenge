#!/usr/bin/env python3

import math
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
      - odom (nav_msgs/Odometry): Current robot pose and orientation
    
    Service Clients:
      - pid_point_controller/next_pid_waypoint (custom_interfaces/srv/NextPIDWaypoint): Requests the next waypoint
    
    Service Servers:
      - pid_point_controller/controller_on (std_srvs/SetBool): Enable/disable controller output
    
    Publishes to:
      - cmd_vel (geometry_msgs/Twist): Commanded linear and angular velocity

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

    This node includes a service (pid_point_controller/controller_on) that can enable/disable the controller output.
    When disabled, the controller publishes zero Twist commands.
    """

    def __init__(self):
        super().__init__('pid_point_controller')
        
        # Declare parameters
        self.declare_parameter('update_rate',         30.0)        # Hz

        self.declare_parameter('Kp_V',                0.15)      
        self.declare_parameter('Ki_V',                0.0)
        self.declare_parameter('Kd_V',                0.0)

        self.declare_parameter('Kp_Omega',            0.09)
        self.declare_parameter('Ki_Omega',            0.0)
        self.declare_parameter('Kd_Omega',            0.0)

        self.declare_parameter('goal_tolerance',      0.08)         # m
        self.declare_parameter('heading_tolerance',   0.09)         # m

        self.declare_parameter('min_linear_speed',    0.1)          # m/s
        self.declare_parameter('max_linear_speed',    0.17)         # m/s

        self.declare_parameter('min_angular_speed',  -0.15)         # rad/s
        self.declare_parameter('max_angular_speed',    0.15)        # rad/s

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
        self.controller_enabled = True
        
        # PID internals 
        self.integral_e_d = 0.0
        self.integral_e_theta = 0.0
        self.last_signed_e_d = 0.0
        self.last_e_theta = 0.0

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )
        
        self.waypoint_pub   = self.create_publisher(Pose2D,  'pid_point_controller/waypoint', 10)
        self.signed_e_d_pub   = self.create_publisher(Float32, 'pid_point_controller/signed_e_d', 10)
        self.abs_e_d_pub = self.create_publisher(Float32, 'pid_point_controller/abs_e_d', 10)
        self.e_theta_pub    = self.create_publisher(Float32, 'pid_point_controller/e_theta', 10)

        # Subscriber for odometry
        self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        
        # Service client for requesting the next waypoint
        self.next_waypoint_client = self.create_client(
            NextPIDWaypoint, 'pid_point_controller/next_pid_waypoint')
        while not self.next_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pid_point_controller/next_pid_waypoint service not available, waiting...')
            
        # Service server to enable/disable controller output
        self.create_service(SetBool, 'pid_point_controller/controller_on', self.controller_on_callback)

        # Request the first waypoint
        self.request_next_waypoint(True)

        self.get_logger().info("PIDPointController Start.")
    
    def odom_callback(self, msg: Odometry) -> None:
        """Update the current robot pose from odometry message."""
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose.theta = yaw
    
    def request_next_waypoint(self, previous_reached: bool = True) -> None:
        """Request the next waypoint from the waypoint service."""
        request = NextPIDWaypoint.Request()
        request.previous_reached = previous_reached
        future = self.next_waypoint_client.call_async(request)
        future.add_done_callback(self.process_waypoint_response)
    
    def process_waypoint_response(self, future) -> None:
        """Process the asynchronous response containing the next waypoint."""
        try:
            response = future.result()
            
            if response.completed:
                self.get_logger().info("Path completed! No more waypoints available.")
                self.path_completed = True
                self.goal_active = False
                self.cmd_pub.publish(Twist())
                return
            
            self.current_waypoint_id = response.waypoint_id
            self.waypoint.x = response.goal.pose.position.x
            self.waypoint.y = response.goal.pose.position.y
            self.waypoint.theta = response.goal.theta
            self.waypoint_pub.publish(self.waypoint)
            
            # Reset PID state for new waypoint
            self.integral_e_d = 0.0
            self.integral_e_theta = 0.0
            self.last_signed_e_d = 0.0
            self.last_e_theta = 0.0
            self.goal_active = True
            
            BLUE = "\033[1;34m"  
            RESET = "\033[0m"     

            msg = (
                f"New waypoint {self.current_waypoint_id + 1} -> "
                f"x={self.waypoint.x:.2f}, y={self.waypoint.y:.2f}."
            )
            self.get_logger().info(f"{BLUE}{msg}{RESET}")

        except Exception as e:
            self.get_logger().error(f"Error processing waypoint response: {e}.")

    def controller_on_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Service callback to enable or disable the controller output."""
        self.controller_enabled = request.data
        response.success = True
        
        if self.controller_enabled:
            response.message = "Controller enabled."
            self.get_logger().info("Controller enabled via service.")
        else:
            response.message = "Controller disabled."
            # Reset PID state when disabling
            self.integral_e_d = 0.0
            self.integral_e_theta = 0.0
            self.last_signed_e_d = 0.0
            self.last_e_theta = 0.0
            self.get_logger().info("Controller disabled via service.")
            
        return response
    
    def control_loop(self) -> None:
        """Main control loop running at update_rate; computes and publishes velocity commands."""
        # Calculate dt based on update rate
        dt = 1.0 / self.update_rate
        
        # If controller is disabled, publish zero command and return
        if not self.controller_enabled:
            self.cmd_pub.publish(Twist())
            self.get_logger().debug("Controller disabled; publishing zero command.", 
                                  throttle_duration_sec=2.0)
            return
        
        # Run PID control only when a goal is active
        if not self.goal_active or self.path_completed:
            self.cmd_pub.publish(Twist())
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

            PURPLE = "\033[1;35m"
            RESET = "\033[0m"

            msg = (
                f"Waypoint {self.current_waypoint_id + 1} reached at "
                f"x={self.current_pose.x:.3f}, y={self.current_pose.y:.3f}."
            )
            self.get_logger().info(f"{PURPLE}{msg}{RESET}")
            
            # Mark the current goal as completed
            self.goal_active = False
            
            # Automatically request the next waypoint
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

        # Save errors for the next iteration
        self.last_signed_e_d = signed_e_d
        self.last_e_theta = e_theta

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

        # Log control info using ROS 2 throttle
        self.get_logger().info(
            f"Control -> dist_err={abs_e_d:.3f}, ang_err={e_theta:.3f}, V={V:.3f}, Omega={Omega:.3f}",
            throttle_duration_sec=1.0
        )

    def apply_velocity_constraints(self, velocity: float, min_vel: float, max_vel: float) -> float:
        """Clamp the velocity to specified minimum and maximum limits."""
        if abs(velocity) < 0.01:    
            return 0.0
            
        # Apply minimum threshold (to overcome friction/inertia)
        if abs(velocity) < abs(min_vel):
            velocity = abs(min_vel) * (1 if velocity > 0 else -1)
            
        # Apply maximum limit (saturation)
        if abs(velocity) > abs(max_vel):
            velocity = abs(max_vel) * (1 if velocity > 0 else -1)
            
        return velocity

    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validates and applies updated node parameters."""
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
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

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
                self.get_logger().info(f"goal_tolerance updated: {self.goal_tolerance} m.")

            elif param.name == 'heading_tolerance':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="heading_tolerance must be a non-negative number."
                    )
                self.heading_tolerance = float(param.value)
                self.get_logger().info(f"heading_tolerance updated: {self.heading_tolerance} m.")

            elif param.name == 'min_linear_speed':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="min_linear_speed must be a non-negative number."
                    )
                self.min_linear_speed = float(param.value)
                self.get_logger().info(f"min_linear_speed updated: {self.min_linear_speed} m/s.")

            elif param.name == 'max_linear_speed':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="max_linear_speed must be a number."
                    )
                self.max_linear_speed = float(param.value)
                self.get_logger().info(f"max_linear_speed updated: {self.max_linear_speed} m/s.")

            elif param.name == 'min_angular_speed':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="min_angular_speed must be a number."
                    )
                self.min_angular_speed = float(param.value)
                self.get_logger().info(f"min_angular_speed updated: {self.min_angular_speed} rad/s.")

            elif param.name == 'max_angular_speed':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="max_angular_speed must be a number."
                    )
                self.max_angular_speed = float(param.value)
                self.get_logger().info(f"max_angular_speed updated: {self.max_angular_speed} rad/s.")
            
            elif param.name == 'velocity_scale_factor':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="velocity_scale_factor must be a non-negative number."
                    )
                self.velocity_scale_factor = float(param.value)
                self.get_logger().info(f"velocity_scale_factor updated: {self.velocity_scale_factor}.")

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