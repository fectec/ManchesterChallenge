#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from std_srvs.srv import SetBool

class LineFollowController(Node):
    """
    Line following controller that maintains constant linear velocity and adjusts 
    angular velocity based on line detection error to keep the robot centered on the lane.

    Subscribes to:
      - centroid_error (std_msgs/Float32): Normalized line centroid error (-1.0 to 1.0)
    
    Publishes to:
      - cmd_vel (geometry_msgs/Twist): Commanded linear and angular velocity

    Control logic:
      - Maintains constant forward velocity (V = constant)
      - Computes angular velocity using PID control based on line centroid error:
            Omega = Kp_Omega * e_line + Ki_Omega * ∫(e_line) dt + Kd_Omega * d(e_line)/dt
      
      Where:
        - e_line is the normalized centroid error from line detection (-1.0 = left, +1.0 = right)
        - Positive angular velocity turns left (counter-clockwise)
        - Negative angular velocity turns right (clockwise)

    The controller includes a service to enable/disable output and handles cases where
    no line is detected by maintaining the last known steering command briefly.
    """

    def __init__(self):
        super().__init__('line_follow_controller')
        
        # Declare parameters
        self.declare_parameter('update_rate',           60.0)         # Hz
        self.declare_parameter('linear_velocity',       0.06)         # m/s 
        self.declare_parameter('Kp_Omega',              0.2)          
        self.declare_parameter('Ki_Omega',              0.1)          
        self.declare_parameter('Kd_Omega',              0.1)          
        self.declare_parameter('max_angular_speed',     1.5)          # rad/s 
        self.declare_parameter('line_timeout',          0.5)          # s 
        self.declare_parameter('velocity_scale_factor', 1.0)       
        self.declare_parameter('steering_deadband',    0.06)         
    
        # Retrieve parameters
        self.update_rate                = self.get_parameter('update_rate').value
        self.linear_velocity            = self.get_parameter('linear_velocity').value
        self.Kp_Omega                   = self.get_parameter('Kp_Omega').value
        self.Ki_Omega                   = self.get_parameter('Ki_Omega').value
        self.Kd_Omega                   = self.get_parameter('Kd_Omega').value
        self.max_angular_speed          = self.get_parameter('max_angular_speed').value
        self.line_timeout               = self.get_parameter('line_timeout').value
        self.velocity_scale_factor      = self.get_parameter('velocity_scale_factor').value
        self.steering_deadband          = self.get_parameter('steering_deadband').value

        # Timer for the control loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        # Register parameter update callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Validate initial parameter values
        init_params = [
            Parameter('update_rate',                Parameter.Type.DOUBLE, self.update_rate),
            Parameter('linear_velocity',            Parameter.Type.DOUBLE, self.linear_velocity),
            Parameter('Kp_Omega',                   Parameter.Type.DOUBLE, self.Kp_Omega),
            Parameter('Ki_Omega',                   Parameter.Type.DOUBLE, self.Ki_Omega),
            Parameter('Kd_Omega',                   Parameter.Type.DOUBLE, self.Kd_Omega),
            Parameter('max_angular_speed',          Parameter.Type.DOUBLE, self.max_angular_speed),
            Parameter('line_timeout',               Parameter.Type.DOUBLE, self.line_timeout),
            Parameter('velocity_scale_factor',      Parameter.Type.DOUBLE, self.velocity_scale_factor),
            Parameter('steering_deadband',          Parameter.Type.DOUBLE, self.steering_deadband),
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")

        # Controller state variables
        self.line_error = 0.0                    # Current line centroid error (-1.0 to 1.0)
        self.line_detected = False               # Whether a line is currently detected
        self.last_line_time = None               # Timestamp of last line detection
        
        # PID control variables for angular velocity
        self.integral_e_line = 0.0               # Accumulated error for integral term
        self.last_e_line = 0.0                   # Previous error for derivative term
        self.last_time = None                    # Previous control loop timestamp
        
        # Control output state
        self.last_angular_velocity = 0.0        # Last commanded angular velocity
        self.controller_enabled = True          # Enable/disable controller output
        self.has_received_data = False          # Flag to track if we've received any image data

        # Limit logging frequency
        self.last_log_time = 0.0

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )
        
        # Publishers for debugging/monitoring
        self.angular_cmd_pub = self.create_publisher(Float32, 'line_pid/angular_cmd', 10)

        # Subscriber for line detection centroid error
        self.create_subscription(
            Float32,
            'centroid_error',
            self.line_centroid_callback,
            qos.qos_profile_sensor_data
        )
        
        # Service server to enable/disable controller
        self.create_service(SetBool, 'line_pid/pid_toggle', self.pid_toggle_callback)

        self.get_logger().info("LineFollowController Start.")
    
    def line_centroid_callback(self, msg: Float32) -> None:
        """Update line detection error from the line detection node."""
        self.line_error = msg.data
        self.line_detected = True
        self.last_line_time = self.get_clock().now().nanoseconds * 1e-9
        self.has_received_data = True  
        
    def pid_toggle_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Service callback to enable or disable the controller output."""
        self.controller_enabled = not request.data  
        response.success = True
        if not self.controller_enabled:
            response.message = "Line follow controller stopped."
            # Reset PID state when stopping
            self.integral_e_line = 0.0
            self.last_e_line = 0.0
            self.last_time = None
        else:
            response.message = "Line follow controller resumed."
        return response
    
    def control_loop(self) -> None:
        """Main control loop running at update_rate; computes and publishes velocity commands."""
        # Get current timestamp
        now_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Initialize timing on first run
        if self.last_time is None:
            self.last_time = now_time
            return
        
        # Calculate elapsed time since last update
        dt = now_time - self.last_time
        if dt < 1.0 / self.update_rate:
            return
        self.last_time = now_time
        
        # If controller is disabled, publish zero velocity
        if not self.controller_enabled:
            self.cmd_pub.publish(Twist())
            return
        
        # Don't publish anything until we've received image data
        if not self.has_received_data:
            self.get_logger().debug("Waiting for image data...")
            return

        # Check if we've lost line detection (timeout)
        current_time = now_time
        line_lost = False
        if self.last_line_time is not None:
            time_since_line = current_time - self.last_line_time
            if time_since_line > self.line_timeout:
                line_lost = True
                self.line_detected = False

        # Determine the error signal to use for control
        if self.line_detected and not line_lost:
            # Use current line detection error
            e_line = self.line_error
        else:
            # No line detected or line lost: set error to zero and reset PID state
            e_line = 0.0
            # Reset PID state to prevent using stale integral/derivative terms
            self.integral_e_line = 0.0
            self.last_e_line = 0.0
            
            if line_lost:
                self.get_logger().warn("Line detection lost - stopping steering.")
            elif self.last_line_time is None:
                # Only log this once when starting up
                pass  

        # Apply deadband to reduce small oscillations
        if abs(e_line) < self.steering_deadband:
            e_line = 0.0

        # PID control for angular velocity (steering correction)
        # Note: e_line is negative when line is to the left, positive when to the right
        # We want positive angular velocity (turn left) when line is to the left (negative error)
        # So we use -e_line to get the correct sign
        steering_error = -e_line
        
        # Integral term (with windup protection)
        self.integral_e_line += steering_error * dt
        # Clamp integral to prevent windup
        max_integral = self.max_angular_speed / max(self.Ki_Omega, 0.001)  # Prevent division by zero
        self.integral_e_line = max(-max_integral, min(max_integral, self.integral_e_line))
        
        # Derivative term
        derivative_e_line = (steering_error - self.last_e_line) / dt
        
        # Compute angular velocity using PID
        Omega = (self.Kp_Omega * steering_error + 
                 self.Ki_Omega * self.integral_e_line + 
                 self.Kd_Omega * derivative_e_line)

        # Save error for next iteration
        self.last_e_line = steering_error

        # Apply angular velocity constraints
        Omega = max(-self.max_angular_speed, min(self.max_angular_speed, Omega))
        
        # Store the computed angular velocity
        self.last_angular_velocity = Omega

        # Apply velocity scaling
        linear_vel = self.linear_velocity * self.velocity_scale_factor
        angular_vel = Omega * self.velocity_scale_factor

        # Create and publish the velocity command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

        # Publish angular command for monitoring
        self.angular_cmd_pub.publish(Float32(data=angular_vel))

        # Periodic logging (limit frequency)
        current_log_time = time.time()
        if current_log_time - self.last_log_time > 1.0:  # Log once per second
            if self.line_detected and not line_lost:
                status = "ACTIVE"
            elif line_lost:
                status = "LOST"
            elif self.last_line_time is None:
                status = "WAITING"
            else:
                status = "NO_DATA"
                
            self.get_logger().info(
                f"Line Follow -> status={status}, error={e_line:.3f}, V={linear_vel:.3f}, Omega={angular_vel:.3f}"
            )
            self.last_log_time = current_log_time

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
                # Recreate timer with new rate
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

            elif param.name == 'linear_velocity':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="linear_velocity must be a non-negative number."
                    )
                self.linear_velocity = float(param.value)
                self.get_logger().info(f"linear_velocity updated: {self.linear_velocity} m/s.")

            elif param.name in ('Kp_Omega', 'Ki_Omega', 'Kd_Omega'):
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a non-negative number."
                    )
                setattr(self, param.name, float(param.value))
                self.get_logger().info(f"{param.name} updated: {param.value}.")

            elif param.name == 'max_angular_speed':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="max_angular_speed must be > 0."
                    )
                self.max_angular_speed = float(param.value)
                self.get_logger().info(f"max_angular_speed updated: {self.max_angular_speed} rad/s.")

            elif param.name == 'line_timeout':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="line_timeout must be a non-negative number."
                    )
                self.line_timeout = float(param.value)
                self.get_logger().info(f"line_timeout updated: {self.line_timeout} s.")

            elif param.name == 'velocity_scale_factor':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="velocity_scale_factor must be a non-negative number."
                    )
                self.velocity_scale_factor = float(param.value)
                self.get_logger().info(f"velocity_scale_factor updated: {self.velocity_scale_factor}.")

            elif param.name == 'steering_deadband':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="steering_deadband must be a non-negative number."
                    )
                self.steering_deadband = float(param.value)
                self.get_logger().info(f"steering_deadband updated: {self.steering_deadband}.")

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = LineFollowController()
    except Exception as e:
        print(f"[FATAL] LineFollowController failed to initialize: {e}.", file=sys.stderr)
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