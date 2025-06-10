#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from std_srvs.srv import SetBool, Trigger

class LineFollowController(Node):
    """
    Line following controller that maintains constant linear velocity and adjusts 
    angular velocity based on line detection error to keep the robot centered on the lane.

    Subscribes to:
        - centroid_error (std_msgs/Float32): Normalized line centroid error (-1.0 to 1.0)

    Publishes to:
      - cmd_vel (geometry_msgs/Twist): Commanded linear and angular velocity

    Service Clients:
      - line_detection/is_line_detected (std_srvs/Trigger): Checks if line is detected

    Service Servers:
      - line_follow_controller/controller_on (std_srvs/SetBool): Enable/disable controller

    Control logic:
      - If controller_on is False: stops the robot (V=0, Omega=0)
      - If line is not detected: stops the robot (V=0, Omega=0)
      - If controller_on is True AND line is detected: maintains constant forward velocity and PID steering
    """
    
    def __init__(self):
        super().__init__('line_follow_controller')
        
        # Declare parameters
        self.declare_parameter('update_rate', 30.0)

        self.declare_parameter('linear_velocity', 0.07)

        self.declare_parameter('Kp_Omega', 0.35)
        self.declare_parameter('Ki_Omega', 0.1)
        self.declare_parameter('Kd_Omega', 0.1)

        self.declare_parameter('max_angular_speed', 1.5)

        self.declare_parameter('velocity_scale_factor', 1.0)

        self.declare_parameter('steering_deadband', 0.03)
    
        # Retrieve parameters
        self.update_rate = self.get_parameter('update_rate').value

        self.linear_velocity = self.get_parameter('linear_velocity').value

        self.Kp_Omega = self.get_parameter('Kp_Omega').value
        self.Ki_Omega = self.get_parameter('Ki_Omega').value
        self.Kd_Omega = self.get_parameter('Kd_Omega').value

        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        self.velocity_scale_factor = self.get_parameter('velocity_scale_factor').value
        self.steering_deadband = self.get_parameter('steering_deadband').value
        
        # Timer for periodic processing
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        # Register the parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Validate initial parameters
        init_params = [
            Parameter('update_rate', Parameter.Type.DOUBLE, self.update_rate),
            Parameter('linear_velocity', Parameter.Type.DOUBLE, self.linear_velocity),
            Parameter('Kp_Omega', Parameter.Type.DOUBLE, self.Kp_Omega),
            Parameter('Ki_Omega', Parameter.Type.DOUBLE, self.Ki_Omega),
            Parameter('Kd_Omega', Parameter.Type.DOUBLE, self.Kd_Omega),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, self.max_angular_speed),
            Parameter('velocity_scale_factor', Parameter.Type.DOUBLE, self.velocity_scale_factor),
            Parameter('steering_deadband', Parameter.Type.DOUBLE, self.steering_deadband),
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")

        # Cached line detection status
        self.line_detected_cached = False
        self.last_line_status_check = 0.0
        self.line_status_check_interval = 0.1  # Check every 100ms
        
        # Controller state variables
        self.line_error = 0.0
        self.controller_enabled = True
        
        # PID state
        self.integral_error = 0.0
        self.last_error = 0.0

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )

        # Subscribers
        self.create_subscription(
            Float32, 
            'line_detection/centroid_error', 
            self.error_callback, 
            qos.qos_profile_sensor_data
        )
        
        # Service server for controller enable/disable
        self.create_service(
            SetBool, 
            'line_follow_controller/controller_on', 
            self.controller_on_callback
        )

        # Service client to check line detection status
        self.line_status_client = self.create_client(
            Trigger, 
            'line_detection/is_line_detected'
        )

        self.get_logger().info("LineFollowController Start.")
    
    def error_callback(self, msg):
        """Update line error from centroid detection."""
        self.line_error = msg.data
        
    def controller_on_callback(self, request, response):
        """Enable/disable controller service callback."""
        old_state = self.controller_enabled
        self.controller_enabled = request.data
        response.success = True
        
        if old_state != self.controller_enabled:
            if self.controller_enabled:
                response.message = "Controller enabled."
                self.get_logger().info("Controller enabled via service.")
            else:
                response.message = "Controller disabled."
                # Reset PID state when disabling
                self.integral_error = 0.0
                self.last_error = 0.0
                self.get_logger().info("Controller disabled via service.")
        else:
            response.message = f"Controller already {'enabled' if self.controller_enabled else 'disabled'}."
            
        return response

    def check_line_status_async(self):
        """Asynchronously check line detection status and cache result."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Only check periodically to avoid overwhelming the service
        if current_time - self.last_line_status_check < self.line_status_check_interval:
            return
            
        self.last_line_status_check = current_time
        
        if not self.line_status_client.service_is_ready():
            self.get_logger().warn("Line detection service not ready.", throttle_duration_sec=2.0)
            return
            
        try:
            request = Trigger.Request()
            future = self.line_status_client.call_async(request)
            future.add_done_callback(self.line_status_callback)
        except Exception as e:
            self.get_logger().warn(f"Error initiating line detection service call: {e}")

    def line_status_callback(self, future):
        """Callback for line detection status service response."""
        try:
            result = future.result()
            if result is not None:
                old_status = self.line_detected_cached
                self.line_detected_cached = result.success
                
                # Log status changes
                if old_status != self.line_detected_cached:
                    self.get_logger().info(f"Line detection status changed: {self.line_detected_cached}.")
                    
        except Exception as e:
            self.get_logger().warn(f"Error processing line detection service response: {e}")

    def control_loop(self):
        """Main control loop with cached line detection."""
        # Check line status asynchronously
        self.check_line_status_async()
        
        # Calculate dt based on update rate
        dt = 1.0 / self.update_rate
        
        # Create command message
        cmd = Twist()
        
        # Check if controller is enabled and line is detected
        if not self.controller_enabled:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            status = "CONTROLLER_DISABLED."
        elif not self.line_detected_cached:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            status = "NO_LINE_DETECTED."
            # Reset PID when line is lost
            self.integral_error = 0.0
            self.last_error = 0.0
        else:
            # Line detected and controller enabled - do PID control
            error = self.line_error
            
            # Apply deadband
            if abs(error) < self.steering_deadband:
                error = 0.0
            
            # PID calculation (flip sign for correct steering)
            steering_error = -error
            
            # Integral with windup protection
            self.integral_error += steering_error * dt
            max_integral = self.max_angular_speed / max(self.Ki_Omega, 0.001)
            self.integral_error = max(-max_integral, min(max_integral, self.integral_error))
            
            # Derivative
            derivative = (steering_error - self.last_error) / dt
            self.last_error = steering_error
            
            # PID output
            angular_vel = (self.Kp_Omega * steering_error + 
                          self.Ki_Omega * self.integral_error + 
                          self.Kd_Omega * derivative)
            
            # Apply limits and scaling
            angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
            
            cmd.linear.x = self.linear_velocity * self.velocity_scale_factor
            cmd.angular.z = angular_vel * self.velocity_scale_factor
            status = "ACTIVE"

        # Publish commands
        self.cmd_pub.publish(cmd)

        # Log periodically using ROS 2 throttle
        self.get_logger().info(
            f"Status: {status}, V={cmd.linear.x:.3f}, Ω={cmd.angular.z:.3f}, LineDetected: {self.line_detected_cached}", 
            throttle_duration_sec=1.0
        )

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
                if hasattr(self, 'timer') and self.timer is not None:
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

            elif param.name == 'linear_velocity':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="linear_velocity must be a number."
                    )
                self.linear_velocity = float(param.value)
                self.get_logger().info(f"linear_velocity updated: {self.linear_velocity}.")

            elif param.name in ('Kp_Omega', 'Ki_Omega', 'Kd_Omega'):
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a number."
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
                self.get_logger().info(f"max_angular_speed updated: {self.max_angular_speed}.")

            elif param.name in ('velocity_scale_factor', 'steering_deadband'):
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be >= 0."
                    )
                setattr(self, param.name, float(param.value))
                self.get_logger().info(f"{param.name} updated: {param.value}.")

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = LineFollowController()
    except Exception as e:
        print(f"[FATAL] LineFollowController failed to initialize: {e}", file=sys.stderr)
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