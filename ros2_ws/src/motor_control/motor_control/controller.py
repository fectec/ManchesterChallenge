#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        
        # Declare parameters
        self.declare_parameter("Kp", 1.0)
        self.declare_parameter("Kd", 1.0)
        self.declare_parameter("Ki", 1.0)
        self.declare_parameter("sample_time", 1.0)
        
        # Retrieve parameters as variables
        self.kp = self.get_parameter("Kp").value
        self.kd = self.get_parameter("Kd").value
        self.ki = self.get_parameter("Ki").value
        self.sample_time = self.get_parameter("sample_time").value
        
        # Constants to limit the output
        self.MAX_OUTPUT = 50.0 
        self.MIN_OUTPUT = -50.0
        
        # Internal variables
        self.motor_output = 0.0
        self.setpoint = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Subscribers, publishers, and timer
        self.motor_output_sub = self.create_subscription(
            Float32, 
            "motor_output_y", 
            self.motor_output_callback, 
            10
        )
        self.setpoint_sub = self.create_subscription(
            Float32, 
            "setpoint", 
            self.setpoint_callback, 
            10
        )
        self.motor_input_pub = self.create_publisher(Float32, "motor_input_u", 10)
        # Additional publisher to debug the error signal
        self.error_pub = self.create_publisher(Float32, "error", 10)
        
        self.timer = self.create_timer(self.sample_time, self.timer_callback)
        
        # Set up the parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Output message
        self.motor_input_msg = Float32()
        
        # Node started log message
        self.get_logger().info("Controller Node Started 🚀")
    
    # Callback for receiving motor output values
    def motor_output_callback(self, msg):
        self.motor_output = msg.data
    
    # Callback for receiving setpoint values
    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    # Parameter callback to update controller parameters dynamically
    def parameter_callback(self, params):
        for param in params:
            if param.name == "Kp":
                self.kp = param.value
                self.get_logger().info(f"Kp updated: {self.kp}")
            elif param.name == "Ki":
                self.ki = param.value
                self.get_logger().info(f"Ki updated: {self.ki}")
            elif param.name == "Kd":
                self.kd = param.value
                self.get_logger().info(f"Kd updated: {self.kd}")
            elif param.name == "sample_time":
                # Check that sample_time is greater than zero
                if param.value <= 0.0:
                    self.get_logger().warn("Invalid value for sample_time. It must be greater than 0.")
                    return SetParametersResult(successful=False, reason="Sample time must be greater than 0.")
                self.sample_time = param.value
                self.get_logger().info(f"Sample time updated: {self.sample_time}")
                # Restart the timer with the new sample_time
                self.timer.cancel()
                self.timer = self.create_timer(self.sample_time, self.timer_callback)
        return SetParametersResult(successful=True)
    
    # Helper function to limit the output value within valid bounds
    def limit_output(self, value):
        return max(min(value, self.MAX_OUTPUT), self.MIN_OUTPUT)
    
    # Timer callback: Compute PID control output and publish it along with error for debugging
    def timer_callback(self):
        # Calculate error between setpoint and motor output
        error = self.setpoint - self.motor_output
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * self.sample_time
        i_term = self.ki * self.integral
        
        # Derivative term (avoid division by zero)
        d_term = 0.0
        if self.sample_time > 0:
            d_term = self.kd * (error - self.prev_error) / self.sample_time
        
        # Compute the control output
        control_output = p_term + i_term + d_term
        
        # Update the previous error for the next iteration
        self.prev_error = error
        
        # Publish the limited control output
        self.motor_input_msg.data = control_output
        self.motor_input_pub.publish(self.motor_input_msg)
        
        # Publish the error for debugging
        error_msg = Float32()
        error_msg.data = error
        self.error_pub.publish(error_msg)
        # Optionally log the error value
        self.get_logger().debug(f"Error: {error}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()