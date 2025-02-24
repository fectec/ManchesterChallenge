#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        
        # Parameters
        self.declare_parameter("Kp", 1.0)
        self.declare_parameter("Kd", 1.0)
        self.declare_parameter("Ki", 1.0)
        self.declare_parameter("sample_time", 1.0)
        
        # Parameters as variables
        self.kp = self.get_parameter("Kp").value
        self.kd = self.get_parameter("Kd").value
        self.ki = self.get_parameter("Ki").value
        self.sample_time = self.get_parameter("sample_time").value
        
        # Constants for output limiting
        self.MAX_OUTPUT = 1000.0  # Adjust based on your system requirements
        self.MIN_OUTPUT = -1000.0
        
        # Variables
        self.motor_output = 0.0
        self.setpoint = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Subscribers, publishers & timers
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
        self.timer = self.create_timer(self.sample_time, self.timer_callback)
        
        # Messages
        self.motor_input_msg = Float32()
        
        # Node started
        self.get_logger().info("Controller Node Started \U0001F680")
    
    def motor_output_callback(self, msg):
        self.motor_output = msg.data
    
    def setpoint_callback(self, msg):
        self.setpoint = msg.data
    
    def limit_output(self, value):
        """Limit the output to be within valid float32 bounds"""
        return max(min(value, self.MAX_OUTPUT), self.MIN_OUTPUT)
    
    def timer_callback(self):
        # Calculate error
        error = self.setpoint - self.motor_output
        
        # Proportional term
        p_term = self.kp * error
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / self.sample_time
        
        # Integral term with anti-windup
        self.integral += error * self.sample_time
        i_term = self.ki * self.integral
        
        # Calculate control output
        control_output = p_term + d_term + i_term
        
        # Limit the control output
        limited_output = self.limit_output(control_output)
        
        # Anti-windup: if output is saturated, don't increase integral term
        if abs(limited_output - control_output) > 0:
            # Reset integral term to prevent windup
            self.integral -= error * self.sample_time
        
        # Save the current error for the next iteration
        self.prev_error = error
        
        # Publish the limited result
        self.motor_input_msg.data = limited_output
        self.motor_input_pub.publish(self.motor_input_msg)

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