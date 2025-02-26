#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import numpy as np

class SetpointPublisherNode(Node):

    def __init__(self):
        super().__init__("setpoint_publisher")

        # Declare parameters
        self.declare_parameter("amplitude", 1.0)      # Amplitude of the sine wave
        self.declare_parameter("omega", 1.0)          # Angular frequency (omega)
        self.declare_parameter("timer_period", 1.0)   # Timer period (in seconds)

        # Get parameters as variables
        self.amplitude = self.get_parameter("amplitude").value
        self.omega = self.get_parameter("omega").value
        self.timer_period = self.get_parameter("timer_period").value

        # Create a publisher and a timer for the signal
        self.signal_publisher = self.create_publisher(Float32, "setpoint", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Message and starting time for generating the signal
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Set up parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Node startup message
        self.get_logger().info("Setpoint Node Started 🚀")

    # Timer callback: generates and publishes the sine wave signal
    def timer_callback(self):
        # Calculate the elapsed time in seconds
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Generate the sine wave signal
        self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        
        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)

    # Parameter callback: validates and applies dynamic parameter updates
    def parameter_callback(self, params):
        for param in params:
            if param.name == "amplitude":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid value for amplitude.")
                    return SetParametersResult(successful=False, reason="Amplitude cannot be negative.")
                else:
                    self.amplitude = param.value
                    self.get_logger().info(f"Amplitude updated to {self.amplitude}.")
            elif param.name == "omega":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid value for omega.")
                    return SetParametersResult(successful=False, reason="Omega cannot be negative.")
                else:
                    self.omega = param.value
                    self.get_logger().info(f"Omega updated to {self.omega}.")
            elif param.name == "timer_period":
                if param.value <= 0.0:
                    self.get_logger().warn("Invalid value for timer_period. It must be greater than 0.")
                    return SetParametersResult(successful=False, reason="Timer period must be greater than 0.")
                else:
                    self.timer_period = param.value
                    self.get_logger().info(f"Timer period updated to {self.timer_period}.")
                    # Restart the timer with the new period
                    self.timer.cancel()
                    self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    setpoint = SetpointPublisherNode()
    
    try:
        rclpy.spin(setpoint)
    except KeyboardInterrupt:
        pass
    finally:
        setpoint.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()