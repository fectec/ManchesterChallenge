#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool
import numpy as np

class SetpointPublisherNode(Node):
    """
    Publishes a configurable sine or square waveform as a setpoint signal.
    Supports dynamic parameter updates and simulation toggling via service.
    """
    def __init__(self):
        super().__init__("SetpointPublisher")

        # Declare parameters
        self.declare_parameter("timer_period", 1.0)         # Timer period
        self.declare_parameter("amplitude", 1.0)            # Amplitude of the wave
        self.declare_parameter("angular_frequency", 1.0)    # Angular frequency
        self.declare_parameter("wave_type", "sine")         # Wave type: sine or square

        # Retrieve parameter values
        self.timer_period = self.get_parameter("timer_period").value
        self.amplitude = self.get_parameter("amplitude").value
        self.angular_frequency = self.get_parameter("angular_frequency").value
        self.wave_type = self.get_parameter("wave_type").value

        # Publisher setup
        self.signal_publisher = self.create_publisher(Float32, "setpoint", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.parameter_callback)

        # Enable control via service
        self.simulation_running = False
        self.srv = self.create_service(SetProcessBool, 'EnableSetpointNode', self.enable_setpoint_node)

        self.get_logger().info("Setpoint Node Started.")

    def timer_callback(self):
        if not self.simulation_running:
            return
    
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Generate waveform
        if self.wave_type == "sine":
            self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * elapsed_time)
        elif self.wave_type == "square":
            self.signal_msg.data = self.amplitude * np.sign(np.sin(self.angular_frequency * elapsed_time))
        else:
            self.get_logger().warn(f"Unknown wave type: {self.wave_type}. Defaulting to sine wave.")
            self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * elapsed_time)
        
        self.signal_publisher.publish(self.signal_msg)

    def enable_setpoint_node(self, request, response):
        if request.enable:
            self.simulation_running = True
            self.get_logger().info("Setpoint Simulation Started.")
            response.success = True
            response.message = "Setpoint Simulation Started Successfully."
        else:
            self.simulation_running = False
            self.get_logger().info("Setpoint Simulation Stopped.")
            response.success = True
            response.message = "Setpoint Simulation Stopped Successfully."
        
        return response

    def parameter_callback(self, params):
        # Handle dynamic parameter updates
        for param in params:
            if param.name == "amplitude":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="Amplitude cannot be negative.")
                self.amplitude = param.value
                self.get_logger().info(f"Amplitude updated to {self.amplitude}.")

            elif param.name == "angular_frequency":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="Angular frequency cannot be negative.")
                self.angular_frequency = param.value
                self.get_logger().info(f"Angular frequency updated to {self.angular_frequency}.")

            elif param.name == "timer_period":
                if param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="Timer period must be greater than 0.")
                self.timer_period = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.timer_period, self.timer_callback)
                self.get_logger().info(f"Timer period updated to {self.timer_period}.")

            elif param.name == "wave_type":
                if param.value not in ["sine", "square"]:
                    return SetParametersResult(successful=False, reason="Invalid wave type.")
                self.wave_type = param.value
                self.get_logger().info(f"Wave type updated to {self.wave_type}.")

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