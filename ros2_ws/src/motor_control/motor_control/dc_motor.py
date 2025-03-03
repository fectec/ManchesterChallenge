#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

class DCMotorNode(Node):
    def __init__(self):
        super().__init__("DCMotor")

        # Declare parameters
        self.declare_parameter("sample_time", 1.0)         # System sample time in seconds
        self.declare_parameter("gain_K", 1.0)              # System gain K
        self.declare_parameter("tau_T", 1.0)               # System time constant Tau
        self.declare_parameter("initial_conditions", 1.0)  # System initial conditions

        # Get parameters as variables
        self.sample_time = self.get_parameter("sample_time").value
        self.gain_K = self.get_parameter("gain_K").value
        self.tau_T = self.get_parameter("tau_T").value
        self.initial_conditions = self.get_parameter("initial_conditions").value

        # Variables for simulation state
        self.motor_input_u = 0.0
        self.motor_output_y = self.initial_conditions

        # Subscribers, publishers, and timers
        self.motor_input_sub = self.create_subscription(
            Float32, "motor_input_u", self.motor_input_callback, 10
        )
        self.motor_output_pub = self.create_publisher(Float32, "motor_output_y", 10)
        self.timer = self.create_timer(self.sample_time, self.timer_callback)

        # Message for publishing motor output
        self.motor_output_msg = Float32()

        # Parameter callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Set service callback
        self.simulation_running = False
        self.srv = self.create_service(SetProcessBool, 'EnableDynamicSystemNode', self.enable_dynamic_system_node)

        # Node startup log
        self.get_logger().info("Dynamical System Node Started 🚀")

    # Timer callback: Simulate the DC Motor dynamics
    def timer_callback(self):
        if not self.simulation_running:
            return

        # DC Motor simulation equation:
        # y[k+1] = y[k] + ((-1/τ)*y[k] + (K/τ)*u[k]) * T_s
        self.motor_output_y += (
            -1.0 / self.tau_T * self.motor_output_y +
            self.gain_K / self.tau_T * self.motor_input_u
        ) * self.sample_time

        # Publish the updated motor output
        self.motor_output_msg.data = self.motor_output_y
        self.motor_output_pub.publish(self.motor_output_msg)

    # Subscriber callback: Update motor input
    def motor_input_callback(self, msg):
        self.motor_input_u = msg.data

     # Service callback to start / stop simulation
    def enable_dynamic_system_node(self, request, response):
        if request.enable:
            self.simulation_running = True
            self.get_logger().info("🚀 Dynamic System Simulation Started")
            response.success = True
            response.message = "Dynamic System Simulation Started Successfully"
        else:
            self.simulation_running = False
            self.get_logger().info("🚀 Dynamic System Simulation Stopped")
            response.success = True
            response.message = "Dynamic System Simulation Stopped Successfully"
        
        return response

    # Parameter callback: Validate and apply dynamic parameter updates
    def parameter_callback(self, params):
        for param in params:
            if param.name == "sample_time":
                if param.value <= 0.0:
                    self.get_logger().warn("Invalid sample_time value.")
                    return SetParametersResult(
                        successful=False,
                        reason="Sample time must be greater than 0."
                    )
                else:
                    self.sample_time = param.value
                    self.get_logger().info(f"Sample time updated to {self.sample_time}.")
                    self.timer.cancel()
                    self.timer = self.create_timer(self.sample_time, self.timer_callback)
            elif param.name == "tau_T":
                if param.value <= 0.0:
                    self.get_logger().warn("Invalid tau_T value.")
                    return SetParametersResult(
                        successful=False,
                        reason="Time constant (tau_T) must be greater than 0."
                    )
                else:
                    self.tau_T = param.value
                    self.get_logger().info(f"Time constant (tau_T) updated to {self.tau_T}.")
            elif param.name == "gain_K":
                self.gain_K = param.value
                self.get_logger().info(f"System gain (gain_K) updated to {self.gain_K}.")
            elif param.name == "initial_conditions":
                self.initial_conditions = param.value
                self.motor_output_y = self.initial_conditions
                self.get_logger().info(f"Initial conditions updated to {self.initial_conditions} and motor output reset accordingly.")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = DCMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("DC Motor Node shutting down gracefully.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()