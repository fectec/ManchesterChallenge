#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

class DCMotorNode(Node):
    """
    Simulates a first-order DC motor system using discrete-time update:
    y[k+1] = y[k] + ((-1/τ)*y[k] + (K/τ)*u[k]) * T_s
    Parameters and simulation can be dynamically changed or enabled via services.
    """
    def __init__(self):
        super().__init__("DCMotor")
       
        # Declare system parameters
        self.declare_parameter("sample_time", 1.0)         # System sample time (s)
        self.declare_parameter("gain_K", 1.0)              # System gain K
        self.declare_parameter("tau_T", 1.0)               # System time constant Tau
        self.declare_parameter("initial_conditions", 1.0)  # System initial conditions

        # Retrieve parameter values
        self.sample_time = self.get_parameter("sample_time").value
        self.gain_K = self.get_parameter("gain_K").value
        self.tau_T = self.get_parameter("tau_T").value
        self.initial_conditions = self.get_parameter("initial_conditions").value

        # Initialize state
        self.motor_input_u = 0.0
        self.motor_output_y = self.initial_conditions

        # Set up pub/sub/timer
        self.create_subscription(Float32, "motor_input_u", self.motor_input_callback, 10)
        self.motor_output_pub = self.create_publisher(Float32, "motor_output_y", 10)
        self.timer = self.create_timer(self.sample_time, self.timer_callback)

        # Preallocate message object
        self.motor_output_msg = Float32()

        # Allow dynamic reconfiguration of parameters
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Provide service to enable/disable simulation
        self.simulation_running = False
        self.srv = self.create_service(SetProcessBool, 'EnableDynamicSystemNode', self.enable_dynamic_system_node)

        self.get_logger().info("Dynamical System Node Started.")

    def timer_callback(self):
        if not self.simulation_running:
            return

        # Discrete-time first-order model update
        self.motor_output_y += (
            -1.0 / self.tau_T * self.motor_output_y +
            self.gain_K / self.tau_T * self.motor_input_u
        ) * self.sample_time

        self.motor_output_msg.data = self.motor_output_y
        self.motor_output_pub.publish(self.motor_output_msg)

    def motor_input_callback(self, msg):
        self.motor_input_u = msg.data

    def enable_dynamic_system_node(self, request, response):
        self.simulation_running = request.enable
        if request.enable:
            self.get_logger().info("Dynamical System Simulation Started.")
            response.message = "Dynamical System Simulation Started Successfully."
        else:
            self.get_logger().info("Dynamical System Simulation Stopped.")
            response.message = "Dynamical System Simulation Stopped Successfully."
        response.success = True
        return response

    def parameter_callback(self, params):
        # Update parameters on-the-fly and validate values
        for param in params:
            if param.name == "sample_time":
                if param.value <= 0.0:
                    self.get_logger().warn("Invalid sample_time value.")
                    return SetParametersResult(successful=False, reason="Sample time must be greater than 0.")
                self.sample_time = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.sample_time, self.timer_callback)
                self.get_logger().info(f"Sample time updated to {self.sample_time}.")

            elif param.name == "tau_T":
                if param.value <= 0.0:
                    self.get_logger().warn("Invalid tau_T value.")
                    return SetParametersResult(successful=False, reason="Time constant (tau_T) must be greater than 0.")
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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()