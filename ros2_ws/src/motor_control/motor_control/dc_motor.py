#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class DCMotorNode(Node):
    def __init__(self):
        super().__init__("dc_motor")

        # Parameters
        # System sample time in seconds
        self.declare_parameter("sys_sample_time", 1.0)
        
        # System gain K
        self.declare_parameter("sys_gain_K", 1.0)

        # System time constant Tau
        self.declare_parameter("sys_tau_T", 1.0)

        # System initial conditions
        self.declare_parameter("sys_initial_conditions", 1.0)

        # Parameters as variables
        self.sample_time = self.get_parameter("sys_sample_time").value
        self.gain = self.get_parameter("sys_gain_K").value
        self.time_constant = self.get_parameter("sys_tau_T").value
        self.initial_conditions = self.get_parameter("sys_initial_conditions").value

        # Variables
        self.input_u = 0.0
        self.output_y = self.initial_conditions
        
        # Subscribers, publishers & timers
        self.motor_input_sub = self.create_subscription(
            Float32, "motor_input_u", self.input_callback, 10
        )
        self.motor_speed_pub = self.create_publisher(Float32, "motor_output_y", 10)
        self.timer = self.create_timer(self.sample_time, self.timer_callback)

        # Messages
        self.motor_output_msg = Float32()

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Node started
        self.get_logger().info("Dynamical System Node Started \U0001F680")

    # Timer callback
    def timer_callback(self):
        # DC Motor Simulation
        # DC Motor Equation y[k+1] = y[k] + ((-1/τ) y[k] + (K/τ) u[k]) T_s
        self.output_y += (
            -1.0 / self.time_constant * self.output_y 
            + self.gain / self.time_constant * self.input_u
        ) * self.sample_time
        
        # Publish the result
        self.motor_output_msg.data = self.output_y
        self.motor_speed_pub.publish(self.motor_output_msg)

    # Subscriber callback
    def input_callback(self, input_signal):
        self.input_u = input_signal.data

    # Parameter callback
    def parameter_callback(self, params):
        for param in params:
            if param.name == "sys_sample_time":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid value.")
                    return SetParametersResult(
                        successful=False, reason="Sample time cannot be negative."
                    )
                else:
                    self.sample_time = param.value
                    self.get_logger().info(f"Sample time updated to {self.sample_time}.")
            elif param.name == "sys_gain_K":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid value.")
                    return SetParametersResult(
                        successful=False, reason="Gain cannot be negative."
                    )
                else:
                    self.gain = param.value
                    self.get_logger().info(f"Gain updated to {self.gain}.")
            elif param.name == "sys_tau_T":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid value.")
                    return SetParametersResult(
                        successful=False, reason="Time constant cannot be negative."
                    )
                else:
                    self.time_constant = param.value
                    self.get_logger().info(f"Time constant updated to {self.time_constant}.")
            elif param.name == "sys_initial_conditions":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid value.")
                    return SetParametersResult(
                        successful=False, reason="Initial conditions cannot be negative."
                    )
                else:
                    self.initial_conditions = param.value
                    self.get_logger().info(f"Initial conditions updated to {self.initial_conditions}.")
        
        return SetParametersResult(successful=True)

# Main
def main(args=None):
    rclpy.init(args=args)
    node = DCMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()