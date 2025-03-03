#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        
        # Declare parameters
        self.declare_parameter("sample_time", 1.0)
        self.declare_parameter("Kp", 1.0)
        self.declare_parameter("Kd", 1.0)
        self.declare_parameter("Ki", 1.0)
        
        # Retrieve parameters
        self.sample_time = self.get_parameter("sample_time").value
        self.kp = self.get_parameter("Kp").value
        self.kd = self.get_parameter("Kd").value
        self.ki = self.get_parameter("Ki").value
        
        # Internal variables
        self.motor_output = 0.0
        self.setpoint = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Subscribers, publishers, and timer
        self.motor_output_sub = self.create_subscription(
            Float32, "motor_output_y", self.motor_output_callback, 10
        )
        self.setpoint_sub = self.create_subscription(
            Float32, "setpoint", self.setpoint_callback, 10
        )
        self.motor_input_pub = self.create_publisher(Float32, "motor_input_u", 10)
        self.error_pub = self.create_publisher(Float32, "error", 10)
        self.timer = self.create_timer(self.sample_time, self.timer_callback)
        
        # Set up parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create a service client for /EnableDynamicSystemNode and /EnableSetpointNode 

        self.simulation_running = False
        self.setpoint_cli = self.create_client(SetProcessBool, 'EnableSetpointNode')
        while not self.setpoint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EnableSetpointNode not available...')

        self.dynamic_system_enabled = False
        self.dynamic_system_cli = self.create_client(SetProcessBool, 'EnableDynamicSystemNode')
        while not self.dynamic_system_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EnableDynamicSystemNode not available...')

        # Node startup log
        self.get_logger().info("Controller Node Started 🚀")

        # Enable Dynamic System Node
        self.enable_setpoint_node(True)
    
    # Timer callback: Compute PID control output and publish it
    def timer_callback(self):
        if not self.simulation_running:
            return
        
        # Compute error
        error = self.setpoint - self.motor_output

        # PID calculations
        p_term = self.kp * error
        self.integral += error * self.sample_time 
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error) / self.sample_time 

        # Compute control output
        control_output = p_term + i_term + d_term
        self.prev_error = error 

        # Publish control output
        motor_input_msg = Float32()
        motor_input_msg.data = control_output
        self.motor_input_pub.publish(motor_input_msg)

        # Publish error for debugging
        error_msg = Float32()
        error_msg.data = error
        self.error_pub.publish(error_msg)

    # Callback for receiving motor output values
    def motor_output_callback(self, msg):
        self.motor_output = msg.data
    
    # Callback for receiving setpoint values
    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    def enable_setpoint_node(self, enable: bool):
        request = SetProcessBool.Request()
        request.enable = enable
        
        future = self.setpoint_cli.call_async(request)
        future.add_done_callback(self.enable_setpoint_node_callback)

    def enable_dynamic_system_node(self, enable: bool):
        if enable and not self.dynamic_system_enabled:
            request = SetProcessBool.Request()
            request.enable = enable
            
            future = self.dynamic_system_cli.call_async(request)
            future.add_done_callback(self.enable_dynamic_system_node_callback)
        else:
            self.get_logger().info("Setpoint Node is already enabled, skipping service call.")
        
    def enable_setpoint_node_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.simulation_running = True
                self.get_logger().info(f'Successful EnableSetpointNode service: {response.message} / 🚀 Controller Simulation Started')
                self.enable_dynamic_system_node(True)
            else:
                self.simulation_running = False
                self.get_logger().warn(f'Failed EnableSetpointNode service: {response.message}')
        except Exception as e:
            self.simulation_running = False
            self.get_logger().error(f'Failed EnableSetpointNode service call: {e}')

    def enable_dynamic_system_node_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.dynamic_system_enabled = True  
                self.get_logger().info(f'Successful EnableDynamicSystemNode service: {response.message}')
            else:
                self.get_logger().warn(f'Failed EnableDynamicSystemNode service: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Failed EnableDynamicSystemNode service call: {e}')

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
                if param.value <= 0.0:
                    self.get_logger().warn("Invalid sample_time! It must be > 0.")
                    return SetParametersResult(successful=False, reason="Sample time must be greater than 0.")
                
                self.sample_time = param.value
                self.get_logger().info(f"Sample time updated: {self.sample_time}")
                self.timer.cancel()
                self.timer = self.create_timer(self.sample_time, self.timer_callback)
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Controller Node shutting down gracefully.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()