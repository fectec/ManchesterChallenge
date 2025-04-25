#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

class ControllerNode(Node):
    """
    Implements a PID controller node that regulates motor input based on a setpoint and plant output.
    """
    def __init__(self):
        super().__init__("controller")
        
        # Declare and retrieve PID parameters
        self.declare_parameter("sample_time", 1.0)
        self.declare_parameter("Kp", 1.0)
        self.declare_parameter("Kd", 1.0)
        self.declare_parameter("Ki", 1.0)
        
        self.sample_time = self.get_parameter("sample_time").value
        self.kp = self.get_parameter("Kp").value
        self.kd = self.get_parameter("Kd").value
        self.ki = self.get_parameter("Ki").value
        
        # Internal controller state
        self.motor_output = 0.0
        self.setpoint = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        
        # I/O topics
        self.motor_output_sub = self.create_subscription(
            Float32, "motor_output_y", self.motor_output_callback, 10
        )
        self.setpoint_sub = self.create_subscription(
            Float32, "setpoint", self.setpoint_callback, 10
        )
        self.motor_input_pub = self.create_publisher(Float32, "motor_input_u", 10)
        self.error_pub = self.create_publisher(Float32, "error", 10)
        self.timer = self.create_timer(self.sample_time, self.timer_callback)
        
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Setup service clients for external nodes to enable
        self.simulation_running = False
        self.dynamic_system_cli = self.create_client(SetProcessBool, 'EnableDynamicSystemNode')
        while not self.dynamic_system_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EnableDynamicSystemNode not available...')

        self.setpoint_enabled = False
        self.setpoint_cli = self.create_client(SetProcessBool, 'EnableSetpointNode')
        while not self.setpoint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EnableSetpointNode not available...')

        self.get_logger().info("Controller Node Started.")
        self.enable_dynamic_system_node(True)
    
    def timer_callback(self):
        if not self.simulation_running:
            return
        
        error = self.setpoint - self.motor_output

        # PID formula components
        p_term = self.kp * error
        self.integral += error * self.sample_time 
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error) / self.sample_time 
        control_output = p_term + i_term + d_term
        self.prev_error = error 

        self.motor_input_pub.publish(Float32(data=control_output))
        self.error_pub.publish(Float32(data=error))

    def motor_output_callback(self, msg):
        self.motor_output = msg.data
    
    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    def enable_dynamic_system_node(self, enable: bool):
        request = SetProcessBool.Request()
        request.enable = enable
        
        future = self.dynamic_system_cli.call_async(request)
        future.add_done_callback(self.enable_dynamic_system_node_callback)

    def enable_setpoint_node(self, enable: bool):
        if enable and not self.setpoint_enabled:
            request = SetProcessBool.Request()
            request.enable = enable
            
            future = self.setpoint_cli.call_async(request)
            future.add_done_callback(self.enable_setpoint_node_callback)
        else:
            self.get_logger().info("Setpoint Node is already enabled, skipping service call.")
        
    def enable_dynamic_system_node_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.simulation_running = True
                self.get_logger().info(f'Successful EnableDynamicSystemNode service: {response.message} / Controller Simulation Started.')
                self.enable_setpoint_node(True)
            else:
                self.simulation_running = False
                self.get_logger().warn(f'Failed EnableDynamicSystemNode service: {response.message}.')
        except Exception as e:
            self.simulation_running = False
            self.get_logger().error(f'Failed EnableDynamicSystemNode service call: {e}.')

    def enable_setpoint_node_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.setpoint_enabled = True  
                self.get_logger().info(f'Successful EnableSetpointNode service: {response.message}.')
            else:
                self.get_logger().warn(f'Failed EnableSetpointNode service: {response.message}.')
        except Exception as e:
            self.get_logger().error(f'Failed EnableSetpointNode service call: {e}.')

    def parameter_callback(self, params):
        # Update parameters dynamically at runtime
        for param in params:
            if param.name == "Kp":
                self.kp = param.value
                self.get_logger().info(f"Kp updated: {self.kp}.")
            elif param.name == "Ki":
                self.ki = param.value
                self.get_logger().info(f"Ki updated: {self.ki}.")
            elif param.name == "Kd":
                self.kd = param.value
                self.get_logger().info(f"Kd updated: {self.kd}.")
            elif param.name == "sample_time":
                if param.value <= 0.0:
                    self.get_logger().warn("Invalid sample_time! It must be > 0.")
                    return SetParametersResult(successful=False, reason="Sample time must be greater than 0.")
                
                self.sample_time = param.value
                self.get_logger().info(f"Sample time updated: {self.sample_time}.")
                self.timer.cancel()
                self.timer = self.create_timer(self.sample_time, self.timer_callback)
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
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