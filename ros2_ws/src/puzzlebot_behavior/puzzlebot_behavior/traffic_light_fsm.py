#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from rcl_interfaces.srv import SetParameters

from custom_interfaces.msg import ColorBlobDetection

from std_srvs.srv import SetBool

# FSM state constants
NO_TRAFFIC_LIGHT = 0
GREEN_TRAFFIC_LIGHT = 1
YELLOW_TRAFFIC_LIGHT = 2
RED_TRAFFIC_LIGHT = 3

class TrafficLightFSM(Node):
    def __init__(self):
        super().__init__('traffic_light_fsm')

        # Declare parameters
        self.declare_parameter('update_rate', 100.0)    # Hz
        self.declare_parameter('color_detection_timeout', 0.5)  # seconds
        self.declare_parameter('green_velocity_scale', 1.0)    # scale factor
        self.declare_parameter('yellow_velocity_scale', 0.6)   # scale factor

        # Retrieve parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.color_detection_timeout = self.get_parameter('color_detection_timeout').value
        self.green_velocity_scale = self.get_parameter('green_velocity_scale').value
        self.yellow_velocity_scale = self.get_parameter('yellow_velocity_scale').value

        # Timer for the FSM loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
        
        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate',   Parameter.Type.DOUBLE, self.update_rate),
            Parameter('color_detection_timeout', Parameter.Type.DOUBLE, self.color_detection_timeout),
            Parameter('green_velocity_scale', Parameter.Type.DOUBLE, self.green_velocity_scale),
            Parameter('yellow_velocity_scale', Parameter.Type.DOUBLE, self.yellow_velocity_scale)
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")

        # State register
        self.state = NO_TRAFFIC_LIGHT
        self.previous_state = NO_TRAFFIC_LIGHT

        # Last detection
        self.detection = ColorBlobDetection()
        self.last_color = ColorBlobDetection.COLOR_NONE
        self.last_valid_color = ColorBlobDetection.COLOR_NONE
        self.new_detection = False
        self.color_lost_time = None

        # Time tracking
        self.now_time = None
        self.last_time = None

        # Flag for tracking PID controller status
        self.pid_stopped = True  # Assume PID is initially stopped

        # Subscriber for color blob detection
        self.create_subscription(
            ColorBlobDetection,
            'puzzlebot_real/color_blob_detection',
            self.color_blob_callback,
            10
        )
        
        # Create a client for stopping/resuming the PID controller via a service
        self.pid_stop_client = self.create_client(SetBool, 'puzzlebot_real/point_PID/PID_stop')
        while not self.pid_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/puzzlebot_real/point_PID/PID_stop service not available, waiting...")
        
        # Create a parameter client to update the Kp_V parameter
        self.parameter_client = self.create_client(SetParameters, 'pid_point_controller/set_parameters')
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /pid_point_controller/set_parameters service...')

        self.process_state_actions()
        self.get_logger().info('TrafficLightFSM Start.')

    def color_blob_callback(self, msg):
        # Store the detection and set a flag
        self.detection = msg
        self.new_detection = True

    def call_pid_stop_service(self, stop):
        # Prepare a service request for stopping (if stop==True) or resuming (if stop==False) the PID controller
        req = SetBool.Request()
        req.data = stop
        future = self.pid_stop_client.call_async(req)
        # Use asynchronous callback so that the timer thread is not blocked
        future.add_done_callback(lambda fut: self.pid_stop_response_callback(fut, stop))    

    def pid_stop_response_callback(self, future, stop):
        try:
            result = future.result()
            self.pid_stopped = stop
            self.get_logger().info(f"PID stop service call successful: {'stopping' if stop else 'resuming'} PID.")
        except Exception as e:
            self.get_logger().error("PID stop service call failed: " + str(e))
        
    def update_pid_parameter(self, param_name, param_value):
        """
        Updates a single parameter in the PID controller
        
        Args:
            param_name (str): Name of the parameter to update
            param_value: Value to set the parameter to (type will be inferred)
        """
        req = SetParameters.Request()
        
        # Determine parameter type based on value
        if isinstance(param_value, bool):
            param_type = Parameter.Type.BOOL
        elif isinstance(param_value, (int, float)):
            param_type = Parameter.Type.DOUBLE
        elif isinstance(param_value, str):
            param_type = Parameter.Type.STRING
        else:
            self.get_logger().error(f"Unsupported parameter type for {param_name}: {type(param_value)}.")
            return
        
        # Create parameter message    
        req.parameters = [
            Parameter(param_name, param_type, param_value).to_parameter_msg()
        ]
        
        # Call the service asynchronously
        future = self.parameter_client.call_async(req)
        future.add_done_callback(
            lambda fut: self.pid_parameter_response_callback(fut, param_name, param_value)
        )

    def pid_parameter_response_callback(self, future, param_name, param_value):
        """
        Callback for parameter update service response
        """
        try:
            result = future.result()
            if result is not None and any(result.results):
                if result.results[0].successful:
                    self.get_logger().info(f"Updated {param_name} to {param_value}.")
                else:
                    self.get_logger().error(f"Failed to update {param_name}: {result.results[0].reason}")
            else:
                self.get_logger().error(f"Failed to update {param_name} to {param_value}.")
        except Exception as e:
            self.get_logger().error(f"Error updating {param_name} to {param_value}: {str(e)}.")

    def fsm_update_loop(self):
        # Get current time
        self.now_time = self.get_clock().now().nanoseconds * 1e-9  # seconds

        # Initialization on first run
        if self.last_time is None:
            self.last_time = self.now_time
            return
            
        # Calculate the elapsed time since the last update
        dt = self.now_time - self.last_time
        if dt < 1.0 / self.update_rate:
            return   
        self.last_time = self.now_time

        # Check for new detection and transition if the color changed
        if self.new_detection:
            color = self.detection.color
            
            # Store the last valid color if this is a valid color
            if color != ColorBlobDetection.COLOR_NONE:
                self.last_valid_color = color
                self.color_lost_time = None
            elif self.last_valid_color != ColorBlobDetection.COLOR_NONE:
                # If we lost color detection but have a valid previous color
                if self.color_lost_time is None:
                    self.color_lost_time = self.now_time
                elif self.now_time - self.color_lost_time > self.color_detection_timeout:
                    # If timeout exceeded, actually consider it as no color
                    self.last_valid_color = ColorBlobDetection.COLOR_NONE
                else:
                    # During timeout, use the last valid color
                    color = self.last_valid_color
            
            # If the color changed, process state transition
            if color != self.last_color:
                self.previous_state = self.state
                self.last_color = color
                self.process_state_transitions(color)
            
            self.new_detection = False

    def process_state_actions(self):
        if (self.state == NO_TRAFFIC_LIGHT or self.state == RED_TRAFFIC_LIGHT):
            # Stop
            self.call_pid_stop_service(True)
            self.get_logger().info("Stopping PID controller.")
            
        elif self.state == GREEN_TRAFFIC_LIGHT:
            self.call_pid_stop_service(False)
            self.get_logger().info("Resuming PID controller.")
            
            # Set velocity scaling for GREEN
            self.update_pid_parameter("velocity_scale_factor", self.green_velocity_scale)

        elif self.state == YELLOW_TRAFFIC_LIGHT:
            self.call_pid_stop_service(False)
            self.get_logger().info("Resuming PID controller.")
            
            # Set velocity scaling for YELLOW
            self.update_pid_parameter("velocity_scale_factor", self.yellow_velocity_scale)

    def process_state_transitions(self, color):
        next_state = self.state
        
        if self.state == NO_TRAFFIC_LIGHT and self.pid_stopped:
            if color == ColorBlobDetection.COLOR_GREEN: next_state = GREEN_TRAFFIC_LIGHT

        elif self.state == GREEN_TRAFFIC_LIGHT and not self.pid_stopped:
            if color == ColorBlobDetection.COLOR_YELLOW: next_state = YELLOW_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_RED: next_state = RED_TRAFFIC_LIGHT

        elif self.state == YELLOW_TRAFFIC_LIGHT and not self.pid_stopped:
            if color == ColorBlobDetection.COLOR_RED: next_state = RED_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_GREEN: next_state = GREEN_TRAFFIC_LIGHT

        elif self.state == RED_TRAFFIC_LIGHT and self.pid_stopped:
            if color == ColorBlobDetection.COLOR_GREEN: next_state = GREEN_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_YELLOW: next_state = YELLOW_TRAFFIC_LIGHT

        # Apply state transition if needed
        if next_state != self.state:
            self.get_logger().info(f'{self._state_name(self.state)} → {self._state_name(next_state)}.')
            self.state = next_state
                        
            # Process current state actions
            self.process_state_actions()

    def _state_name(self, s):
        return {
            NO_TRAFFIC_LIGHT: 'NO_TRAFFIC_LIGHT',
            GREEN_TRAFFIC_LIGHT: 'GREEN_TRAFFIC_LIGHT',
            YELLOW_TRAFFIC_LIGHT: 'YELLOW_TRAFFIC_LIGHT',
            RED_TRAFFIC_LIGHT: 'RED_TRAFFIC_LIGHT'
        }[s]
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="update_rate must be > 0."
                    )
                self.update_rate = float(param.value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
                self.get_logger().info(f"Update rate changed: {self.update_rate} Hz.")
                
            elif param.name == 'color_detection_timeout':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="color_detection_timeout must be a non-negative number."
                    )
                self.color_detection_timeout = float(param.value)
                self.get_logger().info(f"Color detection timeout updated: {self.color_detection_timeout}.")
                
            elif param.name == 'green_velocity_scale':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="green_velocity_scale must be a non-negative number."
                    )
                self.green_velocity_scale = float(param.value)
                self.get_logger().info(f"Green velocity scale updated: {self.green_velocity_scale}.")
                
            elif param.name == 'yellow_velocity_scale':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="yellow_velocity_scale must be a non-negative number."
                    )
                self.yellow_velocity_scale = float(param.value)
                self.get_logger().info(f"Yellow velocity scale updated: {self.yellow_velocity_scale}.")
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrafficLightFSM()
    except Exception as e:
        print(f"[FATAL] TrafficLightFSM failed to initialize: {e}.", file=sys.stderr)
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