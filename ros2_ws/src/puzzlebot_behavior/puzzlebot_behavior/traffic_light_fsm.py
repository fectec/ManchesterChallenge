#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy import qos
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
    """ 
    Implements a finite state machine (FSM) for traffic light-based robot behavior control.
    
    It subscribes to color blob detection messages and transitions between NO_TRAFFIC_LIGHT, GREEN, YELLOW, and RED states
    based on the detected traffic light color. The node controls a controller via services to pause or resume movement,
    and dynamically adjusts velocity scaling parameters according to the active light state.
    """

    def __init__(self):
        super().__init__('traffic_light_fsm')

        # Declare parameters
        self.declare_parameter('update_rate', 30.0)               # Hz

        self.declare_parameter('color_detection_timeout', 2.0)    # s

        self.declare_parameter('green_velocity_scale', 1.0)     
        self.declare_parameter('yellow_velocity_scale', 0.6)  

        self.declare_parameter('controller_on_service', 'pid_point_controller/controller_on')
        self.declare_parameter('controller_parameter_service', 'pid_point_controller/set_parameters')

        # Retrieve parameters
        self.update_rate = self.get_parameter('update_rate').value
        
        self.color_detection_timeout = self.get_parameter('color_detection_timeout').value
        
        self.green_velocity_scale = self.get_parameter('green_velocity_scale').value
        self.yellow_velocity_scale = self.get_parameter('yellow_velocity_scale').value
        
        self.controller_on_service = self.get_parameter('controller_on_service').value
        self.controller_parameter_service = self.get_parameter('controller_parameter_service').value

        # Timer for the FSM loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
        
        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate',                Parameter.Type.DOUBLE,  self.update_rate),
            Parameter('color_detection_timeout',    Parameter.Type.DOUBLE,  self.color_detection_timeout),
            Parameter('green_velocity_scale',       Parameter.Type.DOUBLE,  self.green_velocity_scale),
            Parameter('yellow_velocity_scale',      Parameter.Type.DOUBLE,  self.yellow_velocity_scale),
            Parameter('controller_on_service',      Parameter.Type.STRING,  self.controller_on_service),
            Parameter('controller_parameter_service', Parameter.Type.STRING, self.controller_parameter_service)
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

        # Flag for tracking controller status
        self.controller_enabled = False

        # Subscriber for color blob detection
        self.create_subscription(
            ColorBlobDetection,
            'color_blob_detection',
            self.color_blob_callback,
            qos.qos_profile_sensor_data
        )
        
        # Create a client for enabling or disabling the controller via a service
        self.controller_on_client = self.create_client(SetBool, self.controller_on_service)
        while not self.controller_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{self.controller_on_service} service not available, waiting...")
        
        # Create a parameter client to update the controller parameters
        self.controller_parameter_client = self.create_client(SetParameters, self.controller_parameter_service)
        while not self.controller_parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.controller_parameter_service} service not available, waiting...')
        
        self.get_logger().info('TrafficLightFSM Start.')
        self.process_state_actions()

    def color_blob_callback(self, msg: ColorBlobDetection) -> None:
        """Receives color blob detection data and marks it as a new detection."""
        self.detection = msg
        self.new_detection = True

    def call_controller_on_service(self, enable: bool) -> None:
        """Sends a request to enable or disable the controller."""
        req = SetBool.Request()
        req.data = enable
        future = self.controller_on_client.call_async(req)
        future.add_done_callback(lambda fut: self.controller_on_response_callback(fut, enable))    

    def controller_on_response_callback(self, future, enable: bool) -> None:
        """Processes the result of the controller enable/disable service call."""
        try:
            result = future.result()
            self.controller_enabled = enable
            action_str = 'enabling' if enable else 'disabling'
            self.get_logger().info(f"Controller service call successful: {action_str} controller.")
        except Exception as e:
            self.get_logger().error("Controller service call failed: " + str(e))
        
    def update_controller_parameter(self, param_name: str, param_value) -> None:
        """Requests an update to a parameter of the controller."""
        req = SetParameters.Request()
        
        if isinstance(param_value, bool):
            param_type = Parameter.Type.BOOL
        elif isinstance(param_value, (int, float)):
            param_type = Parameter.Type.DOUBLE
        elif isinstance(param_value, str):
            param_type = Parameter.Type.STRING
        else:
            self.get_logger().error(f"Unsupported parameter type for {param_name}: {type(param_value)}.")
            return
        
        req.parameters = [
            Parameter(param_name, param_type, param_value).to_parameter_msg()
        ]
        
        future = self.controller_parameter_client.call_async(req)
        future.add_done_callback(
            lambda fut: self.controller_parameter_response_callback(fut, param_name, param_value)
        )

    def controller_parameter_response_callback(self, future, param_name: str, param_value) -> None:
        """Processes the response from the controller parameter update service."""
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

    def fsm_update_loop(self) -> None:
        """Main FSM loop that checks detection and transitions between states."""
        # Get current time
        now_time = self.get_clock().now().nanoseconds * 1e-9  # s

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
                    self.color_lost_time = now_time
                elif now_time - self.color_lost_time > self.color_detection_timeout:
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

    def process_state_actions(self) -> None:
        """Performs actions based on the current FSM state."""
        if (self.state == NO_TRAFFIC_LIGHT or self.state == RED_TRAFFIC_LIGHT):
            self.call_controller_on_service(False)  # Disable controller
            
        elif self.state == GREEN_TRAFFIC_LIGHT:
            self.call_controller_on_service(True)   # Enable controller

            # Set velocity scaling for GREEN
            self.update_controller_parameter("velocity_scale_factor", self.green_velocity_scale)

        elif self.state == YELLOW_TRAFFIC_LIGHT:
            self.call_controller_on_service(True)   # Enable controller
            
            # Set velocity scaling for YELLOW
            self.update_controller_parameter("velocity_scale_factor", self.yellow_velocity_scale)

    def process_state_transitions(self, color: int) -> None:
        """Handles transitions between FSM states based on detected color."""
        next_state = self.state
        
        if self.state == NO_TRAFFIC_LIGHT and not self.controller_enabled:
            if color == ColorBlobDetection.COLOR_GREEN: next_state = GREEN_TRAFFIC_LIGHT

        elif self.state == GREEN_TRAFFIC_LIGHT and self.controller_enabled:
            if color == ColorBlobDetection.COLOR_YELLOW: next_state = YELLOW_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT

        elif self.state == YELLOW_TRAFFIC_LIGHT and self.controller_enabled:
            if color == ColorBlobDetection.COLOR_RED: next_state = RED_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT

        elif self.state == RED_TRAFFIC_LIGHT and not self.controller_enabled:
            if color == ColorBlobDetection.COLOR_GREEN: next_state = GREEN_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT

        if next_state != self.state:
            self.get_logger().info(f'{self._state_name(self.state)} → {self._state_name(next_state)}.')
            self.state = next_state
                        
            self.process_state_actions()

    def _state_name(self, s: int) -> str:
        """Returns the string representation of a given FSM state."""
        return {
            NO_TRAFFIC_LIGHT: 'NO_TRAFFIC_LIGHT',
            GREEN_TRAFFIC_LIGHT: 'GREEN_TRAFFIC_LIGHT',
            YELLOW_TRAFFIC_LIGHT: 'YELLOW_TRAFFIC_LIGHT',
            RED_TRAFFIC_LIGHT: 'RED_TRAFFIC_LIGHT'
        }[s]
    
    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validates and applies updated node parameters."""
        new_green_scale = self.green_velocity_scale
        new_yellow_scale = self.yellow_velocity_scale

        for param in params:
            if param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="update_rate must be > 0."
                    )

            elif param.name == 'color_detection_timeout':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="color_detection_timeout must be a non-negative number."
                    )

            elif param.name == 'green_velocity_scale':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="green_velocity_scale must be a number."
                    )
                new_green_scale = float(param.value)

            elif param.name == 'yellow_velocity_scale':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="yellow_velocity_scale must be a number."
                    )
                new_yellow_scale = float(param.value)

            elif param.name == 'controller_on_service':
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(
                        successful=False,
                        reason="controller_on_service must be a non-empty string."
                    )

            elif param.name == 'controller_parameter_service':
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(
                        successful=False,
                        reason="controller_parameter_service must be a non-empty string."
                    )

        # Validate green and yellow velocity scales
        if not (0.0 < new_green_scale <= 1.0):
            return SetParametersResult(
                successful=False,
                reason="green_velocity_scale must be > 0.0 and ≤ 1.0."
            )

        if not (0.0 < new_yellow_scale <= 1.0):
            return SetParametersResult(
                successful=False,
                reason="yellow_velocity_scale must be > 0.0 and ≤ 1.0."
            )

        # Ensure yellow is less than green
        if not (new_yellow_scale < new_green_scale):
            return SetParametersResult(
                successful=False,
                reason="yellow_velocity_scale must be strictly less than green_velocity_scale."
            )

        # Apply parameters if all are valid
        for param in params:
            if param.name == 'update_rate':
                self.update_rate = float(param.value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

            elif param.name == 'color_detection_timeout':
                self.color_detection_timeout = float(param.value)
                self.get_logger().info(f"color_detection_timeout updated: {self.color_detection_timeout} s.")

            elif param.name == 'green_velocity_scale':
                self.green_velocity_scale = new_green_scale
                self.get_logger().info(f"green_velocity_scale updated: {self.green_velocity_scale}.")

            elif param.name == 'yellow_velocity_scale':
                self.yellow_velocity_scale = new_yellow_scale
                self.get_logger().info(f"yellow_velocity_scale updated: {self.yellow_velocity_scale}.")

            elif param.name == 'controller_on_service':
                self.controller_on_service = param.value
                self.get_logger().info(f"controller_on_service updated: {self.controller_on_service}.")

            elif param.name == 'controller_parameter_service':
                self.controller_parameter_service = param.value
                self.get_logger().info(f"controller_parameter_service updated: {self.controller_parameter_service}.")

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