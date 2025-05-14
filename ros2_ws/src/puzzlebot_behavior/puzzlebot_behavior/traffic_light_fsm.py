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
        self.declare_parameter('green_proportional', 0.17)     # m/s
        self.declare_parameter('yellow_proportional', 0.09)    # m/s

        # Retrieve parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.green_proportional = self.get_parameter('green_proportional').value
        self.yellow_proportional = self.get_parameter('yellow_proportional').value

        # Timer for the FSM loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
        
        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate',   Parameter.Type.DOUBLE, self.update_rate),
            Parameter('green_proportional',   Parameter.Type.DOUBLE, self.green_proportional),
            Parameter('yellow_proportional',  Parameter.Type.DOUBLE, self.yellow_proportional)
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
        self.new_detection = False

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

    def update_pid_proportional(self, kp_v):
        req = SetParameters.Request()
        req.parameters = [
            Parameter("Kp_V", Parameter.Type.DOUBLE, kp_v).to_parameter_msg()
        ]
        future = self.parameter_client.call_async(req)
        future.add_done_callback(lambda fut: self.pid_proportional_response_callback(fut, kp_v))
        
    def pid_proportional_response_callback(self, future, kp_v):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f"Updated Kp_V to {kp_v}.")
            else:
                self.get_logger().error(f"Failed to update Kp_V to {kp_v}.")
        except Exception as e:
            self.get_logger().error(f"Error updating Kp_V to {kp_v}: {str(e)}.")

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
            if self.new_detection and self.detection.color != self.last_color:
                self.previous_state = self.state
                self.process_state_transitions()
                self.last_color = self.detection.color
                self.new_detection = False

    def process_state_actions(self):
        if (self.state == NO_TRAFFIC_LIGHT or self.state == RED_TRAFFIC_LIGHT):
            # Stop
            self.call_pid_stop_service(True)
            self.get_logger().info("Stopping PID controller.")
            
        elif self.state == GREEN_TRAFFIC_LIGHT:
            self.call_pid_stop_service(False)
            self.get_logger().info("Resuming PID controller.")
            self.update_pid_proportional(self.green_proportional)

        elif self.state == YELLOW_TRAFFIC_LIGHT:
            self.call_pid_stop_service(False)
            self.get_logger().info("Resuming PID controller.")
            self.update_pid_proportional(self.yellow_proportional)

    def process_state_transitions(self):
        color = self.detection.color
        next_state = self.state
        
        if self.state == NO_TRAFFIC_LIGHT and self.pid_stopped:
            if color == ColorBlobDetection.COLOR_GREEN: next_state = GREEN_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_YELLOW: next_state = YELLOW_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_RED: next_state = RED_TRAFFIC_LIGHT

        elif self.state == GREEN_TRAFFIC_LIGHT and not self.pid_stopped:
            if color == ColorBlobDetection.COLOR_YELLOW: next_state = YELLOW_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT

        elif self.state == YELLOW_TRAFFIC_LIGHT and not self.pid_stopped:
            if color == ColorBlobDetection.COLOR_RED: next_state = RED_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT

        elif self.state == RED_TRAFFIC_LIGHT and self.pid_stopped:
            if color == ColorBlobDetection.COLOR_GREEN: next_state = GREEN_TRAFFIC_LIGHT
            elif color == ColorBlobDetection.COLOR_NONE: next_state = NO_TRAFFIC_LIGHT

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

            elif param.name == 'green_proportional':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="green_proportional must be a non-negative number."
                    )
                self.green_proportional = float(param.value)
                self.get_logger().info(f"Green Kp_V updated: {self.green_proportional}.")

            elif param.name == 'yellow_proportional':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="yellow_proportional must be a non-negative number."
                    )
                self.yellow_proportional = float(param.value)
                self.get_logger().info(f"Yellow Kp_V updated: {self.yellow_proportional}.")
        
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