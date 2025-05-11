#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

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
        self.declare_parameter('update_rate', 100.0)  # Hz

        # Retrieve parameters
        self.update_rate = self.get_parameter('update_rate').value

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
            
        # Timer for the FSM loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
        
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
            
            # Process current state actions
            self.process_state_actions()
            
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

        elif self.state == YELLOW_TRAFFIC_LIGHT:
            self.call_pid_stop_service(False)
            self.get_logger().info("Resuming PID controller.")

    def process_state_transitions(self):
        color = self.detection.color
        next_state = self.state
        
        if self.state == NO_TRAFFIC_LIGHT and self.pid_stopped:
            if color == ColorBlobDetection.COLOR_GREEN: next_state = GREEN_TRAFFIC_LIGHT
            
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

    def _state_name(self, s):
        return {
            NO_TRAFFIC_LIGHT: 'NO_TRAFFIC_LIGHT',
            GREEN_TRAFFIC_LIGHT: 'GREEN_TRAFFIC_LIGHT',
            YELLOW_TRAFFIC_LIGHT: 'YELLOW_TRAFFIC_LIGHT',
            RED_TRAFFIC_LIGHT: 'RED_TRAFFIC_LIGHT'
        }[s]

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