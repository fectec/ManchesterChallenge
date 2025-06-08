#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from rcl_interfaces.srv import SetParameters

from custom_interfaces.msg import ColorBlobDetection
from yolov8_msgs.msg import Yolov8Inference

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from std_srvs.srv import SetBool

# Sign type constants
SIGN_NONE = 0
SIGN_AHEAD_ONLY = 1
SIGN_TURN_RIGHT_AHEAD = 2
SIGN_TURN_LEFT_AHEAD = 3
SIGN_ROADWORK_AHEAD = 4
SIGN_STOP = 5
SIGN_GIVE_WAY = 6

# FSM Actions
ACTION_FULL_SPEED = 0
ACTION_MID_SPEED = 1
ACTION_ZERO_SPEED = 2
ACTION_STRAIGHT_INTERSECTION = 3
ACTION_RIGHT_INTERSECTION = 4
ACTION_LEFT_INTERSECTION = 5

class TrafficFSM(Node):
    """ 
    Implements a finite state machine (FSM) for traffic light and sign-based robot behavior control.
    
    The FSM follows a simple state table based on three variables:
    - current_light: {none, green, yellow, red}
    - current_sign: {none, ahead_only, turn_right_ahead, turn_left_ahead, roadwork_ahead, stop, give_way}
    - line_detected: boolean
    
    Key rules:
    1. STOP sign always results in zero_speed
    2. RED light always results in zero_speed
    3. Intersections (no line + directional sign) require GREEN light to cross
    4. LAST_GIVEWAY modifier affects directional signs after seeing a give way sign
    5. During intersection crossing, all detections are ignored until line is found again
    """

    def __init__(self):
        super().__init__('traffic_fsm')

        # Declare parameters
        self.declare_parameter('update_rate', 50.0)             # Hz
        self.declare_parameter('detection_timeout', 0.8)        # s
        self.declare_parameter('full_speed_scale', 1.0)         
        self.declare_parameter('mid_speed_scale', 0.6)          
        self.declare_parameter('pid_toggle_service', 'point_pid/pid_toggle')
        self.declare_parameter('pid_parameter_service', 'pid_point_controller/set_parameters')
        self.declare_parameter('stop_sign_min_area', 0.1)       
        self.declare_parameter('intersection_linear_vel', 0.2)   # m/s
        self.declare_parameter('intersection_angular_vel', 0.5)  # rad/s
        self.declare_parameter('image_width', 640)               # pixels
        self.declare_parameter('image_height', 480)              # pixels
        
        # Sign class name parameters
        self.declare_parameter('sign_ahead_class', 'fwd')
        self.declare_parameter('sign_right_class', 'right')
        self.declare_parameter('sign_left_class', 'left')
        self.declare_parameter('sign_roadwork_class', 'triUp')
        self.declare_parameter('sign_stop_class', 'stop')
        self.declare_parameter('sign_giveway_class', 'triDwn')

        # Retrieve parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.full_speed_scale = self.get_parameter('full_speed_scale').value
        self.mid_speed_scale = self.get_parameter('mid_speed_scale').value
        self.pid_toggle_service = self.get_parameter('pid_toggle_service').value
        self.pid_parameter_service = self.get_parameter('pid_parameter_service').value
        self.stop_sign_min_area = self.get_parameter('stop_sign_min_area').value
        self.intersection_linear_vel = self.get_parameter('intersection_linear_vel').value
        self.intersection_angular_vel = self.get_parameter('intersection_angular_vel').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # Sign class names
        self.sign_classes = {
            self.get_parameter('sign_ahead_class').value: SIGN_AHEAD_ONLY,
            self.get_parameter('sign_right_class').value: SIGN_TURN_RIGHT_AHEAD,
            self.get_parameter('sign_left_class').value: SIGN_TURN_LEFT_AHEAD,
            self.get_parameter('sign_roadwork_class').value: SIGN_ROADWORK_AHEAD,
            self.get_parameter('sign_stop_class').value: SIGN_STOP,
            self.get_parameter('sign_giveway_class').value: SIGN_GIVE_WAY
        }

        # Timer for the FSM loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
        
        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate',                Parameter.Type.DOUBLE,  self.update_rate),
            Parameter('detection_timeout',          Parameter.Type.DOUBLE,  self.detection_timeout),
            Parameter('full_speed_scale',           Parameter.Type.DOUBLE,  self.full_speed_scale),
            Parameter('mid_speed_scale',            Parameter.Type.DOUBLE,  self.mid_speed_scale),
            Parameter('pid_toggle_service',         Parameter.Type.STRING,  self.pid_toggle_service),
            Parameter('pid_parameter_service',      Parameter.Type.STRING,  self.pid_parameter_service),
            Parameter('stop_sign_min_area',         Parameter.Type.DOUBLE,  self.stop_sign_min_area),
            Parameter('intersection_linear_vel',    Parameter.Type.DOUBLE,  self.intersection_linear_vel),
            Parameter('intersection_angular_vel',   Parameter.Type.DOUBLE,  self.intersection_angular_vel),
            Parameter('image_width',                Parameter.Type.INTEGER, self.image_width),
            Parameter('image_height',               Parameter.Type.INTEGER, self.image_height)
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")

        # Current state variables
        self.current_light = ColorBlobDetection.COLOR_NONE
        self.current_sign = SIGN_NONE
        self.last_sign = SIGN_NONE 
        self.line_detected = True
        
        # State for intersection crossing
        self.crossing_intersection = False
        self.intersection_direction = SIGN_NONE
        
        # Detection tracking
        self.light_lost_time = None
        self.sign_lost_time = None
        self.new_light_detection = False
        self.new_sign_detection = False
        self.new_line_detection = False
        
        # Detection messages
        self.light_detection = ColorBlobDetection()
        self.sign_detection = Yolov8Inference()

        # Time tracking
        self.now_time = None

        # Flag for tracking PID controller status
        self.pid_stopped = True
        self.current_velocity_scale = None  
        
        # Publisher for manual velocity commands during intersection crossing
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for color blob detection (traffic lights)
        self.create_subscription(
            ColorBlobDetection,
            'color_blob_detection',
            self.color_blob_callback,
            10
        )
        
        # Subscriber for YOLO inference (traffic signs)
        self.create_subscription(
            Yolov8Inference,
            'Yolov8_Inference',
            self.sign_detection_callback,
            10
        )
        
        # Subscriber for line detection
        self.create_subscription(
            Bool,
            'line_detected',
            self.line_detection_callback,
            10
        )
        
        # Create a client for stopping or resuming the PID controller via a service
        self.pid_toggle_client = self.create_client(SetBool, self.pid_toggle_service)
        while not self.pid_toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{self.pid_toggle_service} service not available, waiting...")
        
        # Create a parameter client to update the PID parameters
        self.pid_parameter_client = self.create_client(SetParameters, self.pid_parameter_service)
        while not self.pid_parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.pid_parameter_service} service not available, waiting...')
        
        self.get_logger().info('TrafficFSM Start.')
        self.execute_action(self.determine_action())

    def color_blob_callback(self, msg: ColorBlobDetection) -> None:
        """Receives color blob detection data and marks it as a new detection."""
        self.light_detection = msg
        self.new_light_detection = True

    def sign_detection_callback(self, msg: Yolov8Inference) -> None:
        """Receives YOLO inference data and marks it as a new detection."""
        self.sign_detection = msg
        self.new_sign_detection = True
        
    def line_detection_callback(self, msg: Bool) -> None:
        """Receives line detection status."""
        self.line_detected = msg.data
        self.new_line_detection = True

    def classify_sign(self, class_name: str) -> int:
        """Maps YOLO class names to sign type constants."""
        return self.sign_classes.get(class_name, SIGN_NONE)

    def call_pid_toggle_service(self, stop: bool) -> None:
        """Sends a request to stop or resume the PID controller."""
        # Only call service if state is changing
        if self.pid_stopped == stop:
            return
            
        req = SetBool.Request()
        req.data = stop
        future = self.pid_toggle_client.call_async(req)
        future.add_done_callback(lambda fut: self.pid_toggle_response_callback(fut, stop))    

    def pid_toggle_response_callback(self, future, stop: bool) -> None:
        """Processes the result of the stop or resume PID service call."""
        try:
            result = future.result()
            self.pid_stopped = stop
            self.get_logger().info(f"PID toggle service call successful: {'stopping' if stop else 'resuming'} PID.")
        except Exception as e:
            self.get_logger().error("PID toggle service call failed: " + str(e))
        
    def update_pid_parameter(self, param_name: str, param_value) -> None:
        """Requests an update to a parameter of the PID controller."""
        req = SetParameters.Request()
        
        param_type = Parameter.Type.DOUBLE
        req.parameters = [
            Parameter(param_name, param_type, param_value).to_parameter_msg()
        ]
        
        future = self.pid_parameter_client.call_async(req)
        future.add_done_callback(
            lambda fut: self.pid_parameter_response_callback(fut, param_name, param_value)
        )

    def pid_parameter_response_callback(self, future, param_name: str, param_value) -> None:
        """Processes the response from the PID parameter update service."""
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

    def publish_velocity_command(self, linear_x: float, angular_z: float) -> None:
        """Publishes velocity command for manual control during intersections."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def fsm_update_loop(self) -> None:
        """Main FSM loop that checks detection and transitions between states."""
        # Get current time
        self.now_time = self.get_clock().now().nanoseconds * 1e-9  # s

        # Process traffic light detection
        if self.new_light_detection:
            color = self.light_detection.color
            
            if color != ColorBlobDetection.COLOR_NONE:
                self.current_light = color
                self.light_lost_time = None
            elif self.light_lost_time is None:
                self.light_lost_time = self.now_time
            elif self.now_time - self.light_lost_time > self.detection_timeout:
                self.current_light = ColorBlobDetection.COLOR_NONE
            
            self.new_light_detection = False

        # Process traffic sign detection
        if self.new_sign_detection:
            # Find sign with largest area (closest)
            best_sign = SIGN_NONE
            max_area = 0
            
            for inference in self.sign_detection.yolov8_inference:
                detected_sign = self.classify_sign(inference.class_name)
                if detected_sign != SIGN_NONE:
                    area = (inference.bottom - inference.top) * (inference.right - inference.left)
                    
                    # For stop sign, check if close enough
                    if detected_sign == SIGN_STOP:
                        normalized_area = area / (self.image_width * self.image_height)
                        if normalized_area < self.stop_sign_min_area:
                            continue 
                    
                    if area > max_area:
                        max_area = area
                        best_sign = detected_sign
            
            # Update current sign with timeout handling
            if best_sign != SIGN_NONE:
                # Track previous sign before updating
                if self.current_sign != SIGN_NONE and self.current_sign != best_sign:
                    self.last_sign = self.current_sign
                self.current_sign = best_sign
                self.sign_lost_time = None
            elif self.sign_lost_time is None:
                self.sign_lost_time = self.now_time
            elif self.now_time - self.sign_lost_time > self.detection_timeout:
                if self.current_sign != SIGN_NONE:
                    self.last_sign = self.current_sign
                self.current_sign = SIGN_NONE
            
            self.new_sign_detection = False

        # Handle intersection crossing state
        if self.crossing_intersection and self.line_detected:
            # We've found the line again after crossing
            self.crossing_intersection = False
            self.intersection_direction = SIGN_NONE
            self.get_logger().info("Line detected, intersection crossing complete.")

        # Determine and execute action
        action = self.determine_action()
        self.execute_action(action)

    def determine_action(self) -> int:
        """Determines action."""
        # STOP sign always results in zero_speed
        if self.current_sign == SIGN_STOP:
            return ACTION_ZERO_SPEED
        
        # RED light always results in zero_speed
        if self.current_light == ColorBlobDetection.COLOR_RED:
            return ACTION_ZERO_SPEED
        
        # Handle intersection crossing in progress
        if self.crossing_intersection:
            if self.intersection_direction == SIGN_AHEAD_ONLY:
                return ACTION_STRAIGHT_INTERSECTION
            elif self.intersection_direction == SIGN_TURN_RIGHT_AHEAD:
                return ACTION_RIGHT_INTERSECTION
            elif self.intersection_direction == SIGN_TURN_LEFT_AHEAD:
                return ACTION_LEFT_INTERSECTION
        
        # Check if we have LAST_GIVEWAY condition
        has_last_giveway = (self.last_sign == SIGN_GIVE_WAY and 
                           self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD])
        
        # NO LINE cases
        if not self.line_detected:
            if self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD]:
                if self.current_light == ColorBlobDetection.COLOR_GREEN:
                    # Start intersection crossing
                    self.crossing_intersection = True
                    self.intersection_direction = self.current_sign
                    self.get_logger().info(f"Starting intersection crossing: {self._sign_name(self.current_sign)}.")
                    
                    if self.current_sign == SIGN_AHEAD_ONLY:
                        return ACTION_STRAIGHT_INTERSECTION
                    elif self.current_sign == SIGN_TURN_RIGHT_AHEAD:
                        return ACTION_RIGHT_INTERSECTION
                    elif self.current_sign == SIGN_TURN_LEFT_AHEAD:
                        return ACTION_LEFT_INTERSECTION
            # All other NO_LINE cases result in zero_speed
            return ACTION_ZERO_SPEED
        
        # LINE DETECTED cases
        
        # ROADWORK or GIVE_WAY always result in mid_speed
        if self.current_sign in [SIGN_ROADWORK_AHEAD, SIGN_GIVE_WAY]:
            return ACTION_MID_SPEED
        
        # Directional signs with LAST_GIVEWAY modifier always mid_speed
        if has_last_giveway:
            return ACTION_MID_SPEED
        
        # Regular directional signs follow traffic light
        if self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD]:
            if self.current_light == ColorBlobDetection.COLOR_YELLOW:
                return ACTION_MID_SPEED
            else:  # GREEN or NONE
                return ACTION_FULL_SPEED
        
        # No sign - just follow traffic light
        if self.current_light == ColorBlobDetection.COLOR_YELLOW:
            return ACTION_MID_SPEED
        
        # Default: full_speed
        return ACTION_FULL_SPEED

    def execute_action(self, action: int) -> None:
        """Executes the determined action."""
        action_name = self._action_name(action)
        
        if action != getattr(self, 'last_action', None):
            # Log state information when action changes
            light_name = {
                ColorBlobDetection.COLOR_NONE: 'NONE',
                ColorBlobDetection.COLOR_GREEN: 'GREEN',
                ColorBlobDetection.COLOR_YELLOW: 'YELLOW',
                ColorBlobDetection.COLOR_RED: 'RED'
            }.get(self.current_light, 'UNKNOWN')
            
            sign_info = self._sign_name(self.current_sign)
            if (self.last_sign == SIGN_GIVE_WAY and 
                self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD]):
                sign_info += "_LAST_GIVEWAY"
            
            line_info = "LINE" if self.line_detected else "NO_LINE"
            
            self.get_logger().info(f"State: [{light_name}] [{sign_info}_{line_info}] -> Action: {action_name}.")
            self.last_action = action
        
        if action == ACTION_FULL_SPEED:
            self.call_pid_toggle_service(False)
            # Only update velocity scale if it changed
            if self.current_velocity_scale != self.full_speed_scale:
                self.update_pid_parameter("velocity_scale_factor", self.full_speed_scale)
                self.current_velocity_scale = self.full_speed_scale
            
        elif action == ACTION_MID_SPEED:
            self.call_pid_toggle_service(False)
            # Only update velocity scale if it changed
            if self.current_velocity_scale != self.mid_speed_scale:
                self.update_pid_parameter("velocity_scale_factor", self.mid_speed_scale)
                self.current_velocity_scale = self.mid_speed_scale
            
        elif action == ACTION_ZERO_SPEED:
            self.call_pid_toggle_service(True)
            self.publish_velocity_command(0.0, 0.0)
            # Reset velocity scale so it gets updated when resuming
            self.current_velocity_scale = None
            
        elif action == ACTION_STRAIGHT_INTERSECTION:
            self.call_pid_toggle_service(True)
            self.publish_velocity_command(self.intersection_linear_vel, 0.0)
            
        elif action == ACTION_RIGHT_INTERSECTION:
            self.call_pid_toggle_service(True)
            self.publish_velocity_command(self.intersection_linear_vel, -self.intersection_angular_vel)
            
        elif action == ACTION_LEFT_INTERSECTION:
            self.call_pid_toggle_service(True)
            self.publish_velocity_command(self.intersection_linear_vel, self.intersection_angular_vel)

    def _sign_name(self, sign: int) -> str:
        """Returns string representation of sign type."""
        return {
            SIGN_NONE: 'NONE',
            SIGN_AHEAD_ONLY: 'AHEAD_ONLY',
            SIGN_TURN_RIGHT_AHEAD: 'TURN_RIGHT',
            SIGN_TURN_LEFT_AHEAD: 'TURN_LEFT',
            SIGN_ROADWORK_AHEAD: 'ROADWORK',
            SIGN_STOP: 'STOP',
            SIGN_GIVE_WAY: 'GIVE_WAY'
        }.get(sign, 'UNKNOWN')

    def _action_name(self, action: int) -> str:
        """Returns string representation of action."""
        return {
            ACTION_FULL_SPEED: 'FULL_SPEED',
            ACTION_MID_SPEED: 'MID_SPEED',
            ACTION_ZERO_SPEED: 'ZERO_SPEED',
            ACTION_STRAIGHT_INTERSECTION: 'STRAIGHT_INTERSECTION',
            ACTION_RIGHT_INTERSECTION: 'RIGHT_INTERSECTION',
            ACTION_LEFT_INTERSECTION: 'LEFT_INTERSECTION'
        }.get(action, 'UNKNOWN')
    
    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validates and applies updated node parameters."""
        new_full_scale = self.full_speed_scale
        new_mid_scale = self.mid_speed_scale

        for param in params:
            if param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="update_rate must be > 0."
                    )

            elif param.name == 'detection_timeout':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="detection_timeout must be a non-negative number."
                    )

            elif param.name == 'full_speed_scale':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="full_speed_scale must be a number."
                    )
                new_full_scale = float(param.value)

            elif param.name == 'mid_speed_scale':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(
                        successful=False,
                        reason="mid_speed_scale must be a number."
                    )
                new_mid_scale = float(param.value)

            elif param.name == 'pid_toggle_service':
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(
                        successful=False,
                        reason="pid_toggle_service must be a non-empty string."
                    )

            elif param.name == 'pid_parameter_service':
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(
                        successful=False,
                        reason="pid_parameter_service must be a non-empty string."
                    )
                    
            elif param.name == 'stop_sign_min_area':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="stop_sign_min_area must be > 0."
                    )
                    
            elif param.name == 'intersection_linear_vel':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="intersection_linear_vel must be > 0."
                    )
                    
            elif param.name == 'intersection_angular_vel':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="intersection_angular_vel must be > 0."
                    )
                    
            elif param.name == 'image_width':
                if not isinstance(param.value, int) or param.value <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason="image_width must be a positive integer."
                    )
                    
            elif param.name == 'image_height':
                if not isinstance(param.value, int) or param.value <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason="image_height must be a positive integer."
                    )
                    
            elif param.name.startswith('sign_') and param.name.endswith('_class'):
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a non-empty string."
                    )

        # Validate velocity scales
        if not (0.0 < new_full_scale <= 1.0):
            return SetParametersResult(
                successful=False,
                reason="full_speed_scale must be > 0.0 and ≤ 1.0."
            )

        if not (0.0 < new_mid_scale <= 1.0):
            return SetParametersResult(
                successful=False,
                reason="mid_speed_scale must be > 0.0 and ≤ 1.0."
            )

        # Ensure mid is less than full
        if not (new_mid_scale < new_full_scale):
            return SetParametersResult(
                successful=False,
                reason="mid_speed_scale must be strictly less than full_speed_scale."
            )

        # Apply parameters if all are valid
        for param in params:
            if param.name == 'update_rate':
                self.update_rate = float(param.value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

            elif param.name == 'detection_timeout':
                self.detection_timeout = float(param.value)
                self.get_logger().info(f"detection_timeout updated: {self.detection_timeout} s.")

            elif param.name == 'full_speed_scale':
                self.full_speed_scale = new_full_scale
                self.get_logger().info(f"full_speed_scale updated: {self.full_speed_scale}.")

            elif param.name == 'mid_speed_scale':
                self.mid_speed_scale = new_mid_scale
                self.get_logger().info(f"mid_speed_scale updated: {self.mid_speed_scale}.")

            elif param.name == 'pid_toggle_service':
                self.pid_toggle_service = param.value
                self.get_logger().info(f"pid_toggle_service updated: {self.pid_toggle_service}.")

            elif param.name == 'pid_parameter_service':
                self.pid_parameter_service = param.value
                self.get_logger().info(f"pid_parameter_service updated: {self.pid_parameter_service}.")
                
            elif param.name == 'stop_sign_min_area':
                self.stop_sign_min_area = float(param.value)
                self.get_logger().info(f"stop_sign_min_area updated: {self.stop_sign_min_area}.")
                
            elif param.name == 'intersection_linear_vel':
                self.intersection_linear_vel = float(param.value)
                self.get_logger().info(f"intersection_linear_vel updated: {self.intersection_linear_vel} m/s.")
                
            elif param.name == 'intersection_angular_vel':
                self.intersection_angular_vel = float(param.value)
                self.get_logger().info(f"intersection_angular_vel updated: {self.intersection_angular_vel} rad/s.")
                
            elif param.name == 'image_width':
                self.image_width = int(param.value)
                self.get_logger().info(f"image_width updated: {self.image_width} pixels.")
                
            elif param.name == 'image_height':
                self.image_height = int(param.value)
                self.get_logger().info(f"image_height updated: {self.image_height} pixels.")
                
            elif param.name.startswith('sign_') and param.name.endswith('_class'):
                # Update sign class mapping
                old_value = None
                for class_name, sign_type in self.sign_classes.items():
                    if param.name == f'sign_{self._sign_param_name(sign_type)}_class':
                        old_value = class_name
                        break
                
                if old_value is not None:
                    sign_type = self.sign_classes.pop(old_value)
                    self.sign_classes[param.value] = sign_type
                    self.get_logger().info(f"{param.name} updated: {param.value}.")

        return SetParametersResult(successful=True)
    
    def _sign_param_name(self, sign_type: int) -> str:
        """Maps sign type to parameter name suffix."""
        return {
            SIGN_AHEAD_ONLY: 'ahead',
            SIGN_TURN_RIGHT_AHEAD: 'right',
            SIGN_TURN_LEFT_AHEAD: 'left',
            SIGN_ROADWORK_AHEAD: 'roadwork',
            SIGN_STOP: 'stop',
            SIGN_GIVE_WAY: 'giveway'
        }.get(sign_type, '')
    
def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrafficFSM()
    except Exception as e:
        print(f"[FATAL] TrafficFSM failed to initialize: {e}.", file=sys.stderr)
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