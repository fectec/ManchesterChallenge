#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters

from geometry_msgs.msg import Twist
from yolov8_msgs.msg import Yolov8Inference
from custom_interfaces.msg import ColorBlobDetection
from std_srvs.srv import SetBool, Trigger

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
ACTION_SEARCHING_LINE = 6

# Inner Open-Loop FSM States
IDLE = 0
EXECUTING = 1

class TrafficFSM(Node):
    def __init__(self):
        super().__init__('traffic_fsm')
        
        # Declare parameters
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('detection_timeout', 1.0)
        
        # Speed scale parameters
        self.declare_parameter('full_speed_scale', 1.0)
        self.declare_parameter('mid_speed_scale', 0.6)
        
        # Controller service parameters
        self.declare_parameter('line_follow_controller_on_service', 'line_follow_controller/controller_on')
        self.declare_parameter('line_follow_controller_parameter_service', 'line_follow_controller/set_parameters')
        
        # Searching line parameter
        self.declare_parameter('searching_line_linear_speed', 0.09)
        
        # Turn intersection speed limits
        self.declare_parameter('min_turn_intersection_linear_speed', 0.08)
        self.declare_parameter('max_turn_intersection_linear_speed', 0.17)

        self.declare_parameter('min_turn_intersection_angular_speed', 0.5)
        self.declare_parameter('max_turn_intersection_angular_speed', 1.5)
        
        # Turn intersection speeds
        self.declare_parameter('turn_intersection_linear_speed', 0.09)
        self.declare_parameter('turn_intersection_angular_speed', 1.1)
        
        # Turn geometry parameters
        self.declare_parameter('turn_forward_distance', 0.35)
        self.declare_parameter('turn_rotation_angle', 1.6)
        
        # Stop sign area threshold parameter
        self.declare_parameter('stop_sign_area_threshold', 8000.0)  
        
        # Sign class parameters
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

        self.line_follow_controller_on_service = self.get_parameter('line_follow_controller_on_service').value
        self.line_follow_controller_parameter_service = self.get_parameter('line_follow_controller_parameter_service').value

        self.searching_line_linear_speed = self.get_parameter('searching_line_linear_speed').value

        self.min_turn_intersection_linear_speed = self.get_parameter('min_turn_intersection_linear_speed').value
        self.max_turn_intersection_linear_speed = self.get_parameter('max_turn_intersection_linear_speed').value

        self.min_turn_intersection_angular_speed = self.get_parameter('min_turn_intersection_angular_speed').value
        self.max_turn_intersection_angular_speed = self.get_parameter('max_turn_intersection_angular_speed').value

        self.turn_intersection_linear_speed = self.get_parameter('turn_intersection_linear_speed').value
        self.turn_intersection_angular_speed = self.get_parameter('turn_intersection_angular_speed').value
        
        self.turn_forward_distance = self.get_parameter('turn_forward_distance').value
        self.turn_rotation_angle = self.get_parameter('turn_rotation_angle').value
        
        self.stop_sign_area_threshold = self.get_parameter('stop_sign_area_threshold').value
        
        # Sign class mapping
        self.sign_classes = {
            self.get_parameter('sign_ahead_class').value: SIGN_AHEAD_ONLY,
            self.get_parameter('sign_right_class').value: SIGN_TURN_RIGHT_AHEAD,
            self.get_parameter('sign_left_class').value: SIGN_TURN_LEFT_AHEAD,
            self.get_parameter('sign_roadwork_class').value: SIGN_ROADWORK_AHEAD,
            self.get_parameter('sign_stop_class').value: SIGN_STOP,
            self.get_parameter('sign_giveway_class').value: SIGN_GIVE_WAY
        }
        
        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
        
        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Validate initial parameters
        init_params = [
            Parameter('update_rate', Parameter.Type.DOUBLE, self.update_rate),
            Parameter('detection_timeout', Parameter.Type.DOUBLE, self.detection_timeout),
            Parameter('full_speed_scale', Parameter.Type.DOUBLE, self.full_speed_scale),
            Parameter('mid_speed_scale', Parameter.Type.DOUBLE, self.mid_speed_scale),
            Parameter('searching_line_linear_speed', Parameter.Type.DOUBLE, self.searching_line_linear_speed),
            Parameter('turn_intersection_linear_speed', Parameter.Type.DOUBLE, self.turn_intersection_linear_speed),
            Parameter('turn_intersection_angular_speed', Parameter.Type.DOUBLE, self.turn_intersection_angular_speed),
            Parameter('turn_forward_distance', Parameter.Type.DOUBLE, self.turn_forward_distance),
            Parameter('turn_rotation_angle', Parameter.Type.DOUBLE, self.turn_rotation_angle),
            Parameter('stop_sign_area_threshold', Parameter.Type.DOUBLE, self.stop_sign_area_threshold),
        ]
        
        result = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")
        
        # Calculate execution times
        self.turn_forward_duration = self.turn_forward_distance / self.turn_intersection_linear_speed
        self.turn_rotation_duration = abs(self.turn_rotation_angle) / self.turn_intersection_angular_speed
        
        # Detection state
        self.current_light = ColorBlobDetection.COLOR_NONE
        self.current_sign = SIGN_NONE
        self.last_sign = SIGN_NONE
        self.last_directional_sign = SIGN_NONE
        
        # Stop sign area tracking
        self.stop_sign_area = 0.0
        self.stop_sign_close = False
        
        # Detection timeouts
        self.light_lost_time = None
        self.sign_lost_time = None
        
        # Controller state tracking
        self.controller_enabled = None
        self.current_velocity_scale = None
        
        # Line detection caching
        self.line_detected_cached = False
        self.last_line_status_check = 0.0
        self.line_status_check_interval = 0.1
        self.line_status_future = None
        
        # Open-loop control state
        self.inner_state = IDLE
        self.cmd_queue = []
        self.current_cmd = None
        self.cmd_start_time = None
        self.routine_active = False
        
        # Current action state
        self.current_action = ACTION_FULL_SPEED
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )
        
        # Subscribers
        self.create_subscription(
            ColorBlobDetection,
            'color_blob_detection',
            self.color_blob_callback,
            qos.qos_profile_sensor_data
        )
        
        self.create_subscription(
            Yolov8Inference,
            'Yolov8_Inference',
            self.sign_detection_callback,
            qos.qos_profile_sensor_data
        )
        
        # Service clients
        self.line_status_client = self.create_client(Trigger, 'line_detection/is_line_detected')
        self.controller_on_client = self.create_client(SetBool, self.line_follow_controller_on_service)
        self.controller_param_client = self.create_client(SetParameters, self.line_follow_controller_parameter_service)
        
        self.get_logger().info("TrafficFSM Start.")

    def color_blob_callback(self, msg: ColorBlobDetection):
        """Process color blob detection with timeout."""
        color = msg.color
        if color != ColorBlobDetection.COLOR_NONE:
            self.current_light = color
            self.light_lost_time = None
        elif self.light_lost_time is None:
            self.light_lost_time = self.get_clock().now().nanoseconds * 1e-9
        elif self.get_clock().now().nanoseconds * 1e-9 - self.light_lost_time > self.detection_timeout:
            self.current_light = ColorBlobDetection.COLOR_NONE
    
    def sign_detection_callback(self, msg: Yolov8Inference):
        """Process YOLO sign detection with timeout and area calculation for stop signs."""
        best_sign = SIGN_NONE
        max_area = 0
        stop_sign_detected = False
        current_stop_area = 0.0
        
        for inference in msg.yolov8_inference:
            detected_sign = self.sign_classes.get(inference.class_name, SIGN_NONE)
            if detected_sign != SIGN_NONE:
                area = (inference.bottom - inference.top) * (inference.right - inference.left)
                
                # Special handling for stop sign
                if detected_sign == SIGN_STOP:
                    stop_sign_detected = True
                    current_stop_area = max(current_stop_area, area)
                    
                    # Check if stop sign is close enough
                    if area > self.stop_sign_area_threshold:
                        self.stop_sign_close = True
                        self.get_logger().info(f"Stop sign close! Area: {area:.1f} > threshold: {self.stop_sign_area_threshold:.1f}.")
                    else:
                        self.stop_sign_close = False
                        self.get_logger().debug(f"Stop sign detected but far. Area: {area:.1f}", throttle_duration_sec=1.0)
                
                # Track the largest sign for general sign detection
                if area > max_area:
                    max_area = area
                    best_sign = detected_sign
        
        # Update stop sign area
        self.stop_sign_area = current_stop_area
        
        # If no stop sign detected, reset close flag
        if not stop_sign_detected:
            self.stop_sign_close = False
            self.stop_sign_area = 0.0
        
        # Update sign with timeout
        if best_sign != SIGN_NONE:
            if self.current_sign != SIGN_NONE and self.current_sign != best_sign:
                self.last_sign = self.current_sign
                if self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD]:
                    self.last_directional_sign = self.current_sign
            self.current_sign = best_sign
            self.sign_lost_time = None
        elif self.sign_lost_time is None:
            self.sign_lost_time = self.get_clock().now().nanoseconds * 1e-9
        elif self.get_clock().now().nanoseconds * 1e-9 - self.sign_lost_time > self.detection_timeout:
            if self.current_sign != SIGN_NONE:
                self.last_sign = self.current_sign
                if self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD]:
                    self.last_directional_sign = self.current_sign
            self.current_sign = SIGN_NONE
            self.stop_sign_close = False
            self.stop_sign_area = 0.0
    
    def check_line_status_async(self):
        """Asynchronously check line detection status."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if current_time - self.last_line_status_check < self.line_status_check_interval:
            return
            
        self.last_line_status_check = current_time
        
        if self.line_status_future is not None and not self.line_status_future.done():
            return
        
        if not self.line_status_client.service_is_ready():
            return
            
        try:
            request = Trigger.Request()
            self.line_status_future = self.line_status_client.call_async(request)
            self.line_status_future.add_done_callback(self.line_status_callback)
        except Exception as e:
            self.get_logger().warn(f"Error checking line status: {e}")
    
    def line_status_callback(self, future):
        """Callback for line detection status."""
        try:
            result = future.result()
            if result is not None:
                old_status = self.line_detected_cached
                self.line_detected_cached = result.success
                
                if old_status != self.line_detected_cached:
                    self.get_logger().info(f"Line detection status changed: {self.line_detected_cached}")
                    
        except Exception as e:
            self.get_logger().warn(f"Error processing line status: {e}")
        finally:
            self.line_status_future = None
    
    def is_line_detected(self) -> bool:
        """Get cached line detection status."""
        self.check_line_status_async()
        return self.line_detected_cached
    
    def call_controller_on(self, enable: bool):
        """Enable/disable line controller."""
        if self.controller_on_client.service_is_ready():
            req = SetBool.Request()
            req.data = enable
            self.controller_on_client.call_async(req)
            self.get_logger().info(f"Controller {'enabled' if enable else 'disabled'}.")
        else:
            self.get_logger().warn(f"Controller ON service not ready", throttle_duration_sec=2.0)
    
    def update_controller_param(self, param_name: str, value: float):
        """Update controller parameter."""
        if self.controller_param_client.service_is_ready():
            req = SetParameters.Request()
            req.parameters = [Parameter(param_name, Parameter.Type.DOUBLE, value).to_parameter_msg()]
            self.controller_param_client.call_async(req)
            self.get_logger().info(f"Updated {param_name} to {value}")
        else:
            self.get_logger().warn(f"Controller parameter service not ready", throttle_duration_sec=2.0)
    
    def determine_action(self) -> int:
        """Determine action based on current state."""
        # Get line detection status
        line_detected = self.is_line_detected()
        
        # Log current state
        self.get_logger().debug(
            f"State: Line={line_detected}, Light={self._light_name(self.current_light)}, "
            f"Sign={self._sign_name(self.current_sign)}, LastDir={self._sign_name(self.last_directional_sign)}, "
            f"StopClose={self.stop_sign_close}",
            throttle_duration_sec=1.0
        )
        
        # Stop sign (only if close) - highest priority
        if self.current_sign == SIGN_STOP and self.stop_sign_close:
            return ACTION_ZERO_SPEED
        
        # Red light - but ONLY if no stop sign is detected
        if self.current_light == ColorBlobDetection.COLOR_RED and self.current_sign != SIGN_STOP:
            return ACTION_ZERO_SPEED
        
        # Check for intersection start: no line + green light + directional sign
        if not line_detected and self.current_light == ColorBlobDetection.COLOR_GREEN:
            direction = (self.current_sign if self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD] 
                        else self.last_directional_sign)
            
            if direction == SIGN_AHEAD_ONLY:
                self.get_logger().info("Intersection detected: STRAIGHT")
                return ACTION_STRAIGHT_INTERSECTION
            elif direction == SIGN_TURN_RIGHT_AHEAD:
                self.get_logger().info("Intersection detected: RIGHT")
                return ACTION_RIGHT_INTERSECTION
            elif direction == SIGN_TURN_LEFT_AHEAD:
                self.get_logger().info("Intersection detected: LEFT")
                return ACTION_LEFT_INTERSECTION
        
        # Normal line following cases
        if (self.current_sign in [SIGN_ROADWORK_AHEAD, SIGN_GIVE_WAY] or
            (self.last_sign == SIGN_GIVE_WAY and self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD]) or
            self.current_light == ColorBlobDetection.COLOR_YELLOW):
            return ACTION_MID_SPEED
        
        return ACTION_FULL_SPEED
    
    def execute_action(self, action: int):
        """Execute the determined action."""
        if action in [ACTION_FULL_SPEED, ACTION_MID_SPEED]:
            # Only enable controller if not already enabled
            if self.controller_enabled != True:
                self.call_controller_on(True)
                self.controller_enabled = True
                
            new_scale = self.full_speed_scale if action == ACTION_FULL_SPEED else self.mid_speed_scale
            if self.current_velocity_scale != new_scale:
                self.update_controller_param("velocity_scale_factor", new_scale)
                self.current_velocity_scale = new_scale
            
        elif action == ACTION_ZERO_SPEED:
            # Only disable controller if not already disabled
            if self.controller_enabled != False:
                self.call_controller_on(False)
                self.controller_enabled = False
            self.current_velocity_scale = None
            # Stop the robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
        elif action == ACTION_STRAIGHT_INTERSECTION:
            self.execute_straight_intersection()
            
        elif action == ACTION_RIGHT_INTERSECTION:
            self.execute_right_intersection()
            
        elif action == ACTION_LEFT_INTERSECTION:
            self.execute_left_intersection()
    
    def execute_straight_intersection(self):
        """Execute straight intersection - immediately search for line."""
        self.get_logger().info("Starting STRAIGHT intersection crossing.")
        
        # Disable controller
        if self.controller_enabled != False:
            self.call_controller_on(False)
            self.controller_enabled = False
        
        # Transition to searching line
        self.current_action = ACTION_SEARCHING_LINE
    
    def execute_right_intersection(self):
        """Execute right turn intersection using open-loop control."""
        self.get_logger().info("Starting RIGHT intersection crossing.")
        
        # Disable controller
        if self.controller_enabled != False:
            self.call_controller_on(False)
            self.controller_enabled = False
        
        # Create forward command
        forward_cmd = {
            'linear_velocity': self.turn_intersection_linear_speed,
            'angular_velocity': 0.0,
            'execution_time': self.turn_forward_duration
        }
        
        # Create rotation command (negative for right turn)
        rotation_cmd = {
            'linear_velocity': 0.0,
            'angular_velocity': -self.turn_intersection_angular_speed,
            'execution_time': self.turn_rotation_duration
        }
        
        # Queue commands
        self.cmd_queue = [forward_cmd, rotation_cmd]
        self.inner_state = IDLE
        self.routine_active = True
        
        self.get_logger().info(f"Queued right turn: forward {self.turn_forward_distance}m, rotate {-self.turn_rotation_angle}rad")
    
    def execute_left_intersection(self):
        """Execute left turn intersection using open-loop control."""
        self.get_logger().info("Starting LEFT intersection crossing.")
        
        # Disable controller
        if self.controller_enabled != False:
            self.call_controller_on(False)
            self.controller_enabled = False
        
        # Create forward command
        forward_cmd = {
            'linear_velocity': self.turn_intersection_linear_speed,
            'angular_velocity': 0.0,
            'execution_time': self.turn_forward_duration
        }
        
        # Create rotation command (positive for left turn)
        rotation_cmd = {
            'linear_velocity': 0.0,
            'angular_velocity': self.turn_intersection_angular_speed,
            'execution_time': self.turn_rotation_duration
        }
        
        # Queue commands
        self.cmd_queue = [forward_cmd, rotation_cmd]
        self.inner_state = IDLE
        self.routine_active = True
        
        self.get_logger().info(f"Queued left turn: forward {self.turn_forward_distance}m, rotate {self.turn_rotation_angle}rad")
    
    def execute_inner_fsm(self):
        """Execute open-loop commands."""
        twist = Twist()
        
        if self.inner_state == IDLE:
            if not self.cmd_queue:
                # Commands finished, transition to searching
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.routine_active = False
                self.current_action = ACTION_SEARCHING_LINE
                self.get_logger().info("Open-loop routine completed, transitioning to SEARCHING_LINE")
            else:
                # Start next command
                self.current_cmd = self.cmd_queue.pop(0)
                self.cmd_start_time = self.get_clock().now()
                self.inner_state = EXECUTING
                self.get_logger().info(f"Executing: lin={self.current_cmd['linear_velocity']:.2f}, "
                                     f"ang={self.current_cmd['angular_velocity']:.2f}, "
                                     f"time={self.current_cmd['execution_time']:.2f}")
                
        elif self.inner_state == EXECUTING:
            elapsed = (self.get_clock().now() - self.cmd_start_time).nanoseconds * 1e-9
            
            if elapsed < self.current_cmd['execution_time']:
                # Continue executing
                twist.linear.x = self.current_cmd['linear_velocity']
                twist.angular.z = self.current_cmd['angular_velocity']
                self.cmd_vel_pub.publish(twist)
            else:
                # Command finished
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("Command completed.")
                
                self.current_cmd = None
                self.cmd_start_time = None
                self.inner_state = IDLE
    
    def execute_searching_line(self):
        """Search for line at constant speed."""
        twist = Twist()
        twist.linear.x = self.searching_line_linear_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Check if line detected
        if self.is_line_detected():
            self.get_logger().info("Line found! Returning to normal operation.")
            self.current_action = ACTION_FULL_SPEED
            # Re-enable controller
            if self.controller_enabled != True:
                self.call_controller_on(True)
                self.controller_enabled = True
    
    def fsm_update_loop(self):
        """Main FSM update loop."""
        # If executing open-loop routine, handle that first
        if self.routine_active:
            self.execute_inner_fsm()
            return
        
        # If searching for line, handle that
        if self.current_action == ACTION_SEARCHING_LINE:
            self.execute_searching_line()
            return
        
        # Otherwise, determine and execute action
        action = self.determine_action()
        self.current_action = action
        self.execute_action(action)
    
    def _sign_name(self, sign: int) -> str:
        """Get sign name string."""
        return {
            SIGN_NONE: 'NONE',
            SIGN_AHEAD_ONLY: 'AHEAD_ONLY',
            SIGN_TURN_RIGHT_AHEAD: 'TURN_RIGHT',
            SIGN_TURN_LEFT_AHEAD: 'TURN_LEFT',
            SIGN_ROADWORK_AHEAD: 'ROADWORK',
            SIGN_STOP: 'STOP',
            SIGN_GIVE_WAY: 'GIVE_WAY'
        }.get(sign, 'UNKNOWN')
    
    def _light_name(self, light: int) -> str:
        """Get light color name string."""
        return {
            ColorBlobDetection.COLOR_NONE: 'NONE',
            ColorBlobDetection.COLOR_RED: 'RED',
            ColorBlobDetection.COLOR_YELLOW: 'YELLOW',
            ColorBlobDetection.COLOR_GREEN: 'GREEN'
        }.get(light, 'UNKNOWN')
    
    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validate and apply parameters."""
        for param in params:
            if param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="update_rate must be > 0.")
                self.update_rate = float(param.value)
                if hasattr(self, 'timer'):
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
                    
            elif param.name == 'detection_timeout':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(successful=False, reason="detection_timeout must be >= 0.")
                self.detection_timeout = float(param.value)
                
            elif param.name in ['full_speed_scale', 'mid_speed_scale']:
                if not isinstance(param.value, (int, float)) or not (0.0 < param.value <= 1.0):
                    return SetParametersResult(successful=False, reason=f"{param.name} must be > 0.0 and <= 1.0.")
                setattr(self, param.name, float(param.value))
                
            elif param.name == 'searching_line_linear_speed':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="searching_line_linear_speed must be > 0.")
                self.searching_line_linear_speed = float(param.value)
                
            elif param.name == 'stop_sign_area_threshold':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="stop_sign_area_threshold must be > 0.")
                self.stop_sign_area_threshold = float(param.value)
                self.get_logger().info(f"Stop sign area threshold updated to {self.stop_sign_area_threshold}")
                
            elif param.name == 'turn_intersection_linear_speed':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(successful=False, reason="turn_intersection_linear_speed must be a number.")
                speed = float(param.value)
                if speed > self.max_turn_intersection_linear_speed or speed < self.min_turn_intersection_linear_speed:
                    return SetParametersResult(successful=False, reason="turn_intersection_linear_speed exceeds limits.")
                self.turn_intersection_linear_speed = speed
                if hasattr(self, 'turn_forward_distance'):
                    self.turn_forward_duration = self.turn_forward_distance / self.turn_intersection_linear_speed
                
            elif param.name == 'turn_intersection_angular_speed':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(successful=False, reason="turn_intersection_angular_speed must be a number.")
                speed = float(param.value)
                if speed > self.max_turn_intersection_angular_speed or speed < self.min_turn_intersection_angular_speed:
                    return SetParametersResult(successful=False, reason="turn_intersection_angular_speed exceeds limits.")
                self.turn_intersection_angular_speed = speed
                if hasattr(self, 'turn_rotation_angle'):
                    self.turn_rotation_duration = abs(self.turn_rotation_angle) / self.turn_intersection_angular_speed
                
            elif param.name == 'turn_forward_distance':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="turn_forward_distance must be > 0.")
                self.turn_forward_distance = float(param.value)
                if hasattr(self, 'turn_intersection_linear_speed'):
                    self.turn_forward_duration = self.turn_forward_distance / self.turn_intersection_linear_speed
                
            elif param.name == 'turn_rotation_angle':
                if not isinstance(param.value, (int, float)):
                    return SetParametersResult(successful=False, reason="turn_rotation_angle must be a number.")
                self.turn_rotation_angle = float(param.value)
                if hasattr(self, 'turn_intersection_angular_speed'):
                    self.turn_rotation_duration = abs(self.turn_rotation_angle) / self.turn_intersection_angular_speed
                    
            elif param.name in ['min_turn_intersection_linear_speed', 'max_turn_intersection_linear_speed',
                               'min_turn_intersection_angular_speed', 'max_turn_intersection_angular_speed']:
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be > 0.")
                setattr(self, param.name, float(param.value))
                
            elif param.name.startswith('sign_') and param.name.endswith('_class'):
                if not isinstance(param.value, str) or len(param.value.strip()) == 0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be non-empty.")
                # Update sign class mapping
                sign_type_map = {
                    'sign_ahead_class': SIGN_AHEAD_ONLY,
                    'sign_right_class': SIGN_TURN_RIGHT_AHEAD,
                    'sign_left_class': SIGN_TURN_LEFT_AHEAD,
                    'sign_roadwork_class': SIGN_ROADWORK_AHEAD,
                    'sign_stop_class': SIGN_STOP,
                    'sign_giveway_class': SIGN_GIVE_WAY
                }
                
                sign_type = sign_type_map.get(param.name)
                if sign_type is not None:
                    # Remove old class mapping
                    old_classes = [cls for cls, val in self.sign_classes.items() if val == sign_type]
                    for cls in old_classes:
                        self.sign_classes.pop(cls, None)
                    # Add new class mapping
                    self.sign_classes[param.value] = sign_type
                    self.get_logger().info(f"{param.name} updated: {param.value}")
                    
            elif param.name in ['line_follow_controller_on_service', 'line_follow_controller_parameter_service']:
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(successful=False, reason=f"{param.name} must be a non-empty string.")
                setattr(self, param.name, param.value)
                # Recreate service clients
                if param.name == 'line_follow_controller_on_service':
                    self.controller_on_client = self.create_client(SetBool, self.line_follow_controller_on_service)
                else:
                    self.controller_param_client = self.create_client(SetParameters, self.line_follow_controller_parameter_service)
                self.get_logger().info(f"{param.name} updated: {param.value}")
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TrafficFSM()
    except Exception as e:
        print(f"[FATAL] TrafficFSM failed to initialize: {e}", file=sys.stderr)
        rclpy.shutdown()
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted with Ctrl+C.")
    finally:
        # Ensure robot stops
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_vel_pub.publish(twist)
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()