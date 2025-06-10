#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy import qos

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters

from custom_interfaces.msg import ColorBlobDetection
from yolov8_msgs.msg import Yolov8Inference

from geometry_msgs.msg import Twist

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

class TrafficFSM(Node):
    def __init__(self):
        super().__init__('traffic_fsm')

        self.declare_parameter('update_rate', 60.0)                                # Hz

        self.declare_parameter('detection_timeout', 1.0)                           # s 

        self.declare_parameter('full_speed_scale', 1.0)                                 
        self.declare_parameter('mid_speed_scale', 0.6)        

        self.declare_parameter('line_follow_controller_parameter_service', 'line_follow_controller/set_parameters')
        self.declare_parameter('line_follow_controller_on_service', 'line_follow_controller/controller_on')

        self.declare_parameter('stop_sign_min_area', 0.1)  

        self.declare_parameter('straight_intersection_linear_vel', 0.06)           # m/s

        self.declare_parameter('turn_intersection_linear_vel', 0.06)               # m/s
        self.declare_parameter('turn_intersection_angular_vel', 1.0)               # rad/s

        self.declare_parameter('turn_forward_distance', 0.35)                      # meters
        self.declare_parameter('turn_rotation_angle', 1.57)                        # radians

        self.declare_parameter('image_width', 640)                                 # pixels
        self.declare_parameter('image_height', 480)                                # pixels
        
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

        self.line_follow_controller_on_service = self.get_parameter('line_follow_controller_on_service').value 
        self.line_follow_controller_parameter_service = self.get_parameter('line_follow_controller_parameter_service').value

        self.stop_sign_min_area = self.get_parameter('stop_sign_min_area').value

        self.straight_intersection_linear_vel = self.get_parameter('straight_intersection_linear_vel').value

        self.turn_intersection_linear_vel = self.get_parameter('turn_intersection_linear_vel').value
        self.turn_intersection_angular_vel = self.get_parameter('turn_intersection_angular_vel').value

        self.turn_forward_distance = self.get_parameter('turn_forward_distance').value
        self.turn_rotation_angle = self.get_parameter('turn_rotation_angle').value

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # Sign class mapping
        self.sign_classes = {
            self.get_parameter('sign_ahead_class').value: SIGN_AHEAD_ONLY,
            self.get_parameter('sign_right_class').value: SIGN_TURN_RIGHT_AHEAD,
            self.get_parameter('sign_left_class').value: SIGN_TURN_LEFT_AHEAD,
            self.get_parameter('sign_roadwork_class').value: SIGN_ROADWORK_AHEAD,
            self.get_parameter('sign_stop_class').value: SIGN_STOP,
            self.get_parameter('sign_giveway_class').value: SIGN_GIVE_WAY
        }

        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
        
        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Validate initial parameters
        init_params = [
            Parameter('update_rate',                                Parameter.Type.DOUBLE,  self.update_rate),
            Parameter('detection_timeout',                          Parameter.Type.DOUBLE,  self.detection_timeout),
            Parameter('full_speed_scale',                           Parameter.Type.DOUBLE,  self.full_speed_scale),
            Parameter('mid_speed_scale',                            Parameter.Type.DOUBLE,  self.mid_speed_scale),
            Parameter('line_follow_controller_parameter_service',   Parameter.Type.STRING,  self.line_follow_controller_parameter_service),
            Parameter('line_follow_controller_on_service',          Parameter.Type.STRING,  self.line_follow_controller_on_service),
            Parameter('stop_sign_min_area',                         Parameter.Type.DOUBLE,  self.stop_sign_min_area),
            Parameter('straight_intersection_linear_vel',           Parameter.Type.DOUBLE,  self.straight_intersection_linear_vel),
            Parameter('turn_intersection_linear_vel',               Parameter.Type.DOUBLE,  self.turn_intersection_linear_vel),
            Parameter('turn_intersection_angular_vel',              Parameter.Type.DOUBLE,  self.turn_intersection_angular_vel),
            Parameter('turn_forward_distance',                      Parameter.Type.DOUBLE,  self.turn_forward_distance),
            Parameter('turn_rotation_angle',                        Parameter.Type.DOUBLE,  self.turn_rotation_angle),
            Parameter('image_width',                                Parameter.Type.INTEGER, self.image_width),
            Parameter('image_height',                               Parameter.Type.INTEGER, self.image_height),
            Parameter('sign_ahead_class',                           Parameter.Type.STRING,  self.get_parameter('sign_ahead_class').value),
            Parameter('sign_right_class',                           Parameter.Type.STRING,  self.get_parameter('sign_right_class').value),
            Parameter('sign_left_class',                            Parameter.Type.STRING,  self.get_parameter('sign_left_class').value),
            Parameter('sign_roadwork_class',                        Parameter.Type.STRING,  self.get_parameter('sign_roadwork_class').value),
            Parameter('sign_stop_class',                            Parameter.Type.STRING,  self.get_parameter('sign_stop_class').value),
            Parameter('sign_giveway_class',                          Parameter.Type.STRING,  self.get_parameter('sign_giveway_class').value),
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")

        # Calculate durations
        self.turn_forward_duration = self.turn_forward_distance / self.turn_intersection_linear_vel
        self.turn_rotation_duration = abs(self.turn_rotation_angle) / self.turn_intersection_angular_vel
        
        # Detection state
        self.current_light = ColorBlobDetection.COLOR_NONE
        self.current_sign = SIGN_NONE
        self.last_sign = SIGN_NONE 
        self.last_directional_sign = SIGN_NONE
        
        # Detection timeouts
        self.light_lost_time = None
        self.sign_lost_time = None
        
        # Controller state tracking
        self.controller_enabled = None
        self.current_velocity_scale = None
        
        # Line detection state
        self.line_detected = False
        
        # Internal detection flags
        self.process_color_blob = True
        self.process_signs = True
        self.process_line_detection = True
        
        # Flag to prevent detection updates during intersection
        self.crossing_intersection = False
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Publisher for velocity commands
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

        # Service client to check line detection status
        self.line_status_client = self.create_client(
            Trigger, 
            'line_detection/is_line_detected'
        )
        
        # Service clients for controller
        self.controller_parameter_client = self.create_client(SetParameters, self.line_follow_controller_parameter_service)
        while not self.controller_parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.line_follow_controller_parameter_service} service not available, waiting...')
        
        self.controller_on_client = self.create_client(SetBool, self.line_follow_controller_on_service)
        while not self.controller_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{self.line_follow_controller_on_service} service not available, waiting...")
        
        self.get_logger().info('TrafficFSM Start.')

    def color_blob_callback(self, msg: ColorBlobDetection) -> None:
        """Process color blob detection."""
        # Skip if we're not processing color blobs
        if not self.process_color_blob or self.crossing_intersection:
            return
            
        color = msg.color
        if color != ColorBlobDetection.COLOR_NONE:
            self.current_light = color
            self.light_lost_time = None
        elif self.light_lost_time is None:
            self.light_lost_time = self.get_clock().now().nanoseconds * 1e-9
        elif self.get_clock().now().nanoseconds * 1e-9 - self.light_lost_time > self.detection_timeout:
            self.current_light = ColorBlobDetection.COLOR_NONE

    def sign_detection_callback(self, msg: Yolov8Inference) -> None:
        """Process YOLO sign detection."""
        # Skip if we're not processing signs
        if not self.process_signs or self.crossing_intersection:
            return
            
        best_sign = SIGN_NONE
        max_area = 0
        
        for inference in msg.yolov8_inference:
            detected_sign = self.sign_classes.get(inference.class_name, SIGN_NONE)
            if detected_sign != SIGN_NONE:
                area = (inference.bottom - inference.top) * (inference.right - inference.left)
                
                # Check stop sign distance
                if detected_sign == SIGN_STOP:
                    normalized_area = area / (self.image_width * self.image_height)
                    if normalized_area < self.stop_sign_min_area:
                        continue 
                
                if area > max_area:
                    max_area = area
                    best_sign = detected_sign
        
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

    def check_line_status(self):
        """Check line detection status via service."""
        if not self.process_line_detection:
            self.line_detected = False
            return
            
        if not self.line_status_client.service_is_ready():
            return
            
        try:
            request = Trigger.Request()
            future = self.line_status_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            
            if future.done():
                result = future.result()
                if result is not None:
                    old_status = self.line_detected
                    self.line_detected = result.success
                    
                    if old_status != self.line_detected:
                        self.get_logger().info(f"Line detection status changed: {self.line_detected}.")
                        
        except Exception as e:
            self.get_logger().warn(f"Error checking line detection: {e}")

    def is_line_detected(self) -> bool:
        """Check if line is detected."""
        if not self.process_line_detection:
            return False
        return self.line_detected
        
    def call_controller_on(self, enable: bool) -> None:
        """Enable/disable line controller via service."""
        if self.controller_on_client.service_is_ready():
            req = SetBool.Request()
            req.data = enable
            self.controller_on_client.call_async(req)
        
    def update_controller_param(self, param_name: str, value: float) -> None:
        """Update controller parameter."""
        if self.controller_parameter_client.service_is_ready():
            req = SetParameters.Request()
            req.parameters = [Parameter(param_name, Parameter.Type.DOUBLE, value).to_parameter_msg()]
            self.controller_parameter_client.call_async(req)

    def publish_velocity_command(self, linear: float, angular: float) -> None:
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def spin_with_velocity(self, linear: float, angular: float) -> bool:
        """Spin once while publishing velocity commands. Returns False if shutdown requested."""
        if self.shutdown_requested:
            return False
            
        self.publish_velocity_command(linear, angular)
        rclpy.spin_once(self, timeout_sec=0.01)
        return not self.shutdown_requested

    def determine_action(self) -> int:
        """Determine action based on current state."""
        # Get line detection status
        line_detected = self.is_line_detected()
        
        # Stop sign or red light
        if self.current_sign == SIGN_STOP or self.current_light == ColorBlobDetection.COLOR_RED:
            return ACTION_ZERO_SPEED
        
        # Check for intersection start: no line + green light + directional sign
        if not line_detected and self.current_light == ColorBlobDetection.COLOR_GREEN:
            direction = (self.current_sign if self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD] 
                        else self.last_directional_sign)
            
            if direction == SIGN_AHEAD_ONLY:
                return ACTION_STRAIGHT_INTERSECTION
            elif direction == SIGN_TURN_RIGHT_AHEAD:
                return ACTION_RIGHT_INTERSECTION
            elif direction == SIGN_TURN_LEFT_AHEAD:
                return ACTION_LEFT_INTERSECTION
        
        # Normal line following cases
        if (self.current_sign in [SIGN_ROADWORK_AHEAD, SIGN_GIVE_WAY] or
            (self.last_sign == SIGN_GIVE_WAY and self.current_sign in [SIGN_AHEAD_ONLY, SIGN_TURN_RIGHT_AHEAD, SIGN_TURN_LEFT_AHEAD]) or
            self.current_light == ColorBlobDetection.COLOR_YELLOW):
            return ACTION_MID_SPEED
        
        return ACTION_FULL_SPEED

    def execute_action(self, action: int) -> None:
        """Execute the determined action."""
        if action in [ACTION_FULL_SPEED, ACTION_MID_SPEED]:
            # Only enable controller if it's not already enabled
            if self.controller_enabled != True:
                self.call_controller_on(True)
                self.controller_enabled = True
                
            new_scale = self.full_speed_scale if action == ACTION_FULL_SPEED else self.mid_speed_scale
            if self.current_velocity_scale != new_scale:
                self.update_controller_param("velocity_scale_factor", new_scale)
                self.current_velocity_scale = new_scale
            
        elif action == ACTION_ZERO_SPEED:
            # Only disable controller if it's not already disabled
            if self.controller_enabled != False:
                self.call_controller_on(False)
                self.controller_enabled = False
            self.current_velocity_scale = None
            
        elif action == ACTION_STRAIGHT_INTERSECTION:
            self.execute_straight_intersection()
            
        elif action == ACTION_RIGHT_INTERSECTION:
            self.execute_right_intersection()
            
        elif action == ACTION_LEFT_INTERSECTION:
            self.execute_left_intersection()

    def execute_straight_intersection(self):
        """Execute straight intersection crossing."""
        self.crossing_intersection = True
        self.get_logger().info("Starting STRAIGHT intersection crossing.")
        
        try:
            # Disable color blob and sign processing (keep line detection)
            self.process_color_blob = False
            self.process_signs = False
            
            # Move forward until line detected
            self.get_logger().info("Moving forward until line detected.")
            while not self.is_line_detected() and not self.shutdown_requested:
                if not self.spin_with_velocity(self.straight_intersection_linear_vel, 0.0):
                    break
            
            if not self.shutdown_requested:
                self.get_logger().info("Line detected, straight intersection complete.")
                
        except Exception as e:
            self.get_logger().error(f"Error during straight intersection: {e}")
        finally:
            # Re-enable all processing
            self.process_color_blob = True
            self.process_signs = True
            self.crossing_intersection = False
            self.get_logger().info("STRAIGHT intersection crossing complete.")

    def execute_right_intersection(self):
        """Execute right turn intersection."""
        self.crossing_intersection = True
        self.get_logger().info("Starting RIGHT intersection crossing.")
        
        try:
            # Disable ALL processing including line detection
            self.process_color_blob = False
            self.process_signs = False
            self.process_line_detection = False
            
            # Phase 1: Forward
            self.get_logger().info(f"Phase 1: Forward {self.turn_forward_distance}m.")
            start_time = self.get_clock().now().nanoseconds * 1e-9
            while (self.get_clock().now().nanoseconds * 1e-9 - start_time < self.turn_forward_duration 
                   and not self.shutdown_requested):
                if not self.spin_with_velocity(self.turn_intersection_linear_vel, 0.0):
                    break
            
            # Phase 2: Rotate right
            if not self.shutdown_requested:
                self.get_logger().info(f"Phase 2: Rotate right {self.turn_rotation_angle} rad.")
                start_time = self.get_clock().now().nanoseconds * 1e-9
                while (self.get_clock().now().nanoseconds * 1e-9 - start_time < self.turn_rotation_duration 
                       and not self.shutdown_requested):
                    if not self.spin_with_velocity(0.0, -self.turn_intersection_angular_vel):
                        break
            
            # Phase 3: Re-enable line detection and move forward until line
            if not self.shutdown_requested:
                self.get_logger().info("Phase 3: Forward until line detected.")
                self.process_line_detection = True
                while not self.is_line_detected() and not self.shutdown_requested:
                    if not self.spin_with_velocity(self.turn_intersection_linear_vel, 0.0):
                        break
                
                if not self.shutdown_requested:
                    self.get_logger().info("Line detected, right turn complete.")
                
        except Exception as e:
            self.get_logger().error(f"Error during right intersection: {e}")
        finally:
            # Re-enable all processing
            self.process_color_blob = True
            self.process_signs = True
            self.process_line_detection = True
            self.crossing_intersection = False
            self.get_logger().info("RIGHT intersection crossing complete.")

    def execute_left_intersection(self):
        """Execute left turn intersection."""
        self.crossing_intersection = True
        self.get_logger().info("Starting LEFT intersection crossing.")
        
        try:
            # Disable ALL processing including line detection
            self.process_color_blob = False
            self.process_signs = False
            self.process_line_detection = False
            
            # Phase 1: Forward
            self.get_logger().info(f"Phase 1: Forward {self.turn_forward_distance}m.")
            start_time = self.get_clock().now().nanoseconds * 1e-9
            while (self.get_clock().now().nanoseconds * 1e-9 - start_time < self.turn_forward_duration 
                   and not self.shutdown_requested):
                if not self.spin_with_velocity(self.turn_intersection_linear_vel, 0.0):
                    break
            
            # Phase 2: Rotate left
            if not self.shutdown_requested:
                self.get_logger().info(f"Phase 2: Rotate left {self.turn_rotation_angle} rad.")
                start_time = self.get_clock().now().nanoseconds * 1e-9
                while (self.get_clock().now().nanoseconds * 1e-9 - start_time < self.turn_rotation_duration 
                       and not self.shutdown_requested):
                    if not self.spin_with_velocity(0.0, self.turn_intersection_angular_vel):
                        break
            
            # Phase 3: Re-enable line detection and move forward until line
            if not self.shutdown_requested:
                self.get_logger().info("Phase 3: Forward until line detected.")
                self.process_line_detection = True
                while not self.is_line_detected() and not self.shutdown_requested:
                    if not self.spin_with_velocity(self.turn_intersection_linear_vel, 0.0):
                        break
                
                if not self.shutdown_requested:
                    self.get_logger().info("Line detected, left turn complete.")
                
        except Exception as e:
            self.get_logger().error(f"Error during left intersection: {e}")
        finally:
            # Re-enable all processing
            self.process_color_blob = True
            self.process_signs = True
            self.process_line_detection = True
            self.crossing_intersection = False
            self.get_logger().info("LEFT intersection crossing complete.")

    def fsm_update_loop(self) -> None:
        """Main FSM update loop."""
        if not self.shutdown_requested:
            # Update line detection status
            self.check_line_status()
            
            # Determine and execute action
            action = self.determine_action()
            self.execute_action(action)

    def destroy_node(self):
        """Override destroy_node to handle cleanup."""
        self.shutdown_requested = True
        
        # Ensure all processing is re-enabled
        self.get_logger().info("Shutdown requested - ensuring all processing is enabled.")
        self.process_color_blob = True
        self.process_signs = True
        self.process_line_detection = True
        
        # Stop the robot
        self.publish_velocity_command(0.0, 0.0)
        
        super().destroy_node()
        
    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validate and apply parameters."""
        for param in params:
            if param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="update_rate must be > 0.")

            elif param.name == 'detection_timeout':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(successful=False, reason="detection_timeout must be >= 0.")

            elif param.name == 'full_speed_scale':
                if not isinstance(param.value, (int, float)) or not (0.0 < param.value <= 1.0):
                    return SetParametersResult(successful=False, reason="full_speed_scale must be > 0.0 and <= 1.0.")

            elif param.name == 'mid_speed_scale':
                if not isinstance(param.value, (int, float)) or not (0.0 < param.value <= 1.0):
                    return SetParametersResult(successful=False, reason="mid_speed_scale must be > 0.0 and <= 1.0.")

            elif param.name in ['line_follow_controller_on_service', 'line_follow_controller_parameter_service']:
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(successful=False, reason=f"{param.name} must be a non-empty string.")
                    
            elif param.name in ['stop_sign_min_area', 'straight_intersection_linear_vel', 
                              'turn_intersection_linear_vel', 'turn_intersection_angular_vel',
                              'turn_forward_distance', 'turn_rotation_angle']:
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be > 0.")
                    
            elif param.name in ['image_width', 'image_height']:
                if not isinstance(param.value, int) or param.value <= 0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be a positive integer.")
                    
            elif param.name.startswith('sign_') and param.name.endswith('_class'):
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(successful=False, reason=f"{param.name} must be a non-empty string.")

        # Apply parameters after validation
        for param in params:
            if param.name == 'update_rate':
                self.update_rate = float(param.value)
                if hasattr(self, 'timer') and self.timer is not None:
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

            elif param.name == 'detection_timeout':
                self.detection_timeout = float(param.value)
                self.get_logger().info(f"detection_timeout updated: {self.detection_timeout} s.")

            elif param.name == 'full_speed_scale':
                self.full_speed_scale = float(param.value)
                self.get_logger().info(f"full_speed_scale updated: {self.full_speed_scale}.")

            elif param.name == 'mid_speed_scale':
                self.mid_speed_scale = float(param.value)
                self.get_logger().info(f"mid_speed_scale updated: {self.mid_speed_scale}.")

            elif param.name == 'line_follow_controller_on_service':
                self.line_follow_controller_on_service = param.value
                self.controller_on_client = self.create_client(SetBool, self.line_follow_controller_on_service)
                self.get_logger().info(f"line_follow_controller_on_service updated: {self.line_follow_controller_on_service}.")

            elif param.name == 'line_follow_controller_parameter_service':
                self.line_follow_controller_parameter_service = param.value
                self.controller_parameter_client = self.create_client(SetParameters, self.line_follow_controller_parameter_service)
                self.get_logger().info(f"line_follow_controller_parameter_service updated: {self.line_follow_controller_parameter_service}.")
                
            elif param.name == 'stop_sign_min_area':
                self.stop_sign_min_area = float(param.value)
                self.get_logger().info(f"stop_sign_min_area updated: {self.stop_sign_min_area}.")
                
            elif param.name == 'straight_intersection_linear_vel':
                self.straight_intersection_linear_vel = float(param.value)
                self.get_logger().info(f"straight_intersection_linear_vel updated: {self.straight_intersection_linear_vel} m/s.")
                
            elif param.name == 'turn_intersection_linear_vel':
                self.turn_intersection_linear_vel = float(param.value)
                self.turn_forward_duration = self.turn_forward_distance / self.turn_intersection_linear_vel
                self.get_logger().info(f"turn_intersection_linear_vel updated: {self.turn_intersection_linear_vel} m/s.")
                
            elif param.name == 'turn_intersection_angular_vel':
                self.turn_intersection_angular_vel = float(param.value)
                self.turn_rotation_duration = abs(self.turn_rotation_angle) / self.turn_intersection_angular_vel
                self.get_logger().info(f"turn_intersection_angular_vel updated: {self.turn_intersection_angular_vel} rad/s.")
                
            elif param.name == 'turn_forward_distance':
                self.turn_forward_distance = float(param.value)
                self.turn_forward_duration = self.turn_forward_distance / self.turn_intersection_linear_vel
                self.get_logger().info(f"turn_forward_distance updated: {self.turn_forward_distance} m.")
                
            elif param.name == 'turn_rotation_angle':
                self.turn_rotation_angle = float(param.value)
                self.turn_rotation_duration = abs(self.turn_rotation_angle) / self.turn_intersection_angular_vel
                self.get_logger().info(f"turn_rotation_angle updated: {self.turn_rotation_angle} rad.")

            elif param.name == 'image_width':
                self.image_width = param.value
                self.get_logger().info(f"image_width updated: {self.image_width} pixels.")

            elif param.name == 'image_height':
                self.image_height = param.value
                self.get_logger().info(f"image_height updated: {self.image_height} pixels.")

            elif param.name.startswith('sign_') and param.name.endswith('_class'):
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()