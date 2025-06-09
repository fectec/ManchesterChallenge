#!/usr/bin/env python3

import sys
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

class LineDectection(Node):
    def __init__(self):
        super().__init__('line_detection')

        # Declare parameters
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('use_compressed', False)

        self.declare_parameter('update_rate', 30.0)

        self.declare_parameter('target_width', 640)      # pixels
        self.declare_parameter('target_height', 480)
        
        # Bird's eye view perspective transformation points
        # These should be calibrated for your specific camera setup
        self.declare_parameter('perspective_tl_x', 190)  # Top-left (pixels)
        self.declare_parameter('perspective_tl_y', 350)
        self.declare_parameter('perspective_bl_x', 190)  # Bottom-left (pixels)
        self.declare_parameter('perspective_bl_y', 479)
        self.declare_parameter('perspective_tr_x', 450)  # Top-right (pixels)
        self.declare_parameter('perspective_tr_y', 350)
        self.declare_parameter('perspective_br_x', 450)  # Bottom-right (pixels)
        self.declare_parameter('perspective_br_y', 479)

        # Gaussian blur parameters
        self.declare_parameter('gaussian_kernel_size', 5)
        self.declare_parameter('gaussian_sigma', 5)
        
        # Thresholding and morphology parameters
        self.declare_parameter('grayscale_threshold', 95)
        self.declare_parameter('morph_kernel_size', 3)
        self.declare_parameter('morph_erode_iterations', 40)
        self.declare_parameter('morph_dilate_iterations', 90)
        
        # Centroid calculation parameters
        self.declare_parameter('min_contour_area', 180)
        self.declare_parameter('max_contour_area', 50000)
        
        # Filtering parameters
        self.declare_parameter('filter_alpha', 0.1)

        # Line continuity detection parameters
        self.declare_parameter('min_line_continuity_area', 7500) 
        self.declare_parameter('min_line_height', 100)  
        
        # Retrieve parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value

        self.update_rate = self.get_parameter('update_rate').value

        self.target_width = self.get_parameter('target_width').value
        self.target_height = self.get_parameter('target_height').value
        
        self.perspective_tl_x = self.get_parameter('perspective_tl_x').value
        self.perspective_tl_y = self.get_parameter('perspective_tl_y').value
        self.perspective_bl_x = self.get_parameter('perspective_bl_x').value
        self.perspective_bl_y = self.get_parameter('perspective_bl_y').value
        self.perspective_tr_x = self.get_parameter('perspective_tr_x').value
        self.perspective_tr_y = self.get_parameter('perspective_tr_y').value
        self.perspective_br_x = self.get_parameter('perspective_br_x').value
        self.perspective_br_y = self.get_parameter('perspective_br_y').value
        
        self.gaussian_kernel_size = self.get_parameter('gaussian_kernel_size').value
        self.gaussian_sigma = self.get_parameter('gaussian_sigma').value

        self.grayscale_threshold = self.get_parameter('grayscale_threshold').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.morph_erode_iterations = self.get_parameter('morph_erode_iterations').value
        self.morph_dilate_iterations = self.get_parameter('morph_dilate_iterations').value

        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.max_contour_area = self.get_parameter('max_contour_area').value

        self.filter_alpha = self.get_parameter('filter_alpha').value
        
        self.min_line_continuity_area = self.get_parameter('min_line_continuity_area').value
        self.min_line_height = self.get_parameter('min_line_height').value

        # Timer for periodic processing
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Service servers for pause/resume
        self.create_service(Trigger, 'line_detection/pause_timer', self.pause_timer_callback)
        self.create_service(Trigger, 'line_detection/resume_timer', self.resume_timer_callback)
        self.timer_active = True
        
        # Validate initial parameters
        init_params = [
            Parameter('image_topic',                Parameter.Type.STRING,  self.image_topic),
            Parameter('use_compressed',             Parameter.Type.BOOL,    self.use_compressed),
            Parameter('update_rate',                Parameter.Type.DOUBLE,  self.update_rate),
            Parameter('target_width',               Parameter.Type.INTEGER, self.target_width),
            Parameter('target_height',              Parameter.Type.INTEGER, self.target_height),
            Parameter('perspective_tl_x',           Parameter.Type.INTEGER, self.perspective_tl_x),
            Parameter('perspective_tl_y',           Parameter.Type.INTEGER, self.perspective_tl_y),
            Parameter('perspective_bl_x',           Parameter.Type.INTEGER, self.perspective_bl_x),
            Parameter('perspective_bl_y',           Parameter.Type.INTEGER, self.perspective_bl_y),
            Parameter('perspective_tr_x',           Parameter.Type.INTEGER, self.perspective_tr_x),
            Parameter('perspective_tr_y',           Parameter.Type.INTEGER, self.perspective_tr_y),
            Parameter('perspective_br_x',           Parameter.Type.INTEGER, self.perspective_br_x),
            Parameter('perspective_br_y',           Parameter.Type.INTEGER, self.perspective_br_y),
            Parameter('gaussian_kernel_size',       Parameter.Type.INTEGER, self.gaussian_kernel_size),
            Parameter('gaussian_sigma',             Parameter.Type.INTEGER, self.gaussian_sigma),
            Parameter('grayscale_threshold',        Parameter.Type.INTEGER, self.grayscale_threshold),
            Parameter('morph_kernel_size',          Parameter.Type.INTEGER, self.morph_kernel_size),
            Parameter('morph_erode_iterations',     Parameter.Type.INTEGER, self.morph_erode_iterations),
            Parameter('morph_dilate_iterations',    Parameter.Type.INTEGER, self.morph_dilate_iterations),
            Parameter('min_contour_area',           Parameter.Type.INTEGER, self.min_contour_area),
            Parameter('max_contour_area',           Parameter.Type.INTEGER, self.max_contour_area),
            Parameter('filter_alpha',               Parameter.Type.DOUBLE,  self.filter_alpha),
            Parameter('min_line_continuity_area',   Parameter.Type.INTEGER, self.min_line_continuity_area),
            Parameter('min_line_height',            Parameter.Type.INTEGER, self.min_line_height),
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")
        
        # State for filtering
        self.last_centroid_error = None
        
        # Line detection state
        self.is_line_detected = False
        
        # Initialize variables
        self.image = None
        self.bridge = CvBridge()
        
        # Compute perspective transformation matrices
        self.update_perspective_matrices()
        
        # Publishers
        self.thresholded_pub = self.create_publisher(Image, 'line_detection/thresholded', qos.qos_profile_sensor_data)
        self.detected_line_pub = self.create_publisher(Image, 'line_detection/detected_line', qos.qos_profile_sensor_data)
        self.centroid_error_pub = self.create_publisher(Float32, 'line_detection/centroid_error', qos.qos_profile_sensor_data)

        # Service server to check if line is detected
        self.line_status_service = self.create_service(
            Trigger, 
            'line_detection/is_line_detected', 
            self.line_status_callback
        )

        # Create subscribers based on compression setting
        if self.use_compressed:
            self.create_subscription(
                CompressedImage,
                f"{self.image_topic}/compressed",
                self.image_callback,
                qos.qos_profile_sensor_data
            )
        else:
            self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                qos.qos_profile_sensor_data
            )
     
        self.get_logger().info('LineDectection Start.')

    def pause_timer_callback(self, request, response):
        """Service callback to pause the timer."""
        if self.timer is not None and self.timer_active:
            self.timer.cancel()
            self.timer = None
            self.timer_active = False
            self.get_logger().info('Timer paused.')
            response.success = True
            response.message = "Timer paused."
        else:
            response.success = False
            response.message = "Timer already paused or not initialized."
        return response

    def resume_timer_callback(self, request, response):
        """Service callback to resume the timer."""
        if self.timer is None and not self.timer_active:
            self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
            self.timer_active = True
            self.get_logger().info('Timer resumed.')
            response.success = True
            response.message = "Timer resumed."
        else:
            response.success = False
            response.message = "Timer is already running."
        return response

    def line_status_callback(self, request, response):
        """Service callback to report line detection status."""
        response.success = self.is_line_detected
        if self.is_line_detected:
            response.message = "Line is detected and being tracked."
        else:
            response.message = "No line detected."
        return response

    def update_perspective_matrices(self):
        """Update perspective transformation matrices based on current parameters."""
        # Define source points (perspective points in original image)
        tl = (self.perspective_tl_x, self.perspective_tl_y)
        bl = (self.perspective_bl_x, self.perspective_bl_y)
        tr = (self.perspective_tr_x, self.perspective_tr_y)
        br = (self.perspective_br_x, self.perspective_br_y)
        
        # Define destination points (rectangle in bird's eye view)
        pts1 = np.float32([tl, bl, tr, br])
        pts2 = np.float32([[0, 0], [0, self.target_height], [self.target_width, 0], [self.target_width, self.target_height]])
        
        # Compute transformation matrices
        self.perspective_matrix = cv2.getPerspectiveTransform(pts1, pts2)
        self.inverse_perspective_matrix = cv2.getPerspectiveTransform(pts2, pts1)
        
        self.get_logger().info(f"Perspective transformation updated with points: TL{tl}, BL{bl}, TR{tr}, BR{br}.")

    def calculate_lane_centroid(self, mask):
        """
        Calculate the centroid of line contours for IBVS control
        Returns: (centroid_x, centroid_y, lane_angle, lane_width, is_continuous_line)
        """
        # Find all contours in the binary mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, None, None, False
        
        # Filter contours by area to remove noise
        valid_contours = [cnt for cnt in contours 
                         if self.min_contour_area < cv2.contourArea(cnt) < self.max_contour_area]
        
        if not valid_contours:
            return None, None, None, None, False
        
        # Check for continuous line
        total_valid_area = sum(cv2.contourArea(cnt) for cnt in valid_contours)
        max_contour_height = 0
        
        # Check each contour's vertical extent
        for contour in valid_contours:
            _, y, _, h = cv2.boundingRect(contour)
            if h > max_contour_height:
                max_contour_height = h
        
        # Determine if we have a continuous line
        is_continuous_line = (total_valid_area >= self.min_line_continuity_area and 
                             max_contour_height >= self.min_line_height)
        
        # Calculate moments for all valid contours combined
        total_area = 0
        weighted_cx = 0
        weighted_cy = 0
        
        # Find the largest contours (likely the line markings)
        contour_data = []
        for contour in valid_contours:
            area = cv2.contourArea(contour)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                contour_data.append((contour, area, cx, cy))
        
        if not contour_data:
            return None, None, None, None, is_continuous_line
        
        # Sort by area and take the largest contours
        contour_data.sort(key=lambda x: x[1], reverse=True)
        
        # Calculate overall centroid weighted by area
        for contour, area, cx, cy in contour_data:
            weighted_cx += cx * area
            weighted_cy += cy * area
            total_area += area
        
        if total_area > 0:
            centroid_x = int(weighted_cx / total_area)
            centroid_y = int(weighted_cy / total_area)
        else:
            return None, None, None, None, is_continuous_line
        
        # Calculate line angle using the largest contour
        largest_contour = contour_data[0][0]
        
        # Fit a line to estimate line direction
        [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)
        lane_angle = float(np.arctan2(vy[0], vx[0]) * 180 / np.pi)  # Convert to degrees
        
        # Calculate approximate line width (distance between leftmost and rightmost contours)
        if len(contour_data) >= 2:
            x_positions = [data[2] for data in contour_data]  # cx values
            lane_width = max(x_positions) - min(x_positions)
        else:
            lane_width = 0
        
        return centroid_x, centroid_y, lane_angle, lane_width, is_continuous_line

    def image_callback(self, msg):
        """Callback to convert ROS image to OpenCV format and store it."""
        try:
            if self.use_compressed:
                self.image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
    
    def timer_callback(self):
        """Main timer function to process images and detect the line."""
        if self.image is None:
            return

        # Resize image to target size
        frame = cv2.resize(self.image, (self.target_width, self.target_height))

        # Apply perspective transformation to get bird's eye view
        transformed_frame = cv2.warpPerspective(frame, self.perspective_matrix, (self.target_width, self.target_height))

        # Convert to grayscale
        gray = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (self.gaussian_kernel_size, self.gaussian_kernel_size), self.gaussian_sigma)
        
        # Threshold to create binary image (invert so lines are white)
        _, binary_inv = cv2.threshold(blurred, self.grayscale_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Apply morphological operations to clean up noise
        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        morph = cv2.erode(binary_inv, kernel, iterations=self.morph_erode_iterations)
        mask = cv2.dilate(morph, kernel, iterations=self.morph_dilate_iterations)
        
        # Publish thresholded image for debugging
        thresholded_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        self.thresholded_pub.publish(thresholded_msg)
        
        # Calculate line centroid and properties
        centroid_x, centroid_y, lane_angle, lane_width, is_continuous_line = self.calculate_lane_centroid(mask)
        
        # Update line detection status
        self.is_line_detected = is_continuous_line
        
        # Create visualization
        result_frame = transformed_frame.copy()
        
        # Calculate normalized centroid error for IBVS control
        centroid_error = None
        image_center_x = self.target_width / 2.0
        
        if centroid_x is not None and centroid_y is not None:
            # Draw centroid
            cv2.circle(result_frame, (centroid_x, centroid_y), 10, (0, 255, 0), -1)
            cv2.circle(result_frame, (centroid_x, centroid_y), 20, (0, 255, 0), 2)
            
            # Draw target center (image center)
            cv2.circle(result_frame, (int(image_center_x), int(self.target_height/2)), 10, (255, 0, 0), -1)
            cv2.circle(result_frame, (int(image_center_x), int(self.target_height/2)), 20, (255, 0, 0), 2)
            
            # Draw line from centroid to target
            cv2.line(result_frame, (centroid_x, centroid_y), 
                    (int(image_center_x), int(self.target_height/2)), (0, 255, 255), 2)
            
            # Calculate normalized error for IBVS control (-1.0 to 1.0)
            error_x = centroid_x - image_center_x
            centroid_error = error_x / (self.target_width / 2.0)  # Normalize to [-1, 1]
            centroid_error = np.clip(centroid_error, -1.0, 1.0)   # Ensure bounds
            
            # Apply filtering to smooth the output
            if self.last_centroid_error is not None:
                filtered_error = self.filter_alpha * centroid_error + (1 - self.filter_alpha) * self.last_centroid_error
                self.last_centroid_error = filtered_error
                centroid_error = filtered_error
            else:
                self.last_centroid_error = centroid_error
            
            # Display control information on image
            cv2.putText(result_frame, f"Centroid: ({centroid_x}, {centroid_y})", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result_frame, f"Error: {centroid_error:.3f}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if lane_angle is not None:
                cv2.putText(result_frame, f"Angle: {lane_angle:.1f}°", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result_frame, f"Width: {lane_width}", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result_frame, f"Line Detected: {is_continuous_line}", (10, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            self.get_logger().debug(f"IBVS Centroid Error: {centroid_error:.3f}, Line Detected: {is_continuous_line}.")     
        else:
            cv2.putText(result_frame, "No line detected", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(result_frame, f"Line Detected: {is_continuous_line}", (50, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            self.get_logger().debug(f"No line detected, Line Detected: {is_continuous_line}.")

        # Draw contours on the result for visualization
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [cnt for cnt in contours 
                         if self.min_contour_area < cv2.contourArea(cnt) < self.max_contour_area]
        cv2.drawContours(result_frame, valid_contours, -1, (255, 0, 255), 2)

        # Publish detected line visualization
        detected_line_msg = self.bridge.cv2_to_imgmsg(result_frame, encoding='bgr8')
        self.detected_line_pub.publish(detected_line_msg)
        
        # Publish centroid error for IBVS control (only when line is detected)
        if centroid_error is not None and self.is_line_detected:
            centroid_msg = Float32()
            centroid_msg.data = float(centroid_error)
            self.centroid_error_pub.publish(centroid_msg)

    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validates and applies updated node parameters."""
        perspective_changed = False
        
        for param in params:
            if param.name == 'image_topic':
                if not isinstance(param.value, str) or len(param.value.strip()) == 0:
                    return SetParametersResult(
                        successful=False,
                        reason="image_topic must be a non-empty string."
                    )
                self.image_topic = param.value
                self.get_logger().info(f"image_topic updated: {self.image_topic}.")

            elif param.name == 'use_compressed':
                if not isinstance(param.value, bool):
                    return SetParametersResult(
                        successful=False,
                        reason="use_compressed must be a boolean."
                    )
                self.use_compressed = param.value
                self.get_logger().info(f"use_compressed updated: {self.use_compressed}.")

            elif param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="update_rate must be > 0."
                    )
                self.update_rate = float(param.value)
                # Only cancel timer if it exists and is active
                if hasattr(self, 'timer') and self.timer is not None and self.timer_active:
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

            elif param.name in ('target_width', 'target_height'):
                if not isinstance(param.value, int) or param.value <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a positive integer."
                    )
                setattr(self, param.name, param.value)
                perspective_changed = True
                self.get_logger().info(f"{param.name} updated: {param.value} pixels.")

            elif param.name.startswith('perspective_'):
                if not isinstance(param.value, int):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be an integer."
                    )
                # Validate perspective points are within image bounds
                if param.name.endswith('_x'):
                    if param.value < 0 or param.value >= self.target_width:
                        return SetParametersResult(
                            successful=False,
                            reason=f"{param.name} must be between 0 and {self.target_width-1} pixels."
                        )
                elif param.name.endswith('_y'):
                    if param.value < 0 or param.value >= self.target_height:
                        return SetParametersResult(
                            successful=False,
                            reason=f"{param.name} must be between 0 and {self.target_height-1} pixels."
                        )
                setattr(self, param.name, param.value)
                perspective_changed = True
                self.get_logger().info(f"{param.name} updated: {param.value}.")

            elif param.name in ('gaussian_kernel_size', 'morph_kernel_size'):
                if not isinstance(param.value, int) or param.value <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a positive integer."
                    )
                # Ensure kernel size is odd and at least 1
                kernel_size = max(1, param.value)
                if kernel_size % 2 == 0:
                    kernel_size += 1
                setattr(self, param.name, kernel_size)
                self.get_logger().info(f"{param.name} updated: {kernel_size} (adjusted from {param.value}).")

            elif param.name in ('gaussian_sigma', 'morph_erode_iterations', 'morph_dilate_iterations'):
                if not isinstance(param.value, int) or param.value < 0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a non-negative integer."
                    )
                setattr(self, param.name, param.value)
                self.get_logger().info(f"{param.name} updated: {param.value}.")

            elif param.name == 'grayscale_threshold':
                if not isinstance(param.value, int) or param.value < 0 or param.value > 255:
                    return SetParametersResult(
                        successful=False,
                        reason="grayscale_threshold must be between 0 and 255."
                    )
                self.grayscale_threshold = param.value
                self.get_logger().info(f"grayscale_threshold updated: {self.grayscale_threshold}.")

            elif param.name in ('min_contour_area', 'max_contour_area', 'min_line_continuity_area', 'min_line_height'):
                if not isinstance(param.value, int) or param.value <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a positive integer."
                    )
                setattr(self, param.name, param.value)
                self.get_logger().info(f"{param.name} updated: {param.value}.")

            elif param.name == 'filter_alpha':
                if not isinstance(param.value, (int, float)) or param.value < 0.0 or param.value > 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason="filter_alpha must be between 0.0 and 1.0."
                    )
                self.filter_alpha = float(param.value)
                self.get_logger().info(f"filter_alpha updated: {self.filter_alpha}.")

        # Update perspective matrices if any perspective parameters changed
        if perspective_changed:
            self.update_perspective_matrices()

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = LineDectection()
    except Exception as e:
        print(f"[FATAL] LineDectection failed to initialize: {e}", file=sys.stderr)
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