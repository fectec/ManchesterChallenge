#!/usr/bin/env python3

import sys
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from puzzlebot_utils.utils.vision_helpers import get_color_mask

from sensor_msgs.msg import Image

from custom_interfaces.msg import ColorBlobDetection

class ColorBlobDetectionNode(Node):
    def __init__(self):
        super().__init__('color_blob_detection')

        # Declare parameters
        self.declare_parameter('update_rate',   100.0)    # Hz
        self.declare_parameter('debug_view',    True)

        # HSV bounds per color
        self.declare_parameter('hsv_red1_low', [0, 100, 100])
        self.declare_parameter('hsv_red1_high', [10, 255, 255])
        self.declare_parameter('hsv_red2_low', [160, 100, 100])
        self.declare_parameter('hsv_red2_high', [180, 255, 255])
        self.declare_parameter('hsv_green_low', [40, 70, 70])
        self.declare_parameter('hsv_green_high', [80, 255, 255])
        self.declare_parameter('hsv_yellow_low', [20, 150, 150])
        self.declare_parameter('hsv_yellow_high', [35, 255, 255])

        # Blob detector parameters
        self.declare_parameter('blob_min_threshold', 30)
        self.declare_parameter('blob_max_threshold', 255)
        self.declare_parameter('blob_min_area', 30)
        self.declare_parameter('blob_max_area', 10000000)
        self.declare_parameter('blob_min_convexity', 0.1)
        self.declare_parameter('blob_max_convexity', 1.0)
        self.declare_parameter('blob_min_circularity', 0.5)
        self.declare_parameter('blob_max_circularity', 1.0)
        self.declare_parameter('blob_min_inertia_ratio', 0.5)
        self.declare_parameter('blob_max_inertia_ratio', 1.0)

        # Image processing parameters
        self.declare_parameter('gaussian_kernel_size', [9, 9])
        self.declare_parameter('gaussian_sigma', 2)
        self.declare_parameter('grayscale_threshold', 5)
        self.declare_parameter('morph_kernel_size', [3, 3])
        self.declare_parameter('morph_erode_iterations', 8)
        self.declare_parameter('morph_dilate_iterations', 8)

        # Retrieve parameters
        self.update_rate                = self.get_parameter('update_rate').value
        self.debug_view                 = self.get_parameter('debug_view').value
        
        self.hsv_red1_low               = self.get_parameter('hsv_red1_low').value
        self.hsv_red1_high              = self.get_parameter('hsv_red1_high').value
        self.hsv_red2_low               = self.get_parameter('hsv_red2_low').value
        self.hsv_red2_high              = self.get_parameter('hsv_red2_high').value
        self.hsv_green_low              = self.get_parameter('hsv_green_low').value
        self.hsv_green_high             = self.get_parameter('hsv_green_high').value
        self.hsv_yellow_low             = self.get_parameter('hsv_yellow_low').value
        self.hsv_yellow_high            = self.get_parameter('hsv_yellow_high').value

        self.blob_min_threshold         = self.get_parameter('blob_min_threshold').value
        self.blob_max_threshold         = self.get_parameter('blob_max_threshold').value
        self.blob_min_area              = self.get_parameter('blob_min_area').value
        self.blob_max_area              = self.get_parameter('blob_max_area').value
        self.blob_min_convexity         = self.get_parameter('blob_min_convexity').value
        self.blob_max_convexity         = self.get_parameter('blob_max_convexity').value
        self.blob_min_circularity       = self.get_parameter('blob_min_circularity').value
        self.blob_max_circularity       = self.get_parameter('blob_max_circularity').value
        self.blob_min_inertia_ratio     = self.get_parameter('blob_min_inertia_ratio').value
        self.blob_max_inertia_ratio     = self.get_parameter('blob_max_inertia_ratio').value

        self.gaussian_kernel_size       = self.get_parameter('gaussian_kernel_size').value
        self.gaussian_sigma             = self.get_parameter('gaussian_sigma').value
        self.grayscale_threshold        = self.get_parameter('grayscale_threshold').value
        self.morph_kernel_size          = self.get_parameter('morph_kernel_size').value
        self.morph_erode_iterations     = self.get_parameter('morph_erode_iterations').value
        self.morph_dilate_iterations    = self.get_parameter('morph_dilate_iterations').value
        
        # Timer for periodic processing
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate',                Parameter.Type.DOUBLE,  self.update_rate),
            Parameter('debug_view',                 Parameter.Type.BOOL,    self.debug_view),
            Parameter('hsv_red1_low',               Parameter.Type.INTEGER_ARRAY, self.hsv_red1_low),
            Parameter('hsv_red1_high',              Parameter.Type.INTEGER_ARRAY, self.hsv_red1_high),
            Parameter('hsv_red2_low',               Parameter.Type.INTEGER_ARRAY, self.hsv_red2_low),
            Parameter('hsv_red2_high',              Parameter.Type.INTEGER_ARRAY, self.hsv_red2_high),
            Parameter('hsv_green_low',              Parameter.Type.INTEGER_ARRAY, self.hsv_green_low),
            Parameter('hsv_green_high',             Parameter.Type.INTEGER_ARRAY, self.hsv_green_high),
            Parameter('hsv_yellow_low',             Parameter.Type.INTEGER_ARRAY, self.hsv_yellow_low),
            Parameter('hsv_yellow_high',            Parameter.Type.INTEGER_ARRAY, self.hsv_yellow_high),
            Parameter('blob_min_threshold',         Parameter.Type.INTEGER, self.blob_min_threshold),
            Parameter('blob_max_threshold',         Parameter.Type.INTEGER, self.blob_max_threshold),
            Parameter('blob_min_area',              Parameter.Type.INTEGER, self.blob_min_area),
            Parameter('blob_max_area',              Parameter.Type.INTEGER, self.blob_max_area),
            Parameter('blob_min_convexity',         Parameter.Type.DOUBLE,  self.blob_min_convexity),
            Parameter('blob_max_convexity',         Parameter.Type.DOUBLE,  self.blob_max_convexity),
            Parameter('blob_min_circularity',       Parameter.Type.DOUBLE,  self.blob_min_circularity),
            Parameter('blob_max_circularity',       Parameter.Type.DOUBLE,  self.blob_max_circularity),
            Parameter('blob_min_inertia_ratio',     Parameter.Type.DOUBLE,  self.blob_min_inertia_ratio),
            Parameter('blob_max_inertia_ratio',     Parameter.Type.DOUBLE,  self.blob_max_inertia_ratio),
            Parameter('gaussian_kernel_size',       Parameter.Type.INTEGER_ARRAY, self.gaussian_kernel_size),
            Parameter('gaussian_sigma',             Parameter.Type.INTEGER, self.gaussian_sigma),
            Parameter('grayscale_threshold',        Parameter.Type.INTEGER, self.grayscale_threshold),
            Parameter('morph_kernel_size',          Parameter.Type.INTEGER_ARRAY, self.morph_kernel_size),
            Parameter('morph_erode_iterations',     Parameter.Type.INTEGER, self.morph_erode_iterations),
            Parameter('morph_dilate_iterations',    Parameter.Type.INTEGER, self.morph_dilate_iterations),
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")

        # Initialize variables
        self.image = None
        self.bridge = CvBridge()
        
        # Color blob detection publisher
        self.color_blob_detection_pub = self.create_publisher(
            ColorBlobDetection, 
            'puzzlebot_real/color_blob_detection', 
            10
        )
        
        # Raw camera images subscriber
        self.create_subscription(
            Image,
            'video_source/raw',
            self.image_callback,
            qos.qos_profile_sensor_data
        )

        # Define HSV ranges for each color
        self.hsv_ranges = {
            ColorBlobDetection.COLOR_GREEN: (
                np.array(self.hsv_green_low, dtype=np.uint8),
                np.array(self.hsv_green_high, dtype=np.uint8)
            ),
            ColorBlobDetection.COLOR_YELLOW: (
                np.array(self.hsv_yellow_low, dtype=np.uint8),
                np.array(self.hsv_yellow_high, dtype=np.uint8)
            ),
            ColorBlobDetection.COLOR_RED: [
                (
                    np.array(self.hsv_red1_low, dtype=np.uint8),
                    np.array(self.hsv_red1_high, dtype=np.uint8)
                ),
                (
                    np.array(self.hsv_red2_low, dtype=np.uint8),
                    np.array(self.hsv_red2_high, dtype=np.uint8)
                )
            ]
        }

        # Color name mapping for debug visualization
        self.color_names = {
            ColorBlobDetection.COLOR_GREEN: "GREEN",
            ColorBlobDetection.COLOR_YELLOW: "YELLOW",
            ColorBlobDetection.COLOR_RED: "RED",
            ColorBlobDetection.COLOR_NONE: "NONE"
        }

        # BGR color values for visualization
        self.bgr_colors = {
            ColorBlobDetection.COLOR_GREEN: (0, 255, 0),
            ColorBlobDetection.COLOR_YELLOW: (0, 255, 255),
            ColorBlobDetection.COLOR_RED: (0, 0, 255),
            ColorBlobDetection.COLOR_NONE: (255, 255, 255)
        }

        # Create the blob detector object with configured parameters
        self.configure_blob_detector()

        self.get_logger().info("ColorBlobDetection Start.")

    def image_callback(self, msg):
        # Callback to receive image from the camera and convert to CV2 format
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}.")
            return
    
    def timer_callback(self):
        # Check if image has been received
        if self.image is None:
            return

        # Reduce noise with Gaussian blur
        blurred_image = cv.GaussianBlur(
            self.image,
            tuple(self.gaussian_kernel_size),
            self.gaussian_sigma
        )

        # Convert from BGR to HSV color space
        hsv_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2HSV)

        # Create an empty message to publish our detection results
        detection_msg = ColorBlobDetection()
        detection_msg.color = ColorBlobDetection.COLOR_NONE
        
        # Dictionary to store keypoints for each color
        all_keypoints = {}
        
        # Track the largest blob and its color
        largest_blob_size = 0
        largest_blob_color = ColorBlobDetection.COLOR_NONE
        
        # Detect blobs for each color
        for color in [ColorBlobDetection.COLOR_GREEN, 
                    ColorBlobDetection.COLOR_YELLOW, 
                    ColorBlobDetection.COLOR_RED]:
            
            # Get the mask for this color
            mask = get_color_mask(hsv_image, self.hsv_ranges[color])
            
            # Apply mask to extract colored regions from original image
            extracted_image = cv.bitwise_and(self.image, self.image, mask=mask)
            
            # Convert extracted regions to grayscale
            gray_image = cv.cvtColor(extracted_image, cv.COLOR_BGR2GRAY)
            
            # Threshold grayscale image to binary image
            _, binary_image = cv.threshold(gray_image, self.grayscale_threshold, 255, cv.THRESH_BINARY)
            
            # Apply morphological operations to clean noise
            kernel = np.ones(tuple(self.morph_kernel_size), np.uint8)
            cleaned_image = cv.erode(binary_image, kernel, iterations=self.morph_erode_iterations)
            cleaned_image = cv.dilate(cleaned_image, kernel, iterations=self.morph_dilate_iterations)
            
            # Detect blobs
            keypoints = self.blob_detector.detect(cleaned_image)
            
            # Store keypoints for this color
            all_keypoints[color] = keypoints
            
            # If we found blobs of this color, store them
            # We'll use the largest blob across all colors to determine the detection
            if keypoints:
                self.get_logger().debug(f"Found {len(keypoints)} blob(s) of color {self.color_names[color]}.")
        
        # Create combined visualization image for debug view
        if self.debug_view:
            # Start with the original image
            view_blobs = self.image.copy()
            
            # Draw detected blobs for each color
            for color, keypoints in all_keypoints.items():
                if keypoints:
                    # Use the appropriate BGR color for visualization
                    bgr_color = self.bgr_colors[color]
                    
                    # Draw the keypoints for this color
                    view_blobs = cv.drawKeypoints(
                        view_blobs, 
                        keypoints, 
                        np.array([]), 
                        bgr_color, 
                        cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
                    )
                    
                    # Add text label for this color
                    if keypoints:
                        # Get the position of the first keypoint for the text label
                        x = int(keypoints[0].pt[0])
                        y = int(keypoints[0].pt[1])
                        cv.putText(
                            view_blobs,
                            self.color_names[color],
                            (x + 10, y),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            bgr_color,
                            2
                        )
            
            # Add text showing which color is being published
            if detection_msg.color != ColorBlobDetection.COLOR_NONE:
                cv.putText(
                    view_blobs,
                    f"DETECTED: {self.color_names[detection_msg.color]}",
                    (10, 30),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    self.bgr_colors[detection_msg.color],
                    2
                )
            else:
                cv.putText(
                    view_blobs,
                    "NO DETECTION",
                    (10, 30),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),  # White color
                    2
                )
                
            cv.imshow('Color Blob Detection Debug', view_blobs)
            cv.waitKey(1)  # Process events, wait 1 ms
        
        # Now determine which color to publish based on blob sizes
        for color, keypoints in all_keypoints.items():
            for kp in keypoints:
                # Size of blob is stored in the keypoint's size attribute
                if kp.size > largest_blob_size:
                    largest_blob_size = kp.size
                    largest_blob_color = color
        
        # Set the detected color to the one with the largest blob
        if largest_blob_color != ColorBlobDetection.COLOR_NONE:
            detection_msg.color = largest_blob_color
            self.get_logger().debug(f"Detected color: {self.color_names[detection_msg.color]}")
        
        # Publish the detection message
        self.color_blob_detection_pub.publish(detection_msg)

    def parameter_callback(self, params):
        blob_params_changed = False

        for param in params:
            name = param.name
            value = param.value

            if name == 'update_rate':
                if not isinstance(value, (int, float)) or value <= 0.0:
                    return SetParametersResult(successful=False, reason="update_rate must be > 0.")
                self.update_rate = float(value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
                self.get_logger().info(f"Update rate changed to {self.update_rate} Hz.")

            elif name == 'debug_view':
                if not isinstance(value, bool):
                    return SetParametersResult(successful=False, reason="debug_view must be a boolean.")
                self.debug_view = value
                self.get_logger().info(f"Debug view set to {self.debug_view}.")

            elif name.startswith('hsv_'):
                if not (isinstance(value, list) and len(value) == 3 and all(isinstance(v, int) and v >= 0 for v in value)):
                    return SetParametersResult(successful=False, reason=f"{name} must be a list of 3 non-negative integers.")
                setattr(self, name, value)
                self.get_logger().info(f"{name} updated to {value}.")

            elif name in (
                'blob_min_threshold', 'blob_max_threshold',
                'blob_min_area', 'blob_max_area',
                'grayscale_threshold',
                'morph_erode_iterations', 'morph_dilate_iterations',
                'gaussian_sigma'
            ):
                if not isinstance(value, int) or value < 0:
                    return SetParametersResult(successful=False, reason=f"{name} must be a non-negative integer.")
                setattr(self, name, value)
                self.get_logger().info(f"{name} updated to {value}")
                blob_params_changed = True

            elif name in (
                'blob_min_convexity', 'blob_max_convexity',
                'blob_min_circularity', 'blob_max_circularity',
                'blob_min_inertia_ratio', 'blob_max_inertia_ratio'
            ):
                if not isinstance(value, (int, float)) or not (0.0 <= value <= 1.0):
                    return SetParametersResult(successful=False, reason=f"{name} must be between 0.0 and 1.0.")
                setattr(self, name, float(value))
                self.get_logger().info(f"{name} updated to {value}")
                blob_params_changed = True

            elif name in ('gaussian_kernel_size', 'morph_kernel_size'):
                if not (isinstance(value, list) and len(value) == 2 and all(isinstance(v, int) and v > 0 for v in value)):
                    return SetParametersResult(successful=False, reason=f"{name} must be a list of 2 positive integers.")
                setattr(self, name, value)
                self.get_logger().info(f"{name} updated to {value}.")

        if blob_params_changed:
            self.configure_blob_detector()

        return SetParametersResult(successful=True)

    def configure_blob_detector(self):
        params = cv.SimpleBlobDetector_Params()
        params.minThreshold = self.blob_min_threshold
        params.maxThreshold = self.blob_max_threshold
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = self.blob_min_area
        params.maxArea = self.blob_max_area
        params.filterByConvexity = True
        params.minConvexity = self.blob_min_convexity
        params.maxConvexity = self.blob_max_convexity
        params.filterByCircularity = True
        params.minCircularity = self.blob_min_circularity
        params.maxCircularity = self.blob_max_circularity
        params.filterByInertia = True
        params.minInertiaRatio = self.blob_min_inertia_ratio
        params.maxInertiaRatio = self.blob_max_inertia_ratio
        self.blob_detector = cv.SimpleBlobDetector_create(params)

    def destroy_node(self):
        if self.debug_view:
            cv.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = ColorBlobDetectionNode()
    except Exception as e:
        print(f"[FATAL] ColorBlobDetecion failed to initialize: {e}.", file=sys.stderr)
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