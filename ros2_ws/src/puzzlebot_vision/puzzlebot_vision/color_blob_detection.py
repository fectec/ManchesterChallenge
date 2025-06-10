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

from sensor_msgs.msg import Image, CompressedImage

from custom_interfaces.msg import ColorBlobDetection

class ColorBlobDetectionNode(Node):
    """
    Performs color blob detection from camera images.
    Publishes the color of the largest detected blob to a custom message topic.
    Supported colors: RED, GREEN, YELLOW (with double HSV range for RED).
    """
    
    def __init__(self):
        super().__init__('color_blob_detection_node')

        # Declare parameters
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('use_compressed', False)

        self.declare_parameter('update_rate', 60.0)    # Hz

        # GREEN HSV ranges
        self.declare_parameter('hsv_green_h_low', 40)
        self.declare_parameter('hsv_green_s_low', 50)
        self.declare_parameter('hsv_green_v_low', 50)
        self.declare_parameter('hsv_green_h_high', 80)
        self.declare_parameter('hsv_green_s_high', 255)
        self.declare_parameter('hsv_green_v_high', 255)

        # YELLOW HSV ranges
        self.declare_parameter('hsv_yellow_h_low', 20)        
        self.declare_parameter('hsv_yellow_s_low', 100)
        self.declare_parameter('hsv_yellow_v_low', 100)
        self.declare_parameter('hsv_yellow_h_high', 30)
        self.declare_parameter('hsv_yellow_s_high', 255)
        self.declare_parameter('hsv_yellow_v_high', 255)

        # RED HSV ranges (two ranges for red wraparound)
        self.declare_parameter('hsv_red1_h_low', 0)
        self.declare_parameter('hsv_red1_s_low', 120)
        self.declare_parameter('hsv_red1_v_low', 70)
        self.declare_parameter('hsv_red1_h_high', 10)
        self.declare_parameter('hsv_red1_s_high', 255)
        self.declare_parameter('hsv_red1_v_high', 255)

        self.declare_parameter('hsv_red2_h_low', 170)
        self.declare_parameter('hsv_red2_s_low', 120)
        self.declare_parameter('hsv_red2_v_low', 70)
        self.declare_parameter('hsv_red2_h_high', 180)
        self.declare_parameter('hsv_red2_s_high', 255)
        self.declare_parameter('hsv_red2_v_high', 255)

        # Blob detector parameters
        self.declare_parameter('blob_min_threshold', 10)
        self.declare_parameter('blob_max_threshold', 250)
        self.declare_parameter('blob_min_area', 200)
        self.declare_parameter('blob_max_area', 10000000)
        self.declare_parameter('blob_min_convexity', 0.5)
        self.declare_parameter('blob_max_convexity', 1.0)
        self.declare_parameter('blob_min_circularity', 0.8)
        self.declare_parameter('blob_max_circularity', 1.0)
        self.declare_parameter('blob_min_inertia_ratio', 0.2)
        self.declare_parameter('blob_max_inertia_ratio', 1.0)

        # Image processing parameters
        self.declare_parameter('gaussian_kernel_size_width', 9)
        self.declare_parameter('gaussian_kernel_size_height', 9)
        self.declare_parameter('gaussian_sigma', 5)
        self.declare_parameter('grayscale_threshold', 10)
        self.declare_parameter('morph_kernel_size_width', 3)
        self.declare_parameter('morph_kernel_size_height', 3)
        self.declare_parameter('morph_erode_iterations', 10)
        self.declare_parameter('morph_dilate_iterations', 8)

        # Retrieve parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
        self.update_rate = self.get_parameter('update_rate').value

        self.hsv_green_h_low = self.get_parameter('hsv_green_h_low').value
        self.hsv_green_s_low = self.get_parameter('hsv_green_s_low').value
        self.hsv_green_v_low = self.get_parameter('hsv_green_v_low').value
        self.hsv_green_h_high = self.get_parameter('hsv_green_h_high').value
        self.hsv_green_s_high = self.get_parameter('hsv_green_s_high').value
        self.hsv_green_v_high = self.get_parameter('hsv_green_v_high').value

        self.hsv_yellow_h_low = self.get_parameter('hsv_yellow_h_low').value
        self.hsv_yellow_s_low = self.get_parameter('hsv_yellow_s_low').value
        self.hsv_yellow_v_low = self.get_parameter('hsv_yellow_v_low').value
        self.hsv_yellow_h_high = self.get_parameter('hsv_yellow_h_high').value
        self.hsv_yellow_s_high = self.get_parameter('hsv_yellow_s_high').value
        self.hsv_yellow_v_high = self.get_parameter('hsv_yellow_v_high').value

        self.hsv_red1_h_low = self.get_parameter('hsv_red1_h_low').value
        self.hsv_red1_s_low = self.get_parameter('hsv_red1_s_low').value
        self.hsv_red1_v_low = self.get_parameter('hsv_red1_v_low').value
        self.hsv_red1_h_high = self.get_parameter('hsv_red1_h_high').value
        self.hsv_red1_s_high = self.get_parameter('hsv_red1_s_high').value
        self.hsv_red1_v_high = self.get_parameter('hsv_red1_v_high').value

        self.hsv_red2_h_low = self.get_parameter('hsv_red2_h_low').value
        self.hsv_red2_s_low = self.get_parameter('hsv_red2_s_low').value
        self.hsv_red2_v_low = self.get_parameter('hsv_red2_v_low').value
        self.hsv_red2_h_high = self.get_parameter('hsv_red2_h_high').value
        self.hsv_red2_s_high = self.get_parameter('hsv_red2_s_high').value
        self.hsv_red2_v_high = self.get_parameter('hsv_red2_v_high').value

        self.blob_min_threshold = self.get_parameter('blob_min_threshold').value
        self.blob_max_threshold = self.get_parameter('blob_max_threshold').value
        self.blob_min_area = self.get_parameter('blob_min_area').value
        self.blob_max_area = self.get_parameter('blob_max_area').value
        self.blob_min_convexity = self.get_parameter('blob_min_convexity').value
        self.blob_max_convexity = self.get_parameter('blob_max_convexity').value
        self.blob_min_circularity = self.get_parameter('blob_min_circularity').value
        self.blob_max_circularity = self.get_parameter('blob_max_circularity').value
        self.blob_min_inertia_ratio = self.get_parameter('blob_min_inertia_ratio').value
        self.blob_max_inertia_ratio = self.get_parameter('blob_max_inertia_ratio').value

        self.gaussian_kernel_size_width = self.get_parameter('gaussian_kernel_size_width').value
        self.gaussian_kernel_size_height = self.get_parameter('gaussian_kernel_size_height').value
        self.gaussian_sigma = self.get_parameter('gaussian_sigma').value
        self.grayscale_threshold = self.get_parameter('grayscale_threshold').value
        self.morph_kernel_size_width = self.get_parameter('morph_kernel_size_width').value
        self.morph_kernel_size_height = self.get_parameter('morph_kernel_size_height').value
        self.morph_erode_iterations = self.get_parameter('morph_erode_iterations').value
        self.morph_dilate_iterations = self.get_parameter('morph_dilate_iterations').value

        # Timer for periodic processing
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

        # Register the parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Validate initial parameters
        init_params = [
            Parameter('image_topic', Parameter.Type.STRING, self.image_topic),
            Parameter('use_compressed', Parameter.Type.BOOL, self.use_compressed),
            Parameter('update_rate', Parameter.Type.DOUBLE, self.update_rate),
            Parameter('hsv_green_h_low', Parameter.Type.INTEGER, self.hsv_green_h_low),
            Parameter('hsv_green_h_high', Parameter.Type.INTEGER, self.hsv_green_h_high),
            Parameter('hsv_green_s_low', Parameter.Type.INTEGER, self.hsv_green_s_low),
            Parameter('hsv_green_s_high', Parameter.Type.INTEGER, self.hsv_green_s_high),
            Parameter('hsv_green_v_low', Parameter.Type.INTEGER, self.hsv_green_v_low),
            Parameter('hsv_green_v_high', Parameter.Type.INTEGER, self.hsv_green_v_high),
            Parameter('hsv_yellow_h_low', Parameter.Type.INTEGER, self.hsv_yellow_h_low),
            Parameter('hsv_yellow_h_high', Parameter.Type.INTEGER, self.hsv_yellow_h_high),
            Parameter('hsv_yellow_s_low', Parameter.Type.INTEGER, self.hsv_yellow_s_low),
            Parameter('hsv_yellow_s_high', Parameter.Type.INTEGER, self.hsv_yellow_s_high),
            Parameter('hsv_yellow_v_low', Parameter.Type.INTEGER, self.hsv_yellow_v_low),
            Parameter('hsv_yellow_v_high', Parameter.Type.INTEGER, self.hsv_yellow_v_high),
            Parameter('hsv_red1_h_low', Parameter.Type.INTEGER, self.hsv_red1_h_low),
            Parameter('hsv_red1_h_high', Parameter.Type.INTEGER, self.hsv_red1_h_high),
            Parameter('hsv_red1_s_low', Parameter.Type.INTEGER, self.hsv_red1_s_low),
            Parameter('hsv_red1_s_high', Parameter.Type.INTEGER, self.hsv_red1_s_high),
            Parameter('hsv_red1_v_low', Parameter.Type.INTEGER, self.hsv_red1_v_low),
            Parameter('hsv_red1_v_high', Parameter.Type.INTEGER, self.hsv_red1_v_high),
            Parameter('hsv_red2_h_low', Parameter.Type.INTEGER, self.hsv_red2_h_low),
            Parameter('hsv_red2_h_high', Parameter.Type.INTEGER, self.hsv_red2_h_high),
            Parameter('hsv_red2_s_low', Parameter.Type.INTEGER, self.hsv_red2_s_low),
            Parameter('hsv_red2_s_high', Parameter.Type.INTEGER, self.hsv_red2_s_high),
            Parameter('hsv_red2_v_low', Parameter.Type.INTEGER, self.hsv_red2_v_low),
            Parameter('hsv_red2_v_high', Parameter.Type.INTEGER, self.hsv_red2_v_high),
            Parameter('blob_min_threshold', Parameter.Type.INTEGER, self.blob_min_threshold),
            Parameter('blob_max_threshold', Parameter.Type.INTEGER, self.blob_max_threshold),
            Parameter('blob_min_area', Parameter.Type.INTEGER, self.blob_min_area),
            Parameter('blob_max_area', Parameter.Type.INTEGER, self.blob_max_area),
            Parameter('blob_min_convexity', Parameter.Type.DOUBLE, self.blob_min_convexity),
            Parameter('blob_max_convexity', Parameter.Type.DOUBLE, self.blob_max_convexity),
            Parameter('blob_min_circularity', Parameter.Type.DOUBLE, self.blob_min_circularity),
            Parameter('blob_max_circularity', Parameter.Type.DOUBLE, self.blob_max_circularity),
            Parameter('blob_min_inertia_ratio', Parameter.Type.DOUBLE, self.blob_min_inertia_ratio),
            Parameter('blob_max_inertia_ratio', Parameter.Type.DOUBLE, self.blob_max_inertia_ratio),
            Parameter('gaussian_kernel_size_width', Parameter.Type.INTEGER, self.gaussian_kernel_size_width),
            Parameter('gaussian_kernel_size_height', Parameter.Type.INTEGER, self.gaussian_kernel_size_height),
            Parameter('gaussian_sigma', Parameter.Type.INTEGER, self.gaussian_sigma),
            Parameter('grayscale_threshold', Parameter.Type.INTEGER, self.grayscale_threshold),
            Parameter('morph_kernel_size_width', Parameter.Type.INTEGER, self.morph_kernel_size_width),
            Parameter('morph_kernel_size_height', Parameter.Type.INTEGER, self.morph_kernel_size_height),
            Parameter('morph_erode_iterations', Parameter.Type.INTEGER, self.morph_erode_iterations),
            Parameter('morph_dilate_iterations', Parameter.Type.INTEGER, self.morph_dilate_iterations),
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
            'color_blob_detection',  
            qos.qos_profile_sensor_data  
        )

        # Debug image publisher
        self.color_blob_detection_image_pub = self.create_publisher(
            Image, 
            'color_blob_detection_image', 
            qos.qos_profile_sensor_data
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

        # Update HSV ranges dictionary from parameters
        self.update_hsv_ranges()

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
        }

        # Create the blob detector object with configured parameters
        self.configure_blob_detector()

        self.get_logger().info("ColorBlobDetection Start.")
        
    def update_hsv_ranges(self):
        """Update HSV ranges dictionary from individual parameters."""
        self.hsv_ranges = {
            ColorBlobDetection.COLOR_GREEN: (
                np.array([self.hsv_green_h_low, self.hsv_green_s_low, self.hsv_green_v_low], dtype=np.uint8),
                np.array([self.hsv_green_h_high, self.hsv_green_s_high, self.hsv_green_v_high], dtype=np.uint8)
            ),
            ColorBlobDetection.COLOR_YELLOW: (
                np.array([self.hsv_yellow_h_low, self.hsv_yellow_s_low, self.hsv_yellow_v_low], dtype=np.uint8),
                np.array([self.hsv_yellow_h_high, self.hsv_yellow_s_high, self.hsv_yellow_v_high], dtype=np.uint8)
            ),
            ColorBlobDetection.COLOR_RED: [
                (
                    np.array([self.hsv_red1_h_low, self.hsv_red1_s_low, self.hsv_red1_v_low], dtype=np.uint8),
                    np.array([self.hsv_red1_h_high, self.hsv_red1_s_high, self.hsv_red1_v_high], dtype=np.uint8)
                ),
                (
                    np.array([self.hsv_red2_h_low, self.hsv_red2_s_low, self.hsv_red2_v_low], dtype=np.uint8),
                    np.array([self.hsv_red2_h_high, self.hsv_red2_s_high, self.hsv_red2_v_high], dtype=np.uint8)
                )
            ]
        }

    def configure_blob_detector(self) -> None:
        """Configure OpenCV SimpleBlobDetector with loaded parameters."""
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

    def image_callback(self, msg):
        """Callback to convert ROS image to OpenCV format and store it."""
        try:
            if self.use_compressed:
                self.image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            
    def timer_callback(self) -> None:
        """Main timer function to process images and detect color blobs."""
        # Check if image has been received
        if self.image is None:
            return

        # Reduce noise with Gaussian blur
        blurred_image = cv.GaussianBlur(
            self.image,
            (self.gaussian_kernel_size_width, self.gaussian_kernel_size_height),
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

        # Binary images dictionary for each color
        binary_images = {}
        
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
            kernel = np.ones((self.morph_kernel_size_width, self.morph_kernel_size_height), np.uint8)
            cleaned_image = cv.erode(binary_image, kernel, iterations=self.morph_erode_iterations)
            cleaned_image = cv.dilate(cleaned_image, kernel, iterations=self.morph_dilate_iterations)
            
            # Store the binary image for this color
            binary_images[color] = cleaned_image.copy()

            # Detect blobs
            keypoints = self.blob_detector.detect(cleaned_image)
            
            # Store keypoints for this color
            all_keypoints[color] = keypoints
            
            # If we found blobs of this color, store them
            # We'll use the largest blob across all colors to determine the detection
            if keypoints:
                self.get_logger().debug(f"Found {len(keypoints)} blob(s) of color {self.color_names[color]}.")
        
        # Create debug visualization image
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

            color_codes = {
                'RED': '\033[91m',    
                'GREEN': '\033[92m',   
                'YELLOW': '\033[93m',  
                'ENDC': '\033[0m'    
            }

            color_str = self.color_names[detection_msg.color]
            color_code = color_codes.get(color_str.upper(), '')
            end_code = color_codes['ENDC']

            self.get_logger().debug(f"Detected color: {color_code}{color_str}{end_code}.")

        # Publish debug visualization image
        debug_image_msg = self.bridge.cv2_to_imgmsg(view_blobs, encoding='bgr8')
        self.color_blob_detection_image_pub.publish(debug_image_msg)

        # Publish the detection message
        self.color_blob_detection_pub.publish(detection_msg)

    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validates and applies updated node parameters."""
        blob_params_changed = False

        for param in params:
            name = param.name
            value = param.value

            if param.name == 'image_topic':
                if not isinstance(param.value, str) or len(param.value.strip()) == 0:
                    return SetParametersResult(
                        successful=False,
                        reason="image_topic must be a non-empty string."
                    )
                self.image_topic = param.value
                self.get_logger().info(f"image_topic updated: {self.image_topic}. Note: Restart node to apply topic change.")

            elif param.name == 'use_compressed':
                if not isinstance(param.value, bool):
                    return SetParametersResult(
                        successful=False,
                        reason="use_compressed must be a boolean."
                    )
                self.use_compressed = param.value
                self.get_logger().info(f"use_compressed updated: {self.use_compressed}. Note: Restart node to apply compression change.")

            elif name == 'update_rate':
                if not isinstance(value, (int, float)) or value <= 0.0:
                    return SetParametersResult(successful=False, reason="update_rate must be > 0.")
                self.update_rate = float(value)
                # Restart timer
                if hasattr(self, 'timer') and self.timer is not None:
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

            elif name.startswith('hsv_') and ('_low' in name or '_high' in name):
                if not isinstance(value, int) or not (0 <= value <= 255):
                    return SetParametersResult(successful=False, reason=f"{name} must be an integer between 0 and 255.")
                setattr(self, name, value)
                self.update_hsv_ranges()
                self.get_logger().info(f"{name} updated: {value}.")

            elif name in (
                'blob_min_threshold', 'blob_max_threshold',
                'blob_min_area', 'blob_max_area',
                'grayscale_threshold',
                'morph_erode_iterations', 'morph_dilate_iterations',
                'gaussian_sigma',
                'gaussian_kernel_size_width', 'gaussian_kernel_size_height',
                'morph_kernel_size_width', 'morph_kernel_size_height'
            ):
                if not isinstance(value, int) or value < 0:
                    return SetParametersResult(successful=False, reason=f"{name} must be a non-negative integer.")
                setattr(self, name, value)
                self.get_logger().info(f"{name} updated: {value}.")
                if 'blob_' in name:
                    blob_params_changed = True

            elif name in (
                'blob_min_convexity', 'blob_max_convexity',
                'blob_min_circularity', 'blob_max_circularity',
                'blob_min_inertia_ratio', 'blob_max_inertia_ratio'
            ):
                if not isinstance(value, (int, float)) or not (0.0 <= value <= 1.0):
                    return SetParametersResult(successful=False, reason=f"{name} must be between 0.0 and 1.0.")
                setattr(self, name, float(value))
                self.get_logger().info(f"{name} updated: {value}.")
                blob_params_changed = True

        if blob_params_changed:
            self.configure_blob_detector()

        return SetParametersResult(successful=True)

    def destroy_node(self):
        cv.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = ColorBlobDetectionNode()
    except Exception as e:
        print(f"[FATAL] ColorBlobDetectionNode failed to initialize: {e}", file=sys.stderr)
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