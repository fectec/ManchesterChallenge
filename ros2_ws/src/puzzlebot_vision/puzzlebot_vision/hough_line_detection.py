#!/usr/bin/env python3

import sys
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from rclpy import qos

from sensor_msgs.msg import Image

from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

class LineDetection(Node):
    def __init__(self):
        super().__init__('line_detection')

        # Create parameter descriptors
        int_range = IntegerRange(from_value=0, to_value=640, step=1)
        int_descriptor = ParameterDescriptor(integer_range=[int_range])
        
        float_range = FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)
        float_descriptor = ParameterDescriptor(floating_point_range=[float_range])
        
        # Declare parameters
        self.declare_parameter('update_rate', 100.0)                     # Hz
        self.declare_parameter('target_width', 640)                      # pixels
        self.declare_parameter('target_height', 480)                     # pixels
        self.declare_parameter('roi_percent', 50)                        # ROI starts at 50% down the image
        
        # Trapezoid points (relative to ROI, not full image)
        self.declare_parameter('trapezoid_top_left_x', 220, int_descriptor)     
        self.declare_parameter('trapezoid_top_left_y', 0, int_descriptor)       
        self.declare_parameter('trapezoid_top_right_x', 420, int_descriptor)    
        self.declare_parameter('trapezoid_top_right_y', 0, int_descriptor)   
        self.declare_parameter('trapezoid_bottom_right_x', 580, int_descriptor) 
        self.declare_parameter('trapezoid_bottom_right_y', 240, int_descriptor) 
        self.declare_parameter('trapezoid_bottom_left_x', 50, int_descriptor)   
        self.declare_parameter('trapezoid_bottom_left_y', 240, int_descriptor) 
        
        # Gaussian blur parameters
        self.declare_parameter('gaussian_kernel_size', [9, 9])
        self.declare_parameter('gaussian_sigma', 2)
        
        # Thresholding and morphology parameters
        self.declare_parameter('grayscale_threshold', 115)
        self.declare_parameter('morph_kernel_size', [3, 3])
        self.declare_parameter('morph_erode_iterations', 1)
        self.declare_parameter('morph_dilate_iterations', 3)
        
        # Canny edge detection parameters
        self.declare_parameter('canny_threshold1', 100)
        self.declare_parameter('canny_threshold2', 150)
        
        # Side cropping parameters
        self.declare_parameter('crop_side_percent', 0.05, float_descriptor)
        self.declare_parameter('rectangle_thickness', -1)
        
        # Hough transform parameters
        self.declare_parameter('hough_rho', 1)
        self.declare_parameter('hough_theta_fraction', 180)  # We divide pi by this
        self.declare_parameter('hough_threshold', 10)
        self.declare_parameter('hough_min_line_length', 5)
        self.declare_parameter('hough_max_line_gap', 10)

        # Retrieve parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.target_width = self.get_parameter('target_width').value
        self.target_height = self.get_parameter('target_height').value
        self.roi_percent = self.get_parameter('roi_percent').value

        # Trapezoid parameters
        self.trap_tl_x = self.get_parameter('trapezoid_top_left_x').value
        self.trap_tl_y = self.get_parameter('trapezoid_top_left_y').value
        self.trap_tr_x = self.get_parameter('trapezoid_top_right_x').value
        self.trap_tr_y = self.get_parameter('trapezoid_top_right_y').value
        self.trap_br_x = self.get_parameter('trapezoid_bottom_right_x').value
        self.trap_br_y = self.get_parameter('trapezoid_bottom_right_y').value
        self.trap_bl_x = self.get_parameter('trapezoid_bottom_left_x').value
        self.trap_bl_y = self.get_parameter('trapezoid_bottom_left_y').value
        
        # Image processing parameters
        self.gaussian_kernel_size = self.get_parameter('gaussian_kernel_size').value
        self.gaussian_sigma = self.get_parameter('gaussian_sigma').value
        self.grayscale_threshold = self.get_parameter('grayscale_threshold').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.morph_erode_iterations = self.get_parameter('morph_erode_iterations').value
        self.morph_dilate_iterations = self.get_parameter('morph_dilate_iterations').value
        
        # Canny edge parameters
        self.canny_threshold1 = self.get_parameter('canny_threshold1').value
        self.canny_threshold2 = self.get_parameter('canny_threshold2').value
        
        # Side cropping parameters
        self.crop_side_percent = self.get_parameter('crop_side_percent').value
        self.rectangle_thickness = self.get_parameter('rectangle_thickness').value
        
        # Hough transform parameters
        self.hough_rho = self.get_parameter('hough_rho').value
        self.hough_theta_fraction = self.get_parameter('hough_theta_fraction').value
        self.hough_threshold = self.get_parameter('hough_threshold').value
        self.hough_min_line_length = self.get_parameter('hough_min_line_length').value
        self.hough_max_line_gap = self.get_parameter('hough_max_line_gap').value
        
        # Timer for periodic processing
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

        # Initialize variables
        self.image = None
        self.bridge = CvBridge()
        
        # Publishers for each processing step
        self.trapezoidal_masked_roi_pub = self.create_publisher(Image, 'trapezoidal_masked_roi', 10)
        self.grayscale_pub = self.create_publisher(Image, 'grayscale', 10)
        self.thresholded_pub = self.create_publisher(Image, 'thresholded', 10)
        self.edges_pub = self.create_publisher(Image, 'edges', 10)
        self.hough_lines_pub = self.create_publisher(Image, 'hough_lines', 10)
        
        # Raw camera images subscriber
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos.qos_profile_sensor_data
        )
     
        self.get_logger().info('LineDetection Start.')

    def image_callback(self, msg: Image) -> None:
        """Callback to convert ROS image to OpenCV format and store it."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}.")
            return
        
    def timer_callback(self) -> None:
        """Main timer function to process images and detect lines."""
        # Check if image has been received
        if self.image is None:
            return
        
        # Resize image if needed
        height, width = self.image.shape[:2]
        if (width, height) != (self.target_width, self.target_height):
            resized_img = cv2.resize(self.image, (self.target_width, self.target_height))
        else:
            resized_img = self.image.copy()
        
        # Crop the ROI from the bottom part of the image
        roi_start = int(self.target_height * (self.roi_percent/100.0))
        roi = resized_img[roi_start:, :]
        roi_height, roi_width = roi.shape[:2]
        
        # Create customizable trapezoidal mask
        trapezoid = np.array([[
            (self.trap_tl_x, self.trap_tl_y),             # Top-left
            (self.trap_tr_x, self.trap_tr_y),             # Top-right
            (self.trap_br_x, self.trap_br_y),             # Bottom-right
            (self.trap_bl_x, self.trap_bl_y)              # Bottom-left
        ]], dtype=np.int32)
        
        # Create mask with the specified trapezoid
        mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
        cv2.fillPoly(mask, trapezoid, 255)
        
        # Apply the mask
        roi_masked = np.where(mask[:,:,np.newaxis] == 0, 255, roi)
        
        # Publish trapezoidal masked ROI
        trapezoidal_masked_roi_msg = self.bridge.cv2_to_imgmsg(roi_masked, encoding='bgr8')
        self.trapezoidal_masked_roi_pub.publish(trapezoidal_masked_roi_msg)
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi_masked, cv2.COLOR_BGR2GRAY)
        
        # Publish grayscale
        grayscale_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
        self.grayscale_pub.publish(grayscale_msg)
        
        # Reduce noise with Gaussian blur
        blurred = cv2.GaussianBlur(
            gray, 
            tuple(self.gaussian_kernel_size), 
            self.gaussian_sigma
        )

        # Threshold grayscale image to binary image
        _, binary_inv = cv2.threshold(blurred, self.grayscale_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Apply morphological operations to clean noise
        kernel = np.ones(tuple(self.morph_kernel_size), np.uint8)
        morph = cv2.erode(binary_inv, kernel, iterations=self.morph_erode_iterations)
        morph = cv2.dilate(morph, kernel, iterations=self.morph_dilate_iterations)

        # Publish thresholded
        thresholded_msg = self.bridge.cv2_to_imgmsg(morph, encoding='mono8')
        self.thresholded_pub.publish(thresholded_msg)
        
        # Canny edge detection with configurable thresholds
        canny_edges = cv2.Canny(morph, self.canny_threshold1, self.canny_threshold2)
        
        # Side crop mask with configurable percentage
        crop_x = int(roi_width * self.crop_side_percent)
        side_mask = np.zeros_like(canny_edges)
        cv2.rectangle(
            side_mask,
            (crop_x, 0),                        # Top-left corner
            (roi_width - crop_x, roi_height),   # Bottom-right corner
            255,                                # White region
            thickness=self.rectangle_thickness  # Configurable thickness (-1 for filled)
        )
        
        # Apply side mask
        canny_edges = cv2.bitwise_and(canny_edges, canny_edges, mask=side_mask)
        
        # Final mask with trapezoid
        edges_roi = cv2.bitwise_and(canny_edges, canny_edges, mask=mask)
        
        # Publish edges
        edges_msg = self.bridge.cv2_to_imgmsg(edges_roi, encoding='mono8')
        self.edges_pub.publish(edges_msg)
        
        # Detect lines using HoughLinesP with configurable parameters
        lines = cv2.HoughLinesP(
            edges_roi,
            rho=self.hough_rho,
            theta=np.pi / self.hough_theta_fraction,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )
        
        # Draw detected lines
        output = roi.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        # Publish Hough lines
        hough_lines_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        self.hough_lines_pub.publish(hough_lines_msg)
        
        # Log the number of detected lines
        if lines is not None:
            self.get_logger().info(f'Processed frame with {len(lines)} lines detected')
        else:
            self.get_logger().info('Processed frame but no lines detected')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        line_detection_node = LineDetection()
        rclpy.spin(line_detection_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'line_detection_node' in locals():
            line_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()