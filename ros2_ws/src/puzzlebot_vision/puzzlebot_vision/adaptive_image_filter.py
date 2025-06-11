#!/usr/bin/env python3

import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy import qos

from sensor_msgs.msg import Image, CompressedImage

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class AdaptiveImageFilter(Node):
    def __init__(self):
        super().__init__('adaptive_image_filter')
        
        # Declare parameters
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('use_compressed', True)

        self.declare_parameter('sampling_interval', 5.0)    # s
        self.declare_parameter('adaptation_strength', 0.7)  # 0-1
        
        # Target values for normalization
        self.declare_parameter('target_brightness', 128)
        self.declare_parameter('target_contrast', 50)
        self.declare_parameter('target_saturation', 60)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value

        self.sampling_interval = self.get_parameter('sampling_interval').value
        self.adaptation_strength = self.get_parameter('adaptation_strength').value
        
        self.target_brightness = self.get_parameter('target_brightness').value
        self.target_contrast = self.get_parameter('target_contrast').value
        self.target_saturation = self.get_parameter('target_saturation').value
        
        # Initialize
        self.bridge = CvBridge()
        self.last_sample_time = 0
        self.correction_params = {
            'brightness': 0,
            'contrast': 1.0,
            'saturation': 1.0
        }
        
        # Create publishers
        if self.use_compressed:
            output_topic = f"{self.image_topic}_filtered/compressed"
            self.publisher = self.create_publisher(
                CompressedImage,
                output_topic,
                qos.qos_profile_sensor_data
            )
        else:
            output_topic = f"{self.image_topic}_filtered"
            self.publisher = self.create_publisher(
                Image,
                output_topic,
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
        
        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Validate initial parameters
        init_params = [
            Parameter('sampling_interval', Parameter.Type.DOUBLE, self.sampling_interval),
            Parameter('adaptation_strength', Parameter.Type.DOUBLE, self.adaptation_strength),
            Parameter('target_brightness', Parameter.Type.INTEGER, self.target_brightness),
            Parameter('target_contrast', Parameter.Type.INTEGER, self.target_contrast),
            Parameter('target_saturation', Parameter.Type.INTEGER, self.target_saturation),
        ]
        
        result = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")
        
        self.get_logger().info(f"AdaptiveImageFilter Start.")
        self.get_logger().info(f"Input: {self.image_topic}, Output: {output_topic}")
        self.get_logger().info(f"Sampling: {self.sampling_interval}s, Strength: {self.adaptation_strength}")
    
    def analyze_image(self, image):
        """Analyze image properties for adaptive filtering."""
        # Convert to LAB for better luminance analysis
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l_channel = lab[:, :, 0]
        
        # Calculate brightness (mean of L channel)
        brightness = np.mean(l_channel)
        
        # Calculate contrast (std dev of L channel)
        contrast = np.std(l_channel)
        
        # Calculate saturation in HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        saturation = np.mean(hsv[:, :, 1])
        
        return brightness, contrast, saturation
    
    def update_correction_params(self, brightness, contrast, saturation):
        """Update correction parameters based on analysis."""
        # Brightness correction (additive)
        brightness_diff = self.target_brightness - brightness
        self.correction_params['brightness'] = int(brightness_diff * self.adaptation_strength)
        
        # Contrast correction (multiplicative)
        if contrast > 0:
            contrast_ratio = self.target_contrast / contrast
            self.correction_params['contrast'] = 1.0 + (contrast_ratio - 1.0) * self.adaptation_strength
            self.correction_params['contrast'] = np.clip(self.correction_params['contrast'], 0.5, 2.0)
        
        # Saturation correction
        if saturation > 0:
            sat_ratio = self.target_saturation / saturation
            self.correction_params['saturation'] = 1.0 + (sat_ratio - 1.0) * self.adaptation_strength
            self.correction_params['saturation'] = np.clip(self.correction_params['saturation'], 0.5, 2.0)
    
    def apply_fast_correction(self, image):
        """Apply correction with minimal operations."""
        # Apply brightness and contrast in one operation
        # out = image * contrast + brightness
        corrected = cv2.convertScaleAbs(
            image,
            alpha=self.correction_params['contrast'],
            beta=self.correction_params['brightness']
        )
        
        # Apply saturation correction only if significantly different from 1.0
        if abs(self.correction_params['saturation'] - 1.0) > 0.1:
            hsv = cv2.cvtColor(corrected, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 1] *= self.correction_params['saturation']
            hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
            corrected = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
        
        return corrected
    
    def image_callback(self, msg):
        """Process incoming images."""
        try:
            # Convert ROS image to OpenCV
            if self.use_compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Check if it's time to sample
            current_time = time.time()
            if current_time - self.last_sample_time >= self.sampling_interval:
                # Downsample for faster analysis
                small = cv2.resize(cv_image, (0, 0), fx=0.25, fy=0.25)
                brightness, contrast, saturation = self.analyze_image(small)
                self.update_correction_params(brightness, contrast, saturation)
                self.last_sample_time = current_time
                
                self.get_logger().debug(
                    f"Updated params - B:{self.correction_params['brightness']:.1f}, "
                    f"C:{self.correction_params['contrast']:.2f}, "
                    f"S:{self.correction_params['saturation']:.2f}"
                )
            
            # Apply correction
            corrected = self.apply_fast_correction(cv_image)
            
            # Publish
            if self.use_compressed:
                out_msg = self.bridge.cv2_to_compressed_imgmsg(corrected)
                out_msg.header = msg.header
            else:
                out_msg = self.bridge.cv2_to_imgmsg(corrected, encoding='bgr8')
                out_msg.header = msg.header
            
            self.publisher.publish(out_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validate and apply parameters."""
        for param in params:
            if param.name == 'sampling_interval':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="sampling_interval must be > 0.")
                self.sampling_interval = float(param.value)
                
            elif param.name == 'adaptation_strength':
                if not isinstance(param.value, (int, float)) or not (0.0 <= param.value <= 1.0):
                    return SetParametersResult(successful=False, reason="adaptation_strength must be between 0.0 and 1.0.")
                self.adaptation_strength = float(param.value)
                
            elif param.name == 'target_brightness':
                if not isinstance(param.value, int) or not (0 <= param.value <= 255):
                    return SetParametersResult(successful=False, reason="target_brightness must be between 0 and 255.")
                self.target_brightness = param.value
                
            elif param.name == 'target_contrast':
                if not isinstance(param.value, int) or param.value <= 0:
                    return SetParametersResult(successful=False, reason="target_contrast must be > 0.")
                self.target_contrast = param.value
                
            elif param.name == 'target_saturation':
                if not isinstance(param.value, int) or param.value <= 0:
                    return SetParametersResult(successful=False, reason="target_saturation must be > 0.")
                self.target_saturation = param.value
                
            elif param.name == 'image_topic':
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(successful=False, reason="image_topic must be a non-empty string.")
                # Note: Changing image_topic at runtime would require recreating subscribers/publishers
                self.get_logger().warn("Changing image_topic at runtime is not supported. Restart node with new parameter.")
                
            elif param.name == 'use_compressed':
                if not isinstance(param.value, bool):
                    return SetParametersResult(successful=False, reason="use_compressed must be a boolean.")
                # Note: Changing use_compressed at runtime would require recreating subscribers/publishers
                self.get_logger().warn("Changing use_compressed at runtime is not supported. Restart node with new parameter.")

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = AdaptiveImageFilter()
    except Exception as e:
        print(f"[FATAL] AdaptiveImageFilter failed to initialize: {e}", file=sys.stderr)
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