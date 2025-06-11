#!/usr/bin/env python3

import sys
import os
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Image, CompressedImage
from yolov8_msgs.msg import InferenceResult, Yolov8Inference

class YoloV8Detection(Node):
    def __init__(self):
        super().__init__('yolov8_detection')
        
        # Declare parameters
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('use_compressed', False)

        self.declare_parameter('model_name', 'puzzlebot_traffic_signs.pt')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('update_rate', 15.0)

        # Retrieve parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
        self.model_name = self.get_parameter('model_name').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        # Register the parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Validate initial parameters
        init_params = [
            Parameter('image_topic', Parameter.Type.STRING, self.image_topic),
            Parameter('use_compressed', Parameter.Type.BOOL, self.use_compressed),
            Parameter('model_name', Parameter.Type.STRING, self.model_name),
            Parameter('confidence_threshold', Parameter.Type.DOUBLE, self.confidence_threshold),
            Parameter('update_rate', Parameter.Type.DOUBLE, self.update_rate),
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")
        
        # Initialize variables
        self.bridge = CvBridge()
        self.image = None
        self.model = None
        
        # Load YOLO model
        self._load_yolo_model()

        # Publishers
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "Yolov8_Inference", qos.qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, "inference_result", qos.qos_profile_sensor_data)
        
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
        
        self.get_logger().info('YoloV8Detection Start.')

    def _load_yolo_model(self):
        """Load YOLO model from package share directory."""
        try:
            package_share_dir = get_package_share_directory('yolobot_recognition')
            model_path = os.path.join(package_share_dir, 'models', self.model_name)
            
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model not found at {model_path}.")
                
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO model loaded: {self.model_name}.')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

    def image_callback(self, msg):
        """Callback to convert ROS image to OpenCV format and store it."""
        try:
            if self.use_compressed:
                self.image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Check if image is grayscale and convert to BGR if needed
            if len(self.image.shape) == 2:  # Grayscale image
                self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
            elif self.image.shape[2] == 1:  # Single channel
                self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
                
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")

    def timer_callback(self):
        """Main timer function to process images and detect objects."""
        if self.image is None or self.model is None:
            return

        try:
            # Run YOLO inference
            results = self.model(self.image)
            
            # Create the inference message
            yolov8_inference = Yolov8Inference()
            yolov8_inference.header.frame_id = "camera_frame"
            yolov8_inference.header.stamp = self.get_clock().now().to_msg()
            
            # Dictionary to store the highest confidence detection for each class
            best_detections_by_class = {}
            
            for r in results:
                boxes = r.boxes
                if boxes is not None:
                    for box in boxes:
                        # Get confidence score
                        conf = float(box.conf[0].to('cpu').detach().numpy())
                        
                        # Only consider detections above confidence threshold
                        if conf >= self.confidence_threshold:
                            # Get box coordinates and class
                            b = box.xyxy[0].to('cpu').detach().numpy().copy()
                            c = int(box.cls[0].to('cpu').detach().numpy())
                            class_name = self.model.names[c]
                            
                            # Check if this is the most confident detection for this class
                            if class_name not in best_detections_by_class or conf > best_detections_by_class[class_name]['confidence']:
                                best_detections_by_class[class_name] = {
                                    'coords': b,
                                    'class_id': c,
                                    'confidence': conf,
                                    'class_name': class_name
                                }
            
            # Add the best detection for each class to the inference result
            detection_count = 0
            for class_name, detection_data in best_detections_by_class.items():
                inference_result = InferenceResult()
                
                # Fill the message with the best detection data for this class
                inference_result.class_name = detection_data['class_name']
                inference_result.left = int(detection_data['coords'][0])
                inference_result.top = int(detection_data['coords'][1])
                inference_result.right = int(detection_data['coords'][2])
                inference_result.bottom = int(detection_data['coords'][3])
                
                yolov8_inference.yolov8_inference.append(inference_result)
                detection_count += 1
                
                self.get_logger().debug(
                    f"Best {class_name} detection: confidence {detection_data['confidence']:.2f}"
                )
            
            if detection_count > 0:
                self.get_logger().debug(f"Published {detection_count} best detections (one per class).")
            
            # Create custom annotated image with only the strongest detections
            annotated_frame = self.image.copy()
            
            # Define colors for different classes (BGR format)
            class_colors = {
                'fwd': (0, 255, 0),      # Green - Go ahead
                'right': (0, 0, 255),    # Red - Turn right
                'left': (255, 0, 0),     # Blue - Turn left  
                'triUp': (0, 165, 255),  # Orange - Road work warning
                'stop': (0, 0, 128),     # Dark Red - Stop sign
                'triDwn': (0, 255, 255)  # Yellow - Give way
            }
            
            # Draw only the best detections
            for class_name, detection_data in best_detections_by_class.items():
                coords = detection_data['coords']
                confidence = detection_data['confidence']
                
                # Get color for this class (default to green if class not in dictionary)
                color = class_colors.get(class_name, (0, 255, 0))
                
                # Draw bounding box with thinner line
                cv2.rectangle(annotated_frame, 
                            (int(coords[0]), int(coords[1])), 
                            (int(coords[2]), int(coords[3])), 
                            color, 1)
                
                # Draw label with confidence (smaller font)
                label = f"{class_name} {confidence:.2f}"
                font_scale = 0.4
                thickness = 1
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                
                # Draw smaller label background
                cv2.rectangle(annotated_frame,
                            (int(coords[0]), int(coords[1]) - label_size[1] - 6),
                            (int(coords[0]) + label_size[0] + 4, int(coords[1])),
                            color, -1)
                
                # Draw label text
                cv2.putText(annotated_frame, label,
                          (int(coords[0]) + 2, int(coords[1]) - 3),
                          cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
            
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_frame"
            
            self.image_pub.publish(image_msg)
            self.yolov8_pub.publish(yolov8_inference)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')

    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validates and applies updated node parameters."""
        model_changed = False
        
        for param in params:
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

            elif param.name == 'model_name':
                if not isinstance(param.value, str) or len(param.value.strip()) == 0:
                    return SetParametersResult(
                        successful=False,
                        reason="model_name must be a non-empty string."
                    )
                if param.value != self.model_name:
                    self.model_name = param.value
                    model_changed = True
                    self.get_logger().info(f"model_name updated: {self.model_name}.")

            elif param.name == 'confidence_threshold':
                if not isinstance(param.value, (int, float)) or param.value < 0.0 or param.value > 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason="confidence_threshold must be between 0.0 and 1.0."
                    )
                self.confidence_threshold = float(param.value)
                self.get_logger().info(f"confidence_threshold updated: {self.confidence_threshold}.")

            elif param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="update_rate must be > 0."
                    )
                self.update_rate = float(param.value)
                # Restart timer
                if hasattr(self, 'timer') and self.timer is not None:
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
                self.get_logger().info(f"update_rate updated: {self.update_rate} Hz.")

        # Reload model if model_name changed
        if model_changed:
            try:
                self._load_yolo_model()
            except Exception as e:
                return SetParametersResult(
                    successful=False,
                    reason=f"Failed to load new model: {e}"
                )

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = YoloV8Detection()
    except Exception as e:
        print(f"[FATAL] YoloV8Detection failed to initialize: {e}", file=sys.stderr)
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