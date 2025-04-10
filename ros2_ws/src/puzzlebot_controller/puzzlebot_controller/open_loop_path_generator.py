#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import json  # Used for parsing the JSON string of waypoints

from custom_interfaces.msg import OpenLoopPose
from geometry_msgs.msg import Pose, Point, Quaternion

def yaw_to_quaternion(yaw):
    """
    Converts a yaw angle (in radians) into a quaternion
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class OpenLoopPathGenerator(Node):
    def __init__(self):
        super().__init__('open_loop_path_generator')
        
        # Create a publisher for the custom OpenLoopPose messages on the 'pose' topic
        self.pose_pub = self.create_publisher(OpenLoopPose, 'pose', 10)
        
        # Declare parameters
        # 'waypoints_json' will hold a JSON string representing a list of waypoints
        self.declare_parameter('waypoints_json', '[]')
        self.declare_parameter('max_linear_speed', 1.5)         # Maximum allowed linear speed (m/s)
        self.declare_parameter('max_angular_speed', 1.8)        # Maximum allowed angular speed (rad/s)
        self.declare_parameter('safety_margin', 0.2)            # Extra margin (percentage) for robustness
        self.declare_parameter('default_linear_velocity', 1.0)  # Fallback linear speed (m/s)
        self.declare_parameter('default_angular_velocity', 1.2) # Fallback angular speed (rad/s)
        self.declare_parameter('default_execution_time', 5.0)   # Fallback execution time (s)
        
        # Retrieve the parameters
        wjson = self.get_parameter('waypoints_json').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.default_linear_velocity = self.get_parameter('default_linear_velocity').value
        self.default_angular_velocity = self.get_parameter('default_angular_velocity').value
        self.default_execution_time = self.get_parameter('default_execution_time').value
        
        # Parse the JSON string into a list of waypoints (each waypoint is a dictionary)
        try:
            self.waypoints = json.loads(wjson)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse 'waypoints_json'. Error: {e}")
            self.waypoints = []
        
        # Ensure waypoints is a list; otherwise, warn the user
        if not isinstance(self.waypoints, list):
            self.get_logger().error("waypoints_json did not parse into a list. Setting empty.")
            self.waypoints = []
        
        if not self.waypoints:
            self.get_logger().warn("No valid waypoints provided!")
        
        # Initialize the current waypoint index
        self.current_index = 0
        
        # Create a timer to publish waypoints every second
        self.timer = self.create_timer(1.0, self.publish_waypoint)

    def publish_waypoint(self):
        """
        Publishes the next waypoint as an OpenLoopPose message.
        The control mode is determined directly from each waypoint's 'control_mode' field:
            - mode 1: time-based control (we expect an 'execution_time')
            - mode 0: velocity-based control (we expect 'target_linear_velocity' and optionally 'target_angular_velocity')
        The node will compute missing values (e.g., required velocities from time or expected execution time from speeds)
        and issue warnings if computed values exceed dynamic limits.
        """
        # Check if we have published all waypoints
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints have been published")
            self.timer.cancel()
            return
        
        # Retrieve the current waypoint (a dictionary)
        wp = self.waypoints[self.current_index]
        msg = OpenLoopPose()

        # Fill in the pose field
        pos = Point(
            x = wp.get('x', 0.0),
            y = wp.get('y', 0.0),
            z = wp.get('z', 0.0)  # 0 for planar navigation
        )
        yaw = wp.get('yaw', 0.0)
        ori = yaw_to_quaternion(yaw)
        pose = Pose()
        pose.position = pos
        pose.orientation = ori
        msg.pose = pose

        # Retrieve and set the control mode from the waypoint
        # control_mode should be defined by the user:
        #   0 => velocity-based control; 1 => time-based control
        mode = int(wp.get('control_mode', 0))  
        msg.control_mode = mode

        if mode == 1:
            # ---------------------------
            # TIME-BASED CONTROL MODE
            # ---------------------------
            # We expect an execution_time parameter from the user; if not provided, fallback
            exec_time = float(wp.get('execution_time', self.default_execution_time))
            msg.execution_time = exec_time

            # If this is not the first waypoint, estimate the required velocities based on the difference
            # between the current and previous waypoint
            if self.current_index > 0:
                prev_wp = self.waypoints[self.current_index - 1]
                dx = wp.get('x', 0.0) - prev_wp.get('x', 0.0)
                dy = wp.get('y', 0.0) - prev_wp.get('y', 0.0)
                distance = math.sqrt(dx**2 + dy**2)
                required_linear = distance / exec_time

                prev_yaw = prev_wp.get('yaw', 0.0)
                delta_yaw = abs(yaw - prev_yaw)
                required_angular = delta_yaw / exec_time

                # Apply safety margins to account for uncertainties
                required_linear *= (1.0 + self.safety_margin)
                required_angular *= (1.0 + self.safety_margin)

                # Warn if the required velocities exceed dynamic limits
                if (required_linear > self.max_linear_speed) or (required_angular > self.max_angular_speed):
                    self.get_logger().warn(
                        f"Waypoint {self.current_index} may be unreachable: "
                        f"required_linear={required_linear:.2f} m/s, "
                        f"required_angular={required_angular:.2f} rad/s exceed limits"
                    )
                msg.target_linear_velocity = required_linear
                msg.target_angular_velocity = required_angular
            else:
                # For the first waypoint, no relative movement; default to zeros
                msg.target_linear_velocity = 0.0
                msg.target_angular_velocity = 0.0

        else:
            # ---------------------------
            # VELOCITY-BASED CONTROL MODE (mode == 0)
            # ---------------------------
            # Read provided velocities with fallback to defaults
            lin_vel = float(wp.get('target_linear_velocity', self.default_linear_velocity))
            ang_vel = float(wp.get('target_angular_velocity', self.default_angular_velocity))
            msg.target_linear_velocity = lin_vel
            msg.target_angular_velocity = ang_vel

            # If not the first waypoint, compute the expected execution time based on distance and angle difference
            if self.current_index > 0:
                prev_wp = self.waypoints[self.current_index - 1]
                dx = wp.get('x', 0.0) - prev_wp.get('x', 0.0)
                dy = wp.get('y', 0.0) - prev_wp.get('y', 0.0)
                distance = math.sqrt(dx**2 + dy**2)
                linear_time = distance / lin_vel if lin_vel > 0 else 0.0

                prev_yaw = prev_wp.get('yaw', 0.0)
                delta_yaw = abs(yaw - prev_yaw)
                angular_time = (delta_yaw / ang_vel) if ang_vel > 0 else 0.0

                # Choose the maximum time required among translation and rotation and apply safety margin
                exec_time = max(linear_time, angular_time) * (1.0 + self.safety_margin)
                msg.execution_time = exec_time

                # Warn if the provided velocities exceed dynamic limits
                if (lin_vel > self.max_linear_speed) or (ang_vel > self.max_angular_speed):
                    self.get_logger().warn(f"Waypoint {self.current_index} may be unreachable: speeds exceed dynamic limits")
            else:
                msg.execution_time = self.default_execution_time

        # Log the details and publish the message
        self.get_logger().info(
            f"Publishing waypoint {self.current_index}: pos=({pos.x}, {pos.y}), "
            f"yaw={yaw:.2f}, mode={msg.control_mode}, lin_vel={msg.target_linear_velocity:.2f}, "
            f"ang_vel={msg.target_angular_velocity:.2f}, time={msg.execution_time:.2f}"
        )
        self.pose_pub.publish(msg)
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopPathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()