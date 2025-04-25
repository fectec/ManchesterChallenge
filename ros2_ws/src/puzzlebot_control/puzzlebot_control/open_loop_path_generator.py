#!/usr/bin/env python3

import rclpy
import json
import math

from rclpy.node import Node

from custom_interfaces.msg import OpenLoopPose
from puzzlebot_utils.utils.math_helpers import normalize_angle_diff, angle_diff_signed, yaw_to_quaternion

class OpenLoopPathGenerator(Node):
    """
    OpenLoopPathGenerator is a node that generates and publishes 
    robot movement commands based on a list of waypoints.
    Each waypoint may contain a target position, and the robot 
    will either follow these waypoints with time-based speed 
    scaling or fixed linear/angular speeds.
    
    The waypoints are specified in JSON format and consist of either:
    - A total execution time for each waypoint.
    - Or the desired rotational and linear speeds to reach the target.
    """
    
    def __init__(self):
        super().__init__('open_loop_path_generator')

        # Declare parameters
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('waypoints_json', '[]')
        self.declare_parameter('max_linear_speed', 0.17)    # m/s
        self.declare_parameter('max_angular_speed', 1.0)    # rad/s
        self.declare_parameter('safety_margin', 0.2)

        # Retrieve parameters and parse the waypoints JSON string
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        raw_json = self.get_parameter('waypoints_json').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.safety_margin = self.get_parameter('safety_margin').value
        
        try:
            self.waypoints = json.loads(raw_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}.")
            self.waypoints = None

        if not isinstance(self.waypoints, list):
            self.get_logger().error("Invalid waypoints_json: must be a JSON array.")
            rclpy.shutdown()
            return
        
        # Publisher for OpenLoopPose, which contains robot's velocities and execution time
        self.open_loop_pose_pub = self.create_publisher(OpenLoopPose, '/puzzlebot_real/open_loop_pose', 10)

        # Initialize robot's current position and orientation, and start waypoint index
        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0
        self.waypoint_index = 0

        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / update_rate, self.process_waypoint)

        self.get_logger().info("OpenLoopPathGenerator Start.")  

    def process_waypoint(self):
        """
        Process the next waypoint in the list:
        1. Calculate distance and angle to target.
        2. Handle both time-based and speed-based movement options.
        3. Publish corresponding OpenLoopPose messages for each movement phase (rotation + translation).
        """
        # Check if there are more waypoints to process
        if not self.waypoints or self.waypoint_index >= len(self.waypoints):
            self.get_logger().error("No more waypoints or invalid input. Stopping.")
            rclpy.shutdown()
            return

        # Retrieve current waypoint
        wp = self.waypoints[self.waypoint_index]
        x_target = float(wp.get('x', 0.0))
        y_target = float(wp.get('y', 0.0))

        # Calculate the difference between current and target position
        dx = x_target - self.x_curr
        dy = y_target - self.y_curr
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate the angle to the target waypoint using atan2 (handles the correct quadrant)
        raw_goal_angle = math.atan2(dy, dx)
        angle_diff_signed = normalize_angle_diff(self.theta_curr, raw_goal_angle)    # Normalize the angle difference
        angle_diff = abs(angle_diff_signed)                                          # Angle difference in positive value
        sign = 1.0 if angle_diff_signed >= 0 else -1.0                               # Determine the direction of rotation

        # Time-based movement (scale speed to fit within total time)
        if 'total_time' in wp and 'rot_speed' not in wp and 'lin_speed' not in wp:
            total_time = float(wp['total_time'])    # Total time allowed for the movement

            # Calculate the required time for rotation and translation at maximum speeds
            time_for_rotation_at_max = angle_diff / self.max_angular_speed if self.max_angular_speed > 0 else 0
            time_for_translation_at_max = distance / self.max_linear_speed if self.max_linear_speed > 0 else 0

            # Total time required for both rotation and translation
            raw_time_sum = time_for_rotation_at_max + time_for_translation_at_max
            scaled_time_sum = raw_time_sum * (1.0 + self.safety_margin)

            # If the required time exceeds the total available time, stop
            if scaled_time_sum > total_time:
                self.get_logger().error("Impossible speeds for time mode (scaled sum exceeds total_time).")
                rclpy.shutdown()
                return

            # Scale the time for rotation and translation to fit the total time
            scale_factor = total_time / scaled_time_sum if scaled_time_sum > 0 else 1.0
            effective_rotation_speed = self.max_angular_speed * scale_factor
            effective_translation_speed = self.max_linear_speed * scale_factor

            # Clamp the speeds if they exceed the maximum allowed speeds
            if effective_rotation_speed > self.max_angular_speed:
                effective_rotation_speed = self.max_angular_speed
            if effective_translation_speed > self.max_linear_speed:
                effective_translation_speed = self.max_linear_speed

            # Calculate the new time values for rotation and translation
            rotation_time = angle_diff / effective_rotation_speed if effective_rotation_speed > 0 else 0
            translation_time = distance / effective_translation_speed if effective_translation_speed > 0 else 0
        
            # Check if the sum of rotation time and translation time exceeds the total time specified in the waypoint
            if (rotation_time + translation_time) > total_time:
                self.get_logger().error("Even clamped speeds exceed total_time.")
                rclpy.shutdown()
                return
            
            # Publish the subcommands (rotation + translation)
            self.publish_subcommands(
                effective_rotation_speed, 
                effective_translation_speed, 
                angle_diff, 
                distance, 
                rotation_time, 
                translation_time, 
                x_target, 
                y_target, 
                sign
            )

        # Speed-based movement (user-defined rotational and linear speeds)  
        elif 'rot_speed' in wp and 'lin_speed' in wp and 'total_time' not in wp:
            rs = float(wp['rot_speed'])     # User-defined rotational speed
            ls = float(wp['lin_speed'])     # User-defined linear speed

            # Check if the user-defined speeds exceed the limits
            if rs > self.max_angular_speed or ls > self.max_linear_speed:
                self.get_logger().error("User-defined speeds exceed puzzlebot limits.")
                rclpy.shutdown()
                return
            
            # Calculate the rotation and translation times based on the speed values
            rotation_time = angle_diff / rs if rs>0 else 0
            translation_time = distance / ls if ls>0 else 0

            # Publish the subcommands (rotation + translation)
            self.publish_subcommands(
                rs, 
                ls, 
                angle_diff, 
                distance, 
                rotation_time, 
                translation_time, 
                x_target, 
                y_target, 
                sign
            )
        else:
            self.get_logger().error("Waypoint must define either total_time OR rot_speed+lin_speed.")
            rclpy.shutdown()
            return

        self.waypoint_index += 1     # Move to the next waypoint

    def publish_subcommands(self, rot_spd, lin_spd, angle_diff, dist, rot_time, lin_time, x_tar, y_tar, sign):
        """
        Publish the subcommands (rotation and translation) to the 'pose' topic as OpenLoopPose messages.
        The robot will rotate and move to the target waypoint.
        """
        if angle_diff > 1e-6 and rot_time > 1e-6:
            msg_rot = OpenLoopPose()
            msg_rot.pose.position.x = self.x_curr
            msg_rot.pose.position.y = self.y_curr
            msg_rot.pose.orientation = yaw_to_quaternion(self.theta_curr)
            msg_rot.linear_velocity = 0.0
            msg_rot.angular_velocity = sign * rot_spd
            msg_rot.execution_time = rot_time
            self.open_loop_pose_pub.publish(msg_rot)

        if dist > 1e-6 and lin_time > 1e-6:
            msg_trans = OpenLoopPose()
            msg_trans.pose.position.x = self.x_curr
            msg_trans.pose.position.y = self.y_curr
            msg_trans.pose.orientation = yaw_to_quaternion(self.theta_curr + angle_diff_signed(sign, angle_diff))
            msg_trans.linear_velocity = lin_spd
            msg_trans.angular_velocity = 0.0
            msg_trans.execution_time = lin_time
            self.open_loop_pose_pub.publish(msg_trans)

        # Update the current position and orientation of the robot based on the target position
        # This ensures that after completing a subcommand (rotation or translation), the robot's state is updated to reflect the new position
        self.x_curr = x_tar
        self.y_curr = y_tar

        # Adjust the current orientation based on the signed angle difference (sign determines direction)
        # The modulo operation ensures that the angle stays within the range of [-pi, pi], maintaining proper angular continuity
        raw_diff = angle_diff if (sign > 0) else -angle_diff
        self.theta_curr = (self.theta_curr + raw_diff + math.pi) % (2 * math.pi) - math.pi

    def stop_node(self):
        if self.timer is not None:
            self.timer.cancel()
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopPathGenerator()
    
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