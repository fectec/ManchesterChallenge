#!/usr/bin/env python3

import rclpy
import json
import math
import sys

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
        self.declare_parameter('update_rate',        1.0)
        self.declare_parameter('waypoints_json',    '[]')
        self.declare_parameter('min_linear_speed',  0.1 )
        self.declare_parameter('max_linear_speed',  0.17)
        self.declare_parameter('min_angular_speed', 0.1 )
        self.declare_parameter('max_angular_speed', 1.0 )
        self.declare_parameter('drift_margin',      0.0 )

        # Retrieve parameters
        update_rate        = self.get_parameter('update_rate')      .get_parameter_value().double_value
        raw_json           = self.get_parameter('waypoints_json')   .get_parameter_value().string_value
        min_lin            = self.get_parameter('min_linear_speed') .get_parameter_value().double_value
        max_lin            = self.get_parameter('max_linear_speed') .get_parameter_value().double_value
        min_ang            = self.get_parameter('min_angular_speed').get_parameter_value().double_value
        max_ang            = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        drift_margin       = self.get_parameter('drift_margin')     .get_parameter_value().double_value

        # Validate parameters
        if update_rate <= 0.0:
            self.get_logger().error(f"update_rate must be > 0.0 (got {update_rate}).")
            raise ValueError("Invalid update_rate.")

        if min_lin <= 0.0:
            self.get_logger().error(f"min_linear_speed must be > 0.0 (got {min_lin}).")
            raise ValueError("Invalid min_linear_speed.")
        if max_lin <= min_lin:
            self.get_logger().error(
                f"max_linear_speed ({max_lin}) must be > min_linear_speed ({min_lin})."
            )
            raise ValueError("Invalid max_linear_speed.")

        if min_ang <= 0.0:
            self.get_logger().error(f"min_angular_speed must be > 0.0 (got {min_ang}).")
            raise ValueError("Invalid min_angular_speed.")
        if max_ang <= min_ang:
            self.get_logger().error(
                f"max_angular_speed ({max_ang}) must be > min_angular_speed ({min_ang})."
            )
            raise ValueError("Invalid max_angular_speed.")

        if not (0.0 <= drift_margin < 1.0):
            self.get_logger().error(f"drift_margin must be in [0.0,1.0) (got {drift_margin}).")
            raise ValueError("Invalid drift_margin.")

        # Parse & validate waypoints JSON
        try:
            wps = json.loads(raw_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse waypoints_json: {e}.")
            raise

        if not isinstance(wps, list):
            self.get_logger().error("waypoints_json must be a JSON array.")
            raise ValueError("Invalid waypoints_json.")

        for idx, wp in enumerate(wps, start=1):
            if 'total_time' in wp:
                try:
                    t = float(wp['total_time'])
                except (ValueError, TypeError):
                    self.get_logger().error(f"Waypoint {idx}: total_time must be a number (got {wp['total_time']}).")
                    raise
                if t <= 0.0:
                    self.get_logger().error(f"Waypoint {idx}: total_time must be > 0 (got {t:.2f}).")
                    raise ValueError("Invalid waypoint total_time.")

            if 'lin_speed' in wp:
                try:
                    ls = float(wp['lin_speed'])
                except (ValueError, TypeError):
                    self.get_logger().error(f"Waypoint {idx}: lin_speed must be a number (got {wp['lin_speed']}).")
                    raise
                if ls <= 0.0 or ls < min_lin or ls > max_lin:
                    self.get_logger().error(
                        f"Waypoint {idx}: lin_speed ({ls:.2f}) must be >0 and in [{min_lin:.2f},{max_lin:.2f}]."
                    )
                    raise ValueError("Invalid waypoint lin_speed.")

            if 'rot_speed' in wp:
                try:
                    rs = float(wp['rot_speed'])
                except (ValueError, TypeError):
                    self.get_logger().error(f"Waypoint {idx}: rot_speed must be a number (got {wp['rot_speed']}).")
                    raise
                if rs <= 0.0 or rs < min_ang or rs > max_ang:
                    self.get_logger().error(
                        f"Waypoint {idx}: rot_speed ({rs:.2f}) must be >0 and in [{min_ang:.2f},{max_ang:.2f}]."
                    )
                    raise ValueError("Invalid waypoint rot_speed.")

        self.update_rate        = update_rate
        self.waypoints          = wps
        self.min_linear_speed   = min_lin
        self.max_linear_speed   = max_lin
        self.min_angular_speed  = min_ang
        self.max_angular_speed  = max_ang
        self.drift_margin       = drift_margin

        # Publisher for OpenLoopPose, which contains robot's velocities and execution time
        self.open_loop_pose_pub = self.create_publisher(OpenLoopPose, 'puzzlebot_real/open_loop_pose', 10)

        # Initialize robot's current position and orientation, and start waypoint index
        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0
        self.waypoint_index = 0

        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.process_waypoint)

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

            # Compute unscaled times at max speed
            time_for_rotation_at_max    =   angle_diff / self.max_angular_speed   
            time_for_translation_at_max =   distance   / self.max_linear_speed    
            raw_time_sum = time_for_rotation_at_max + time_for_translation_at_max

            # Check feasibility
            if raw_time_sum > total_time:
                self.get_logger().error(
                    "Impossible speeds for time mode "
                    f"(needs {raw_time_sum:.2f} s > allotted {total_time:.2f} s)."
                )
                rclpy.shutdown()
                return

            # Scale exactly to fill total_time
            usable_time = total_time * (1.0 - self.drift_margin)
            scale_factor =  raw_time_sum / usable_time if usable_time > 0.0 else 0.0
            effective_rotation_speed    = self.max_angular_speed * scale_factor 
            effective_translation_speed = self.max_linear_speed *  scale_factor 
 
            # Dead‐zone checks
            if effective_rotation_speed < self.min_angular_speed:
                self.get_logger().error(
                    f"Effective rotation speed {effective_rotation_speed:.3f} < min_angular_speed {self.min_angular_speed:.3f}."
                )
                rclpy.shutdown()
                return

            if effective_translation_speed < self.min_linear_speed:
                self.get_logger().error(
                    f"Effective translation speed {effective_translation_speed:.3f} < min_linear_speed {self.min_linear_speed:.3f}."
                )
                rclpy.shutdown()
                return
            
            # These times will now sum to total_time
            rotation_time    = angle_diff / effective_rotation_speed    if effective_rotation_speed    > 0.0 else 0.0
            translation_time = distance   / effective_translation_speed if effective_translation_speed > 0.0 else 0.0

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
                self.get_logger().error("User-defined speeds exceed Puzzlebot limits.")
                rclpy.shutdown()
                return
            
            # Calculate the rotation and translation times based on the speed values
            rotation_time = angle_diff / rs 
            translation_time = distance / ls

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

    try:
        node = OpenLoopPathGenerator()
    except Exception as e:
        print(f"[FATAL] OpenLoopPathGenerator failed to initialize: {e}.", file=sys.stderr)
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