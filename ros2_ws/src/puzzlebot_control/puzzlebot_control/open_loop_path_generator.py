#!/usr/bin/env python3

import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from puzzlebot_utils.utils.math_helpers import normalize_angle_diff, angle_diff_signed, yaw_to_quaternion

from custom_interfaces.msg import OpenLoopPose

class OpenLoopPathGenerator(Node):
    """
    Node that generates and publishes robot movement 
    commands based on a list of waypoints.
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
        self.declare_parameter('update_rate',        10.0)  # Hz
        self.declare_parameter('waypoints_json',    '[]')   
        self.declare_parameter('min_linear_speed',  0.1 )   # m/s
        self.declare_parameter('max_linear_speed',  0.17)   # m/s
        self.declare_parameter('min_angular_speed', 0.1 )   # rad/s
        self.declare_parameter('max_angular_speed', 1.0 )   # rad/s
        self.declare_parameter('drift_margin',      0.0 )   
        
        # Load parameters
        self.update_rate        = self.get_parameter('update_rate')      .value
        self.waypoints_json     = self.get_parameter('waypoints_json')   .value
        self.min_linear_speed   = self.get_parameter('min_linear_speed') .value
        self.max_linear_speed   = self.get_parameter('max_linear_speed') .value
        self.min_angular_speed  = self.get_parameter('min_angular_speed').value
        self.max_angular_speed  = self.get_parameter('max_angular_speed').value
        self.drift_margin       = self.get_parameter('drift_margin')     .value

        # Timer for periodic waypoint processing
        self.timer = self.create_timer(1.0 / self.update_rate, self.process_waypoint)

        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Immediately validate the initial values
        init_params = [
                    Parameter('update_rate',        Parameter.Type.DOUBLE,
                            self.update_rate),
                    Parameter('waypoints_json',     Parameter.Type.STRING,
                            self.waypoints_json),
                    Parameter('min_linear_speed',   Parameter.Type.DOUBLE,
                            self.min_linear_speed),
                    Parameter('max_linear_speed',   Parameter.Type.DOUBLE,
                            self.max_linear_speed),
                    Parameter('min_angular_speed',  Parameter.Type.DOUBLE,
                            self.min_angular_speed),
                    Parameter('max_angular_speed',  Parameter.Type.DOUBLE,
                            self.max_angular_speed),
                    Parameter('drift_margin',       Parameter.Type.DOUBLE,
                            self.drift_margin),
                ]
        result = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}.")
        
        # Publisher for OpenLoopPose, which contains robot's velocities and execution time
        self.open_loop_pose_pub = self.create_publisher(
            OpenLoopPose, 
            'puzzlebot_real/open_loop_pose', 
            10
        )

        # Initialize robot's current position (m) and orientation (rad), and start waypoint index
        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0
        self.waypoint_index = 0

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

        # Retrieve target waypoint
        wp = self.waypoints[self.waypoint_index]
        x_target = float(wp.get('x', 0.0))
        y_target = float(wp.get('y', 0.0))

        # Calculate the distance between target and current position
        dx = x_target - self.x_curr
        dy = y_target - self.y_curr
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate the angle to the target waypoint using atan2 (handles the correct quadrant)
        angle_diff = math.atan2(dy, dx)
        angle_diff_signed = normalize_angle_diff(self.theta_curr, angle_diff)        # Normalize the angle difference
        angle_diff = abs(angle_diff_signed)                                          # Angle difference in positive value

        # Check for identical waypoint
        if distance < 1e-6 and angle_diff < 1e-6:
            self.get_logger().error(
                f"Waypoint #{self.waypoint_index+1} is identical to current pose; stopping."
            )
            rclpy.shutdown()
            return
    
        sign = 1.0 if angle_diff_signed >= 0 else -1.0                               # Determine the direction of rotation
        
        # Time-based movement (scale speed to fit within total time)
        if 'total_time' in wp and 'rot_speed' not in wp and 'lin_speed' not in wp:
            total_time = float(wp['total_time'])    # Total time allowed for the movement

            # Compute times at max speed
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
            scale_factor =  raw_time_sum / usable_time
            effective_rotation_speed    = self.max_angular_speed * scale_factor 
            effective_translation_speed = self.max_linear_speed  *  scale_factor 
 
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
            rotation_time    = angle_diff / effective_rotation_speed    
            translation_time = distance   / effective_translation_speed

            # Publish the subcommands (rotation + translation)
            self.publish_subcommands(
                effective_rotation_speed, 
                effective_translation_speed, 
                angle_diff, 
                rotation_time, 
                translation_time, 
                x_target, 
                y_target, 
                sign
            )

        # Speed-based movement  
        elif 'rot_speed' in wp and 'lin_speed' in wp and 'total_time' not in wp:
            # User-defined rotational and linear speeds
            rs = float(wp['rot_speed'])    
            ls = float(wp['lin_speed'])    

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

    def publish_subcommands(self, rot_spd, lin_spd, angle_diff, rot_time, lin_time, x_tar, y_tar, sign):
        """
        Publish the subcommands (rotation and translation) to the 'puzzlebot_real/open_loop_pose' topic as OpenLoopPose messages.
        The robot will rotate and move to the target waypoint.
        """
        if rot_time > 1e-6:
            msg_rot = OpenLoopPose()
            msg_rot.pose.position.x = self.x_curr
            msg_rot.pose.position.y = self.y_curr
            msg_rot.pose.orientation = yaw_to_quaternion(self.theta_curr)
            msg_rot.linear_velocity = 0.0
            msg_rot.angular_velocity = sign * rot_spd
            msg_rot.execution_time = rot_time
            self.open_loop_pose_pub.publish(msg_rot)

        if lin_time > 1e-6:
            msg_trans = OpenLoopPose()
            msg_trans.pose.position.x = self.x_curr
            msg_trans.pose.position.y = self.y_curr
            msg_trans.pose.orientation = yaw_to_quaternion(self.theta_curr + angle_diff_signed(sign, angle_diff))
            msg_trans.linear_velocity = lin_spd
            msg_trans.angular_velocity = 0.0
            msg_trans.execution_time = lin_time
            self.open_loop_pose_pub.publish(msg_trans)

        # Update the current position and orientation of the robot based on the target position
        self.x_curr = x_tar
        self.y_curr = y_tar

        # Adjust the current orientation based on the signed angle difference
        # The modulo operation ensures that the angle stays within the range of [-pi, pi]
        raw_diff = angle_diff if (sign > 0) else -angle_diff
        self.theta_curr = (self.theta_curr + raw_diff + math.pi) % (2 * math.pi) - math.pi

    def parameter_callback(self, params):
        new = {p.name: p.value for p in params}

        if 'update_rate' in new:
            ur = float(new['update_rate'])
            if ur <= 0.0:
                return SetParametersResult(successful=False,
                                           reason="update_rate must be > 0.")
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / ur, self.process_waypoint)
            self.update_rate = ur
            self.get_logger().info(f"update_rate set to {self.update_rate} Hz.")

        if 'min_linear_speed' in new:
            ml = float(new['min_linear_speed'])
            if ml <= 0.0:
                return SetParametersResult(successful=False,
                                           reason="min_linear_speed must be > 0.")
            self.min_linear_speed = ml
            self.get_logger().info(f"min_linear_speed set to {self.min_linear_speed} m/s.")

        if 'max_linear_speed' in new:
            mx = float(new['max_linear_speed'])
            if mx <= self.min_linear_speed:
                return SetParametersResult(successful=False,
                                           reason="max_linear_speed must be > min_linear_speed.")
            self.max_linear_speed = mx
            self.get_logger().info(f"max_linear_speed set to {self.max_linear_speed} m/s.")

        if 'min_angular_speed' in new:
            ma = float(new['min_angular_speed'])
            if ma <= 0.0:
                return SetParametersResult(successful=False,
                                           reason="min_angular_speed must be > 0.")
            self.min_angular_speed = ma
            self.get_logger().info(f"min_angular_speed set to {self.min_angular_speed} rad/s.")

        if 'max_angular_speed' in new:
            mx = float(new['max_angular_speed'])
            if mx <= self.min_angular_speed:
                return SetParametersResult(successful=False,
                                           reason="max_angular_speed must be > min_angular_speed.")
            self.max_angular_speed = mx
            self.get_logger().info(f"max_angular_speed set to {self.max_angular_speed} rad/s.")

        if 'drift_margin' in new:
            dm = float(new['drift_margin'])
            if not (0.0 <= dm < 1.0):
                return SetParametersResult(successful=False,
                                           reason="drift_margin must be in [0.0, 1.0).")
            self.drift_margin = dm
            self.get_logger().info(f"drift_margin set to {self.drift_margin}.")

        if 'waypoints_json' in new:
            raw = new['waypoints_json']
            try:
                wps = json.loads(raw)
            except json.JSONDecodeError as e:
                return SetParametersResult(successful=False,
                                           reason=f"bad JSON in waypoints_json: {e}.")
            if not isinstance(wps, list):
                return SetParametersResult(successful=False,
                                           reason="waypoints_json must be a JSON array.")
            for idx, wp in enumerate(wps, start=1):
                if 'total_time' in wp:
                    try:
                        t = float(wp['total_time'])
                    except (ValueError, TypeError):
                        return SetParametersResult(successful=False,
                                                   reason=f"Waypoint {idx}: total_time must be a number.")
                    if t <= 0.0:
                        return SetParametersResult(successful=False,
                                                   reason=f"Waypoint {idx}: total_time must be > 0.")
                if 'lin_speed' in wp:
                    try:
                        ls = float(wp['lin_speed'])
                    except (ValueError, TypeError):
                        return SetParametersResult(successful=False,
                                                   reason=f"Waypoint {idx}: lin_speed must be a number.")
                    if not (self.min_linear_speed <= ls <= self.max_linear_speed):
                        return SetParametersResult(successful=False,
                                                   reason=(
                                                       f"Waypoint {idx}: lin_speed {ls:.2f} "
                                                       f"must be in [{self.min_linear_speed:.2f}, {self.max_linear_speed:.2f}]."
                                                   ))
                if 'rot_speed' in wp:
                    try:
                        rs = float(wp['rot_speed'])
                    except (ValueError, TypeError):
                        return SetParametersResult(successful=False,
                                                   reason=f"Waypoint {idx}: rot_speed must be a number.")
                    if not (self.min_angular_speed <= rs <= self.max_angular_speed):
                        return SetParametersResult(successful=False,
                                                   reason=(
                                                       f"Waypoint {idx}: rot_speed {rs:.2f} "
                                                       f"must be in [{self.min_angular_speed:.2f}, {self.max_angular_speed:.2f}]."
                                                   ))
            self.waypoints = wps
            self.get_logger().info("waypoints_json updated and validated.")

        return SetParametersResult(successful=True)

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