#!/usr/bin/env python3

import rclpy
import json
import math
import sys

from rclpy.node import Node

from puzzlebot_utils.utils.math_helpers import yaw_to_quaternion

from geometry_msgs.msg import Point

from custom_interfaces.msg import PIDGoalPose
from custom_interfaces.srv import NextPIDWaypoint

class PIDPathGenerator(Node):
    """
    Provides waypoints as a service for a PID controller.
    Waypoints are loaded from the ROS parameter 'waypoints_json' as a JSON array of
    objects containing 'x' and 'y' coordinates. The node serves each waypoint via
    the '/puzzlebot_real/point_PID/next_PID_waypoint' service when requested by the PID controller.
    """

    def __init__(self):
        super().__init__('pid_path_generator')
        
        # Declare and parse operating parameters
        self.declare_parameter('waypoints_json', '[]')
        self.declare_parameter('min_waypoint_distance', 0.0)
        self.declare_parameter('max_waypoint_distance', float('inf'))

        raw           = self.get_parameter('waypoints_json').value
        self.min_dist = self.get_parameter('min_waypoint_distance').value
        self.max_dist = self.get_parameter('max_waypoint_distance').value

        # Validate min/max distances
        if self.min_dist < 0.0:
            raise ValueError(f"'min_waypoint_distance' must be ≥ 0.0 (got {self.min_dist}).")

        if self.max_dist <= self.min_dist:
            raise ValueError(
                f"'max_waypoint_distance' ({self.max_dist}) must be > "
                f"'min_waypoint_distance' ({self.min_dist})."
            )

        # Parse the JSON array
        try:
            self.waypoints = json.loads(raw)
        except json.JSONDecodeError as e:
            raise ValueError(f"Failed to parse waypoints_json: {e}.")

        # Validate overall structure
        if not isinstance(self.waypoints, list):
            raise TypeError("`waypoints_json` must be a JSON array.")

        if len(self.waypoints) < 1:
            raise ValueError("`waypoints_json` must contain at least one waypoint.")

        # Validate each waypoint entry
        for idx, wp in enumerate(self.waypoints, start=1):
            if not isinstance(wp, dict):
                raise TypeError(f"Waypoint #{idx} is not an object: {wp!r}.")
            if 'x' not in wp or 'y' not in wp:
                raise KeyError(f"Waypoint #{idx} missing 'x' or 'y': {wp!r}.")

            try:
                x = float(wp['x'])
                y = float(wp['y'])
            except (ValueError, TypeError):
                raise ValueError(f"Waypoint #{idx}: 'x' and 'y' must be numbers (got {wp!r}).")

            if idx == 1:
                px, py = 0.0, 0.0
            else:
                prev = self.waypoints[idx-2]
                px, py = float(prev['x']), float(prev['y'])

            dist = math.hypot(x - px, y - py)
            if dist < self.min_dist or dist > self.max_dist:
                raise ValueError(
                    f"Waypoint #{idx} is {dist:.3f}m from waypoint #{idx-1} "
                    f"(allowed [{self.min_dist:.3f}, {self.max_dist:.3f}])."
        )

        
        # Initialize waypoint index
        self.waypoint_index = 0
        
        # Create a service to provide the next waypoint
        self.next_waypoint_srv = self.create_service(
            NextPIDWaypoint, 
            'puzzlebot_real/point_PID/next_PID_waypoint', 
            self.next_waypoint_callback
        )
        
        self.get_logger().info(f'PIDPathGenerator Start with {len(self.waypoints)} waypoints.')

    def shutdown_node(self):
        # Cleanly shutdown the node once all waypoints are served
        self.get_logger().info('PIDPathGenerator shutting down.')
        rclpy.shutdown()

    def next_waypoint_callback(self, request, response):
        """
        Service callback that provides the next waypoint when requested.
        
        If previous_reached is False, it will resend the current waypoint.
        If all waypoints have been processed, it will set the 'completed' flag in the response.
        """
        # If PID controller didn't reach the previous point, resend the same point
        if not request.previous_reached and self.waypoint_index > 0:
            self.get_logger().warn(f'Previous waypoint not reached. Resending waypoint {self.waypoint_index}.')
            current_index = self.waypoint_index - 1  # Use the previous index
        else:
            current_index = self.waypoint_index
            
        # Check if we've processed all waypoints
        if current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints have been processed.')
            response.completed = True                   # Mark as completed to end the sequence
            response.goal = PIDGoalPose()               # Empty goal
            self.create_timer(1.0, self.shutdown_node)  # Schedule shutdown of the node
            return response
        
        # Get the current waypoint
        wp = self.waypoints[current_index]
        x = float(wp['x'])
        y = float(wp['y'])
            
        # Create and populate the response
        goal = PIDGoalPose()
        goal.pose.position = Point(x=x, y=y, z=0.0)
        goal.theta = 0.0
        goal.pose.orientation = yaw_to_quaternion(0.0)
        
        response.goal = goal
        response.completed = False
        response.waypoint_id = current_index
        
        self.get_logger().info(
            f'Providing waypoint {current_index+1}/{len(self.waypoints)}: x={x}, y={y}.'
        )
        
        # Increment the index if the previous was reached
        if request.previous_reached:
            self.waypoint_index += 1
            
        return response

def main(args=None):
    rclpy.init(args=args)

    try:
        node = PIDPathGenerator()
    except Exception as e:
        print(f"[FATAL] PIDPathGenerator failed to initialize: {e}.", file=sys.stderr)
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