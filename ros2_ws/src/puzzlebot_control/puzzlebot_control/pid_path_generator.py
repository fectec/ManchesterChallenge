#!/usr/bin/env python3

import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from puzzlebot_utils.utils.math_helpers import yaw_to_quaternion

from geometry_msgs.msg import Point

from custom_interfaces.msg import PIDGoalPose
from custom_interfaces.srv import NextPIDWaypoint

class PIDPathGenerator(Node):
    """
    Provides waypoints as a service for a PID controller.

    Waypoints are loaded from the ROS parameter 'waypoints_json' as a JSON array of
    objects containing 'x' and 'y' coordinates.

    Consecutive waypoints must lie within [min_waypoint_distance, max_waypoint_distance].

    The node serves each waypoint via the 'point_pid/next_pid_waypoint' service when requested.
    """

    def __init__(self):
        super().__init__('pid_path_generator')
        
        # Declare parameters
        self.declare_parameter('waypoints_json',       '[]')
        self.declare_parameter('min_waypoint_distance', 0.0)            # m
        self.declare_parameter('max_waypoint_distance', float('inf'))   # m

        # Retrieve parameters
        self.waypoints_json        = self.get_parameter('waypoints_json').value
        self.min_waypoint_distance = self.get_parameter('min_waypoint_distance').value
        self.max_waypoint_distance = self.get_parameter('max_waypoint_distance').value

        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Immediately validate the initial values
        init_params = [
            Parameter('waypoints_json',       Parameter.Type.STRING, self.waypoints_json),
            Parameter('min_waypoint_distance', Parameter.Type.DOUBLE, self.min_waypoint_distance),
            Parameter('max_waypoint_distance', Parameter.Type.DOUBLE, self.max_waypoint_distance),
        ]
        result = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}.")
        
        # Initialize waypoint index
        self.waypoint_index = 0
        
        # Create a service to provide the next waypoint
        self.create_service(
            NextPIDWaypoint, 
            'point_pid/next_pid_waypoint', 
            self.next_waypoint_callback
        )
        
        self.get_logger().info(f'PIDPathGenerator Start.')

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
        x, y = float(wp['x']), float(wp['y'])
            
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
    
    def shutdown_node(self):
        """
        Cleanly shutdown the node once all waypoints are served.
        """
        self.get_logger().info('PIDPathGenerator shutting down.')
        rclpy.shutdown()

    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Validates and applies updated node parameters."""
        new = {p.name: p.value for p in params}

        if 'min_waypoint_distance' in new:
            md = float(new['min_waypoint_distance'])
            if md < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="min_waypoint_distance must be ≥ 0."
                )
            self.min_waypoint_distance = md
            self.get_logger().info(f"min_waypoint_distance updated: {self.min_waypoint_distance} m.")

        if 'max_waypoint_distance' in new:
            Md = float(new['max_waypoint_distance'])
            if Md <= self.min_waypoint_distance:
                return SetParametersResult(
                    successful=False,
                    reason="max_waypoint_distance must be > min_waypoint_distance."
                )
            self.max_waypoint_distance = Md
            self.get_logger().info(f"max_waypoint_distance updated: {self.max_waypoint_distance} m.")

        if 'waypoints_json' in new:
            raw = new['waypoints_json']
            try:
                wps = json.loads(raw)
            except json.JSONDecodeError as e:
                return SetParametersResult(
                    successful=False,
                    reason=f"bad JSON in waypoints_json: {e}."
                )
            if not isinstance(wps, list) or len(wps) < 1:
                return SetParametersResult(
                    successful=False,
                    reason="waypoints_json must be a JSON array with ≥ 1 waypoint."
                )

            for i, wp in enumerate(wps, start=1):
                if not isinstance(wp, dict) or 'x' not in wp or 'y' not in wp:
                    return SetParametersResult(
                        successful=False,
                        reason=f"Waypoint #{i} must be an object with 'x' and 'y'."
                    )
                try:
                    x = float(wp['x'])
                    y = float(wp['y'])
                except (ValueError, TypeError):
                    return SetParametersResult(
                        successful=False,
                        reason=f"Waypoint #{i}: 'x' and 'y' must be numbers."
                    )

                if i == 1:
                    dx, dy = x, y
                else:
                    px = float(wps[i-2]['x'])
                    py = float(wps[i-2]['y'])
                    dx, dy = x - px, y - py

                d = math.hypot(dx, dy)
                if d < self.min_waypoint_distance or d > self.max_waypoint_distance:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"Waypoint #{i} is {d:.3f} m from waypoint #{i-1} "
                            f"(allowed [{self.min_waypoint_distance:.3f}, {self.max_waypoint_distance:.3f}])."
                        )
                    )

            self.waypoints_json = raw
            self.waypoints      = wps
            self.get_logger().info(f"waypoints_json updated with: {len(self.waypoints)} waypoints.")

        return SetParametersResult(successful=True)

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