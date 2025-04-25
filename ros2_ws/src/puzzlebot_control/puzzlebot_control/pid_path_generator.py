#!/usr/bin/env python3

import rclpy
import json
import math

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

        raw = self.get_parameter('waypoints_json').value
        self.min_dist = self.get_parameter('min_waypoint_distance').value
        self.max_dist = self.get_parameter('max_waypoint_distance').value

        try:
            self.waypoints = json.loads(raw)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse waypoints_json: {e}.")
            rclpy.shutdown()
            return
        
        if not isinstance(self.waypoints, list) or len(self.waypoints) < 3:
            self.get_logger().warning(
                'Parameter "waypoints_json" must be a JSON array with at least 3 points.'
            )
            rclpy.shutdown()
            return
        
        # Initialize waypoint index
        self.waypoint_index = 0
        
        # Create a service to provide the next waypoint
        self.next_waypoint_srv = self.create_service(
            NextPIDWaypoint, 
            '/puzzlebot_real/point_PID/next_PID_waypoint', 
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
        
        try:
            x = float(wp['x'])
            y = float(wp['y'])
        except (KeyError, ValueError, TypeError):
            self.get_logger().error(f'Invalid waypoint format: {wp}.')
            response.completed = True       
            response.goal = PIDGoalPose() 
            self.create_timer(1.0, self.shutdown_node)  
            return response
        
        # Check distance constrains
        if current_index > 0:
            prev = self.waypoints[current_index-1]
            px, py = float(prev['x']), float(prev['y'])
            dist = math.hypot(x - px, y - py)
        else:
            dist = math.hypot(x, y)

        if dist < self.min_dist or dist > self.max_dist:
            self.get_logger().warn(
                f'Waypoint {current_index+1} at distance {dist:.3f} '
                f'outside allowed range [{self.min_dist}, {self.max_dist}]. Ending.'
            )
            response.completed = True
            response.goal = PIDGoalPose()
            self.create_timer(1.0, self.shutdown_node)
            return response
            
        # Create and populate the response
        goal = PIDGoalPose()
        goal.pose.position = Point(x=x, y=y, z=0.0)
        goal.theta = 0.0
        goal.pose.orientation = yaw_to_quaternion(0.0)
        
        response.goal = goal
        response.completed = False
        response.waypoint_id = current_index
        
        self.get_logger().info(f'Providing waypoint {current_index+1}/{len(self.waypoints)}: x={x}, y={y}.')
        
        # Increment the index if the previous was reached
        if request.previous_reached:
            self.waypoint_index += 1
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PIDPathGenerator()

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