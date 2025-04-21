#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import math

from custom_interfaces.msg import PIDGoalPose
from custom_interfaces.srv import GoalReached
from geometry_msgs.msg import Point, Quaternion

# ========================
# Utility Functions
# ========================
def yaw_to_quaternion(yaw: float) -> Quaternion:
    # Convert a yaw angle (radians) into a quaternion message
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

# ========================
# PID Path Generator Node
# ========================
class PIDPathGenerator(Node):
    """
    Sequentially publishes goal poses for a PID controller.
    Waypoints are loaded from the ROS parameter 'waypoints_json' as a JSON array of
    objects containing 'x' and 'y' coordinates. The node publishes each goal to '/goal'
    and waits for confirmation via the '/goal_completed' service before proceeding.
    Requires at least three waypoints; otherwise the node will exit.
    """

    def __init__(self):
        super().__init__('pid_path_generator')
        
        # Declare and parse operating parameters
        self.declare_parameter('waypoints_json', '[]')
        self.declare_parameter('timer_interval', 0.5)
        raw = self.get_parameter('waypoints_json').value
        self.interval = self.get_parameter('timer_interval').value

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
        
        # Publisher for PIDGoalPose messages
        self.goal_pub = self.create_publisher(PIDGoalPose, '/goal', 10)

        # Service client for goal completion confirmation
        self.client = self.create_client(GoalReached, '/goal_completed')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /goal_completed service...')

        self.index = 0
        self.timer = self.create_timer(self.interval, self.publish_next_goal) 

    def publish_next_goal(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info('All waypoints processed. Exiting.')
            self.timer.cancel()
            return          
        
        wp = self.waypoints[self.index]
        try:
            x = float(wp['x'])
            y = float(wp['y'])
        except (KeyError, ValueError, TypeError):
            self.get_logger().error(f'Invalid waypoint format: {wp}.')
            self.timer.cancel()
            return
        
        # Construct and publish the goal message
        msg = PIDGoalPose()
        msg.pose.position = Point(x=x, y=y, z=0.0)
        msg.theta = 0.0
        msg.pose.orientation = yaw_to_quaternion(0.0)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Published goal {self.index+1}/{len(self.waypoints)}: x={x}, y={y}')

        # Call the confirmation service and wait
        req = GoalReached.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result and result.success:
            self.get_logger().info('Goal reached confirmed.')
            self.index += 1
        else:
            self.get_logger().warn('Goal confirmation failed or timed out. Stopping.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = PIDPathGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()