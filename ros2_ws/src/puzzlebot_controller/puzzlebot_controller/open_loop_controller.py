#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from custom_interfaces.msg import OpenLoopPose

IDLE = 0
EXECUTING = 1

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(OpenLoopPose, 'pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.state = IDLE
        self.cmd_queue = []
        self.current_cmd = None
        self.cmd_start_time = None

    def pose_callback(self, msg):
        self.cmd_queue.append(msg)
        self.get_logger().info(
            f"Queued command: lin={msg.linear_velocity:.2f}, ang={msg.angular_velocity:.2f}, time={msg.execution_time:.2f}"
        )

    def fsm_loop(self):
        twist = Twist()
        if self.state == IDLE:
            if not self.cmd_queue:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            else:
                self.current_cmd = self.cmd_queue.pop(0)
                self.cmd_start_time = self.get_clock().now()
                self.state = EXECUTING
        elif self.state == EXECUTING:
            elapsed = (self.get_clock().now() - self.cmd_start_time).nanoseconds * 1e-9
            if elapsed < self.current_cmd.execution_time:
                twist.linear.x = self.current_cmd.linear_velocity
                twist.angular.z = self.current_cmd.angular_velocity
                self.cmd_vel_pub.publish(twist)
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("Completed command")
                self.current_cmd = None
                self.cmd_start_time = None
                self.state = IDLE

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()