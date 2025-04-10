#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from custom_interfaces.msg import OpenLoopPose

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        
        # Publisher to send velocity commands to the robot
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber to receive pose commands (custom OpenLoopPose messages)
        self.pose_sub = self.create_subscription(
            OpenLoopPose,
            'pose',
            self.pose_callback,
            10
        )
        
        # Timer for the control loop; runs at 10 Hz (0.1 s period)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Current active command (of type OpenLoopPose) and its start time
        self.current_command = None
        self.cmd_start_time = None

    def pose_callback(self, msg):
        """
        Callback to handle a new pose command.
        When a new command is received, it is stored and the command start time is reset.
        This command contains the control_mode, target speeds, and execution_time.
        """
        self.get_logger().info(
            f"Received new pose command: mode={msg.control_mode}, "
            f"lin_vel={msg.target_linear_velocity:.2f}, ang_vel={msg.target_angular_velocity:.2f}, "
            f"time={msg.execution_time:.2f}"
        )
        self.current_command = msg
        self.cmd_start_time = self.get_clock().now()

    def control_loop(self):
        """
        Main control loop.
        
        - If no command is active, a zero-velocity Twist is published.
        - If a command is active, compute the elapsed time.
          - If the elapsed time is less than the execution_time from the command,
            publish constant velocities (from the command).
          - If the command’s time is up, publish a zero-velocity Twist and clear the command.
        """
        twist = Twist()
        
        if self.current_command is None:
            # No active command: ensure the robot is stopped.
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # Calculate elapsed time (in seconds) since the current command started
        now = self.get_clock().now()
        elapsed = (now - self.cmd_start_time).nanoseconds * 1e-9

        if elapsed < self.current_command.execution_time:
            # Command is still active; publish the constant velocities.
            twist.linear.x = self.current_command.target_linear_velocity
            twist.angular.z = self.current_command.target_angular_velocity
            self.cmd_vel_pub.publish(twist)
        else:
            # Command execution has finished; publish zero velocity and reset the state.
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Completed command")
            self.current_command = None
            self.cmd_start_time = None

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