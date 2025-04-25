#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from custom_interfaces.msg import OpenLoopPose

# Define the states for the finite state machine (FSM)
IDLE = 0  
EXECUTING = 1  

class OpenLoopPointController(Node):
    """
    A ROS2 node that acts as an open-loop controller for a robot. It subscribes to pose commands 
    (OpenLoopPose messages), which contain the robot's desired linear and angular velocities 
    along with their execution times. The node uses a finite state machine (FSM) to process 
    these commands, publishing velocity commands to the 'cmd_vel' topic.

    The node operates in two states:
    - IDLE: The node is waiting for new commands to be queued.
    - EXECUTING: The node is actively executing a command from the queue, updating the robot's 
      velocity as per the command's linear and angular velocities.
    """
    
    def __init__(self):
        super().__init__('open_loop_point_controller')

        # Declare and retrieve parameters 
        self.declare_parameter('update_rate', 50.0)     # The frequency (Hz) at which the FSM is executed
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        # Publisher for the Twist message, sending commands to 'cmd_vel' topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to receive OpenLoopPose messages
        self.open_loop_pose_sub = self.create_subscription(OpenLoopPose, '/puzzlebot_real/open_loop_pose', self.open_loop_pose_callback, 10)

        # Initialize FSM state to IDLE
        self.state = IDLE

        # Queue to store incoming pose commands
        self.cmd_queue = []

        # Variables to hold the current command being executed and its start time
        self.current_cmd = None
        self.cmd_start_time = None

        # Create a timer that triggers the finite state machine loop every certain period
        self.timer = self.create_timer(1.0 / update_rate, self.fsm_loop)

        self.get_logger().info("OpenLoopPointController Start.")  

    def open_loop_pose_callback(self, msg):
        # Add the received pose message to the queue for execution
        self.cmd_queue.append(msg)
        
        self.get_logger().info(
            f"Queued command: LIN={msg.linear_velocity:.2f} ANG={msg.angular_velocity:.2f} TIME={msg.execution_time:.2f}"
        )

    def fsm_loop(self):
        # Create an empty Twist message to send to 'cmd_vel'
        twist = Twist()
        
        # Check the current state of the FSM
        if self.state == IDLE:
            # If the queue is empty, stop the robot
            if not self.cmd_queue:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            else:
                # Otherwise, pop the first command from the queue and start executing it
                self.current_cmd = self.cmd_queue.pop(0)
                self.cmd_start_time = self.get_clock().now()
                self.state = EXECUTING
        elif self.state == EXECUTING:
            # Calculate how much time has passed since the command started
            elapsed = (self.get_clock().now() - self.cmd_start_time).nanoseconds * 1e-9
            
            # If the execution time for the command is not completed, continue sending the command
            if elapsed < self.current_cmd.execution_time:
                twist.linear.x = self.current_cmd.linear_velocity
                twist.angular.z = self.current_cmd.angular_velocity
                self.cmd_vel_pub.publish(twist)
            else:
                # If the command has finished, stop the robot and log the completion
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("Completed command.")
                
                # Reset variables and return to IDLE state
                self.current_cmd = None
                self.cmd_start_time = None
                self.state = IDLE

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopPointController()
    
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