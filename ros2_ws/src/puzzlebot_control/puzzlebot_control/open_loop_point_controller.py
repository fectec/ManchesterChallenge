#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from custom_interfaces.msg import OpenLoopPose

# Define the states for the finite state machine (FSM)
IDLE = 0  
EXECUTING = 1  

class OpenLoopPointController(Node):
    """
    Node that acts as an open-loop controller for a robot. It subscribes to pose commands 
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

        # Declare parameters 
        self.declare_parameter('update_rate', 100.0)    # Hz

        # Load parameters
        self.update_rate = self.get_parameter('update_rate').value

        # Timer that triggers the finite state machine loop every certain period
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_loop)

        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate', Parameter.Type.DOUBLE, self.update_rate),
        ]
        result = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}.")

        # Publisher for the Twist message, sending commands to 'cmd_vel' topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to receive OpenLoopPose messages
        self.create_subscription(
            OpenLoopPose, 
            'puzzlebot_real/open_loop_pose', 
            self.open_loop_pose_callback, 
            10
        )

        # Initialize FSM state to IDLE
        self.state = IDLE

        # Queue to store incoming pose commands
        self.cmd_queue = []

        # Variables to hold the current command being executed and its start time
        self.current_cmd = None
        self.cmd_start_time = None

        self.get_logger().info("OpenLoopPointController Start.")  

    def open_loop_pose_callback(self, msg):
        # Add the received pose message to the queue for execution
        self.cmd_queue.append(msg)
        
        self.get_logger().info(
            f"Queued command: lin={msg.linear_velocity:.2f} ang={msg.angular_velocity:.2f} time={msg.execution_time:.2f}"
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

    def parameter_callback(self, params):
        new = {p.name: p.value for p in params}

        if 'update_rate' in new:
            ur = float(new['update_rate'])
            if ur <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="update_rate must be > 0."
                )
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / ur, self.fsm_loop)
            self.update_rate = ur
            self.get_logger().info(f"update_rate set to {self.update_rate} Hz.")

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = OpenLoopPointController()
    except Exception as e:
        print(f"[FATAL] OpenLoopPointController failed to initialize: {e}.", file=sys.stderr)
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