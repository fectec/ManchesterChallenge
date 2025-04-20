#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class VelocityController(Node):
    """
    Converts the robot's linear and angular velocity commands into wheel speeds.
    The robot's velocity is controlled through the 'cmd_vel' topic, and the calculated 
    wheel velocities are published to the 'simple_velocity_controller/commands' topic.
    """
    def __init__(self):
        super().__init__("velocity_controller")

        # Declare and get parameters
        self.declare_parameter("wheel_radius", 0.05)        # Radius of the wheel (m)      
        self.declare_parameter("wheel_separation", 0.18)    # Distance between the wheels (m)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        # Publisher to send wheel speeds
        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        # Subscriber to receive velocity commands
        self.create_subscription(TwistStamped, "puzzlebot_controller/cmd_vel", self.velocity_callback, 10)

        # Matrix for velocity conversion from robot to wheel speeds
        self.speed_conversion = np.array([[self.wheel_radius / 2, self.wheel_radius / 2], 
                                          [self.wheel_radius / self.wheel_separation, -self.wheel_radius / self.wheel_separation]])
        
    def velocity_callback(self, msg):
        """
        Callback to convert linear and angular velocities to wheel speeds and publish them.
        """
        # Get linear and angular velocities from 'cmd_vel' message
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        # Apply inverse kinematics to calculate wheel speeds
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()