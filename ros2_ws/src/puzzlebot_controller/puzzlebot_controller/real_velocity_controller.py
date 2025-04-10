#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
import numpy as np

class VelocityController(Node):
    def __init__(self):
        super().__init__('real_velocity_controller')

        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_separation", 0.18)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        # Publishers for left and right wheel velocities (Float32 values from 0.0 to 5.0)
        self.pub_wheel_left = self.create_publisher(Float32, '/VelocitySetL', 10)
        self.pub_wheel_right = self.create_publisher(Float32, '/VelocitySetR', 10)
        # Subscribe to twist commands
        self.create_subscription(TwistStamped, "puzzlebot_controller/cmd_vel", self.velocity_callback, 10)

        # Define the speed conversion matrix
        self.speed_conversion = np.array([
            [self.wheel_radius / 2, self.wheel_radius / 2],
            [self.wheel_radius / self.wheel_separation, -self.wheel_radius / self.wheel_separation]
        ])

    def velocity_callback(self, msg):
        # Create the robot speed vector [linear; angular]
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        # Compute the wheel speeds using the inverse conversion matrix
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)
        # Clamp the wheel speeds between 0.0 and 5.0
        wheel_speed = np.clip(wheel_speed, -5.0, 5.0)
        
        # Create and publish the left and right wheel velocity commands
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = float(wheel_speed[0, 0])
        right_msg.data = float(wheel_speed[1, 0])
        
        self.pub_wheel_left.publish(left_msg)
        self.pub_wheel_right.publish(right_msg)
        self.get_logger().info(f"Published wheel speeds: left = {left_msg.data:.2f}, right = {right_msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()