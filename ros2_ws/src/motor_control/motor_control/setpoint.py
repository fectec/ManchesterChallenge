import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class SetpointPublisherNode(Node):

    def __init__(self):
        
        super().__init__('setpoint_publisher')

        # Parameters
        
        # Sine wave amplitude

        self.declare_parameter('amplitude', 2.0)
        
        # Sine wave omega

        self.declare_parameter('omega', 1.0)

        # Parameters as variables

        self.amplitude = self.get_parameter('amplitude').value
        self.omega = self.get_parameter('omega').value

        # Create a publisher and timer for the signal

        timer_period = 0.1 # seconds
        
        self.signal_publisher = self.create_publisher(Float32, 'setpoint', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Create a messages and variables to be used

        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Node started

        self.get_logger().info("SetPoint Node Started \U0001F680")

    # Timer callback: Generate and publish sine wave signal

    def timer_callback(self):

        # Calculate elapsed time

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Generate sine wave signal

        self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        
        # Publish the signal

        self.signal_publisher.publish(self.signal_msg)

# Main

def main(args=None):

    rclpy.init(args=args)

    setpoint = SetpointPublisherNode()

    try:
        rclpy.spin(setpoint)
    except KeyboardInterrupt:
        pass
    finally:
        setpoint.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()