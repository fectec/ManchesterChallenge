import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class SignalGeneratorNode(Node):
    
    def __init__(self):
        
        super().__init__('signal_generator')

        # Publishers for signal and time

        self.signal_publisher = self.create_publisher(
            Float32, 
            'signal',
            10
        )

        self.time_publisher = self.create_publisher(
            Float32,
            'time',
            10
        )

        # Create timer for publishing at 10Hz

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize time counter

        self.t = 0.0

        self.get_logger().info('Signal Generator Node has been started')

    def timer_callback(self):

        # Create message objects

        signal_msg = Float32()
        time_msg = Float32()

        # Calculate sine wave

        signal_msg.data = float(math.sin(self.t))
        time_msg.data = float(self.t)

        # Publish messsages

        self.signal_publisher.publish(signal_msg)
        self.time_publisher.publish(time_msg)

        # Log the values

        self.get_logger().info(f'Time: {self.t:.2f}, Signal: {signal_msg.data:.2f}')

        # Increment time

        self.t += 0.1

def main(args=None):

    rclpy.init(args=args)
    signal_generator = SignalGeneratorNode()

    try:
        rclpy.spin(signal_generator)
    except KeyboardInterrupt:
        pass
    finally:
        signal_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()