import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SignalProcessorNode(Node):

    def __init__(self):
        
        super().__init__('signal_processor')

        # Parameters for signal processing

        self.alpha = 0.5
        self.amplitude_reduction = 0.5
        self.phase_shift = math.pi

        # Last received values

        self.current_time = None
        self.current_signal = None

        # Create subscribers

        self.time_subscriber = self.create_subscription(
            Float32,
            'time',
            self.time_callback,
            10
        )

        self.signal_subscriber = self.create_subscription(
            Float32,
            'signal',
            self.signal_callback,
            10
        )

        # Create publisher for processed signal

        self.processed_publisher = self.create_publisher(
            Float32,
            'proc_signal',
            10
        )

        # Create timer for publishing at 10Hz

        self.timer = self.create_timer(0.1, self.process_callback)

        self.get_logger().info('Signal Processor Node has been started')
        
        self.get_logger().info(
            f'Parameters: alpha={self.alpha}, '
            f'amplitude reduction={self.amplitude_reduction}, '
            f'phase shift={self.phase_shift} radians'
        )

    def time_callback(self, msg): 
        self.current_time = msg.data

    def signal_callback(self, msg):
        self.current_signal = msg.data

    def process_callback(self):

        if self.current_time is None or self.current_signal is None:
            return

        # Modify the signal

        processed_signal = self.amplitude_reduction * (self.current_signal * math.cos(self.phase_shift) + math.cos(math.asin(self.current_signal)) * math.sin(self.phase_shift)) + self.alpha

        # Create and publish message

        output_msg = Float32()
        output_msg.data = processed_signal
        self.processed_publisher.publish(output_msg)

        # Log the values

        self.get_logger().info(
            f'Time: {self.current_time:.2f}, '
            f'Original Signal: {self.current_signal:.2f}, '
            f'Processed Signal: {processed_signal:.2f}'
        )

def main(args=None):

    rclpy.init(args=args)
    signal_processor = SignalProcessorNode()
    
    try:
        rclpy.spin(signal_processor)
    except KeyboardInterrupt:
        pass
    finally:
        signal_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()