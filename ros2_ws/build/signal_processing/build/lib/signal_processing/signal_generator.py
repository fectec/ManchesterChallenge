import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class SignalPublisher(Node):
    
    def __init__(self):
        
        super().__init__('signal_publisher')
        self.publisher_ = self.create_publisher(Float32, 'signal', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):

        signal_value = math.sin(self.angle)

        msg = Float32()
        msg.data = signal_value

        self.publisher_.publish(msg)

        self.angle += 0.1

        if self.angle > 2 * math.pi:
            self.angle = 0.0

        
        self.get_logger().info(f'Publishing: {signal_value}')

def main(args=None):

    rclpy.init(args=args)
    node = SignalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()