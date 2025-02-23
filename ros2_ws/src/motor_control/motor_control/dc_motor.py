import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DCMotorNode(Node):

    def __init__(self):

        super().__init__('dc_motor')

        # Parameters
        
        # System sample time in seconds

        self.declare_parameter('sys_sample_time', 0.02)
        
        # System gain K

        self.declare_parameter('sys_gain_K', 1.75)

        # System time constant Tau

        self.declare_parameter('sys_tau_T', 0.5)

        # System initial conditions

        self.declare_parameter('sys_initial_conditions', 0.0)

        # Parameters as variables

        self.sample_time = self.get_parameter('sys_sample_time').value
        self.gain = self.get_parameter('sys_gain_K').value
        self.time_constant = self.get_parameter('sys_tau_T').value
        self.initial_conditions = self.get_parameter('sys_initial_conditions').value

        # Variables

        self.input_u = 0.0
        self.output_y = self.initial_conditions
        
        # Subscribers, publishers & timers

        self.motor_speed_pub = self.create_publisher(Float32, 'motor_speed_y', 10)
        self.motor_input_sub = self.create_subscription(Float32, 'motor_input_u', self.input_callback, 10)
        self.timer = self.create_timer(self.sample_time, self.timer_callback) 

        # Messages

        self.motor_output_msg = Float32()

        # Node started 

        self.get_logger().info('Dynamical System Node Started \U0001F680')   
        
    # Timer callback

    def timer_callback(self):    

        # DC Motor Simulation

        # DC Motor Equation 𝑦[𝑘+1] = 𝑦[𝑘] + ((−1/𝜏) 𝑦[𝑘] + (𝐾/𝜏) 𝑢[𝑘]) 𝑇_𝑠

        self.output_y += (-1.0 / self.time_constant * self.output_y + self.gain / self.time_constant * self.input_u) * self.sample_time 
        
        # Publish the result 

        self.motor_output_msg.data = self.output_y
        self.motor_speed_pub.publish(self.motor_output_msg)

    # Subscriber callback

    def input_callback(self, input_signal):

        self.input_u = input_signal.data

# Main 

def main(args=None):

    rclpy.init(args=args)

    node = DCMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()