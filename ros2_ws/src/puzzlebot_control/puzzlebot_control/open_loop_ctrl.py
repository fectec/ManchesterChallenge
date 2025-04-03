import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.state = 0
        self.start_time = self.get_clock().now()

        self.linear_speed = 0.2
        self.angular_speed = 0.3

        self.forward_time = 2.0 / self.linear_speed
        self.rotate_time = 3.14159 / self.angular_speed
        self.backward_time = self.forward_time

        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds * 1e-9

        cmd = Twist()

        if self.state == 0:
            cmd.linear.x = self.linear_speed
            if elapsed_time >= self.forward_time:
                self.state = 1
                self.start_time = now

        elif self.state == 1:
            cmd.angular.z = self.angular_speed
            if elapsed_time >= self.rotate_time:
                self.state = 2
                self.start_time = now

        elif self.state == 2:
            cmd.linear.x = self.linear_speed
            if elapsed_time >= self.backward_time:
                self.state = 3
                self.start_time = now

        elif self.state == 3:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()