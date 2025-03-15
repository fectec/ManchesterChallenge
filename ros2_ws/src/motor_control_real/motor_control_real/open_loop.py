#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


class OpenLoopControllerNode(Node):

    def __init__(self):
        super().__init__("OpenLoopControllerNode")

        # Declarar parámetros
        self.declare_parameter("timer_period", 0.1)         # Periodo del timer
        self.declare_parameter("duty_cycle", 8.0)              # Amplitud de la señal

        self.timer_period = self.get_parameter("timer_period").value # Obtener el valor del periodo del timer
        self.duty_cycle = self.get_parameter("duty_cycle").value    # Obtener el valor del duty_cycle

        # Definir un QoS profile confiable según la plantilla
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Crear publicador y timer
        self.signal_publisher = self.create_publisher(Float32, "pwm", qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.signal_msg = Float32()

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("Setpoint Node Started 🚀")

    def timer_callback(self):

        self.signal_msg.data = self.duty_cycle
        # Publicar el mensaje de duty_cycle
        self.signal_publisher.publish(self.signal_msg)

    def parameter_callback(self, params):
        for param in params:
            if param.name == "timer_period":
                if param.value < 0.0:
                    self.get_logger().warn("El periodo del timer no puede ser negativo.")
                    return SetParametersResult(successful=False, reason="timer_period cannot be negative.")
                self.timer_period = param.value
                self.timer.timer_period = self.timer_period
            elif param.name == "duty_cycle":
                if param.value < 0.0:
                    self.get_logger().warn("El duty_cycle no puede ser negativo.")
                    return SetParametersResult(successful=False, reason="duty_cycle cannot be negative.")
                self.duty_cycle = param.value  
            
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
