#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SetpointTrapezoidalPublisher(Node):
    def __init__(self):
        super().__init__('setpoint_trapezoidal_publisher')
        
        # Declarar el parámetro para el periodo del timer (en segundos)
        self.declare_parameter('timer_period', 0.1)
        self.timer_period = self.get_parameter('timer_period').value
        
        # Crear el publicador para el tópico /setpoint
        self.publisher = self.create_publisher(Float32, 'setpoint', 10)
        
        # Inicializar el timer para llamar al callback periódicamente
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Guardar el tiempo de inicio para calcular el tiempo transcurrido
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("SetpointTrapezoidalPublisher iniciado.")

    def timer_callback(self):
        # Calcular el tiempo transcurrido en segundos
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9
        
        msg = Float32()
        
        # Generar la señal según el perfil definido:
        if t < 5:
            msg.data = 4 * t              # Ramp up lineal: 0 -> 20
        elif t < 10:
            msg.data = 20                 # Velocidad constante: 20
        elif t < 15:
            msg.data = 20 + (t - 10)      # Ramp up lineal: 20 -> 25
        elif t < 20:
            msg.data = 25                 # Velocidad constante: 25
        elif t < 25:
            msg.data = 25 - 5 * (t - 20)    # Ramp down lineal: 25 -> 0
        elif t < 30:
            msg.data = 0                  # Velocidad cero: 0
        elif t < 35:
            msg.data = -5 * (t - 30)       # Ramp down lineal (negativo): 0 -> -25
        elif t < 50:
            msg.data = -25                # Velocidad constante (negativo): -25
        elif t < 55:
            msg.data = -25 + 5 * (t - 50)   # Ramp up lineal hacia cero: -25 -> 0
        else:
            msg.data = 0                  # Velocidad cero: 0
        
        # Publicar el mensaje en /setpoint
        self.publisher.publish(msg)
        self.get_logger().info(f"Tiempo: {t:.2f} s, Setpoint: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SetpointTrapezoidalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
