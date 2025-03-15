#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import random
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class PRBSControllerNode(Node):
    def __init__(self):
        super().__init__("PRBSControllerNode")
        
        # Declarar parámetros
        self.declare_parameter("timer_period", 0.1)                # Período de muestreo del timer (segundos)
        self.declare_parameter("prbs_interval", 1.0)                 # Intervalo para actualizar la señal PRBS (segundos)
        self.declare_parameter("base_duty_cycle", 8.0)               # Valor base de la amplitud del PWM
        self.declare_parameter("amplitude_increment", 0.1)           # Incremento relativo (10%) de la amplitud
        self.declare_parameter("amplitude_increment_period", 10.0)   # Intervalo para incrementar la amplitud (segundos)
        
        # Obtener los valores de los parámetros
        self.timer_period = self.get_parameter("timer_period").value
        self.prbs_interval = self.get_parameter("prbs_interval").value
        self.base_duty_cycle = self.get_parameter("base_duty_cycle").value
        self.amplitude_increment = self.get_parameter("amplitude_increment").value
        self.amplitude_increment_period = self.get_parameter("amplitude_increment_period").value
        
        # Inicializar variables internas
        self.current_amplitude = self.base_duty_cycle  # Amplitud actual (se irá incrementando)
        self.duty_cycle = self.base_duty_cycle         # Valor PWM actual (inicialmente positivo)
        self.last_prbs_time = self.get_clock().now()     # Marca de tiempo para la actualización PRBS
        self.last_amplitude_update_time = self.get_clock().now()  # Marca de tiempo para incremento de amplitud
        
        # Configurar el perfil QoS para el publicador
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        # Crear publicador para la señal PWM
        self.pwm_publisher = self.create_publisher(Float32, "pwm", qos_profile)
        # Crear timer que ejecuta el callback cada 'timer_period' segundos
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.signal_msg = Float32()
        
        # Permitir actualización dinámica de parámetros
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info("Nodo PRBS Controller iniciado 🚀")
    
    def timer_callback(self):
        now = self.get_clock().now()
        # Actualizar el valor de la señal PRBS si ha pasado el intervalo definido
        if (now - self.last_prbs_time).nanoseconds * 1e-9 >= self.prbs_interval:
            # Se elige aleatoriamente el signo para la amplitud actual (+ o -)
            self.duty_cycle = random.choice([self.current_amplitude, -self.current_amplitude])
            self.last_prbs_time = now
            self.get_logger().debug(f"Nuevo valor PRBS: {self.duty_cycle}")
        
        # Incrementar gradualmente la amplitud cada 'amplitude_increment_period' segundos
        if (now - self.last_amplitude_update_time).nanoseconds * 1e-9 >= self.amplitude_increment_period:
            self.current_amplitude *= (1 + self.amplitude_increment)
            # Limitar la amplitud máxima a 255
            if self.current_amplitude > 255:
                self.current_amplitude = 255
            self.last_amplitude_update_time = now
            self.get_logger().info(f"Amplitud incrementada a: {self.current_amplitude:.2f}")
        
        # Antes de publicar, restringir el valor del PWM a un máximo de 255 (en valor absoluto)
        max_val = 255.0
        duty_out = self.duty_cycle
        if duty_out > max_val:
            duty_out = max_val
        elif duty_out < -max_val:
            duty_out = -max_val
        
        self.signal_msg.data = duty_out
        self.pwm_publisher.publish(self.signal_msg)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == "timer_period":
                if param.value <= 0.0:
                    self.get_logger().warn("El período del timer debe ser positivo.")
                    return SetParametersResult(successful=False, reason="timer_period must be positive.")
                self.timer_period = param.value
                self.timer.timer_period = self.timer_period
            elif param.name == "prbs_interval":
                if param.value <= 0.0:
                    self.get_logger().warn("El intervalo PRBS debe ser positivo.")
                    return SetParametersResult(successful=False, reason="prbs_interval must be positive.")
                self.prbs_interval = param.value
            elif param.name == "base_duty_cycle":
                if param.value < 0.0:
                    self.get_logger().warn("La amplitud base no puede ser negativa.")
                    return SetParametersResult(successful=False, reason="base_duty_cycle cannot be negative.")
                self.base_duty_cycle = param.value
                self.current_amplitude = self.base_duty_cycle  # Reiniciar la amplitud actual
            elif param.name == "amplitude_increment":
                if param.value < 0.0:
                    self.get_logger().warn("El incremento de amplitud no puede ser negativo.")
                    return SetParametersResult(successful=False, reason="amplitude_increment cannot be negative.")
                self.amplitude_increment = param.value
            elif param.name == "amplitude_increment_period":
                if param.value <= 0.0:
                    self.get_logger().warn("El período de incremento de amplitud debe ser positivo.")
                    return SetParametersResult(successful=False, reason="amplitude_increment_period must be positive.")
                self.amplitude_increment_period = param.value
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = PRBSControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
